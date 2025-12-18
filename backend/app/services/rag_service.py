"""
RAG (Retrieval-Augmented Generation) service.

Handles document ingestion, embedding generation, semantic search, and LLM response generation.
"""

import asyncio
import hashlib
import time
from typing import List, Dict, Optional
import openai
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, Filter, FieldCondition, MatchValue

from app.core.config import settings
from app.utils.chunking import chunk_markdown


class RAGService:
    """Service for RAG operations: ingestion, search, and generation."""

    def __init__(self):
        self.openai_client = openai.AsyncOpenAI(api_key=settings.OPENAI_API_KEY)
        self.qdrant_client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY
        )
        self.collection_name = settings.QDRANT_COLLECTION_NAME
        self.embedding_model = "text-embedding-3-small"
        self.chat_model = "gpt-3.5-turbo"

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding vector for text using OpenAI.

        Args:
            text: Text to embed

        Returns:
            1536-dimensional embedding vector
        """
        response = await self.openai_client.embeddings.create(
            model=self.embedding_model,
            input=text
        )
        return response.data[0].embedding

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in batch.

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors
        """
        # OpenAI allows up to 2048 texts per request
        if len(texts) > 2048:
            # Process in batches
            embeddings = []
            for i in range(0, len(texts), 2048):
                batch = texts[i:i+2048]
                batch_embeddings = await self.generate_embeddings_batch(batch)
                embeddings.extend(batch_embeddings)
            return embeddings

        response = await self.openai_client.embeddings.create(
            model=self.embedding_model,
            input=texts
        )
        return [item.embedding for item in response.data]

    async def ingest_document(
        self,
        document_id: str,
        title: str,
        markdown_content: str,
        metadata: Optional[Dict] = None
    ) -> int:
        """
        Ingest a document into the vector database.

        Args:
            document_id: Unique identifier for document
            title: Document title
            markdown_content: Markdown content to ingest
            metadata: Additional metadata (module, chapter, etc.)

        Returns:
            Number of chunks ingested
        """
        # Chunk the document
        chunks = chunk_markdown(
            markdown_content,
            chunk_size=512,
            overlap=50,
            preserve_structure=True
        )

        if not chunks:
            return 0

        # Generate embeddings for all chunks
        chunk_texts = [chunk['text'] for chunk in chunks]
        embeddings = await self.generate_embeddings_batch(chunk_texts)

        # Prepare points for Qdrant
        points = []
        for idx, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            # Generate unique ID for chunk
            chunk_id = hashlib.md5(
                f"{document_id}_{idx}_{chunk['text'][:50]}".encode()
            ).hexdigest()

            point = PointStruct(
                id=chunk_id,
                vector=embedding,
                payload={
                    "document_id": document_id,
                    "title": title,
                    "chunk_index": idx,
                    "text": chunk['text'],
                    "heading": chunk['metadata']['heading'],
                    "level": chunk['metadata']['level'],
                    **(metadata or {})
                }
            )
            points.append(point)

        # Upsert to Qdrant
        self.qdrant_client.upsert(
            collection_name=self.collection_name,
            points=points
        )

        return len(points)

    async def search_similar(
        self,
        query: str,
        top_k: int = 5,
        filter_conditions: Optional[Dict] = None,
        score_threshold: float = 0.7
    ) -> List[Dict]:
        """
        Search for similar chunks using semantic search.

        Args:
            query: Search query
            top_k: Number of results to return
            filter_conditions: Optional filters (e.g., {"module": "Module 1"})
            score_threshold: Minimum similarity score (0-1)

        Returns:
            List of search results with text, metadata, and scores
        """
        # Generate query embedding
        query_embedding = await self.generate_embedding(query)

        # Build filter if provided
        qdrant_filter = None
        if filter_conditions:
            conditions = [
                FieldCondition(
                    key=key,
                    match=MatchValue(value=value)
                )
                for key, value in filter_conditions.items()
            ]
            qdrant_filter = Filter(must=conditions)

        # Search in Qdrant
        search_results = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=top_k,
            query_filter=qdrant_filter,
            score_threshold=score_threshold
        )

        # Format results
        results = []
        for result in search_results:
            results.append({
                "text": result.payload["text"],
                "title": result.payload["title"],
                "heading": result.payload.get("heading", ""),
                "document_id": result.payload["document_id"],
                "score": result.score,
                "metadata": {
                    key: value for key, value in result.payload.items()
                    if key not in ["text", "title", "heading", "document_id"]
                }
            })

        return results

    async def generate_answer(
        self,
        query: str,
        context_chunks: List[Dict],
        conversation_history: Optional[List[Dict]] = None,
        max_tokens: int = 1000
    ) -> Dict:
        """
        Generate answer using LLM with retrieved context.

        Args:
            query: User question
            context_chunks: Retrieved context from vector search
            conversation_history: Previous messages for context
            max_tokens: Maximum tokens in response

        Returns:
            Dict with 'answer' and 'sources'
        """
        # Build context from chunks
        context_text = "\n\n".join([
            f"[{chunk['title']} - {chunk['heading']}]\n{chunk['text']}"
            for chunk in context_chunks
        ])

        # Build system prompt
        system_prompt = f"""You are an expert teaching assistant for a Physical AI and Humanoid Robotics course.
Your goal is to provide comprehensive, detailed answers based on the course content.

Course Context:
{context_text}

Guidelines:
- Provide detailed, thorough explanations based on the context
- Even if the user asks about a single word or concept, explain it comprehensively
- Include relevant examples, code snippets, and technical details from the context
- Cite specific chapters/sections when answering
- If a concept appears in multiple chapters, synthesize information from all relevant sections
- If information is not in the context, clearly state that
- Make your answers educational and complete, not just brief summaries"""

        # Build messages
        messages = [{"role": "system", "content": system_prompt}]

        # Add conversation history if provided
        if conversation_history:
            messages.extend(conversation_history[-6:])  # Last 3 exchanges

        # Add current query
        messages.append({"role": "user", "content": query})

        # Generate response with retry logic
        max_retries = 3
        for attempt in range(max_retries):
            try:
                response = await self.openai_client.chat.completions.create(
                    model=self.chat_model,
                    messages=messages,
                    max_tokens=max_tokens,
                    temperature=0.7
                )
                break
            except openai.RateLimitError:
                if attempt < max_retries - 1:
                    wait_time = 2 ** attempt  # Exponential backoff
                    await asyncio.sleep(wait_time)
                else:
                    raise
            except Exception as e:
                if attempt < max_retries - 1:
                    await asyncio.sleep(1)
                else:
                    raise

        answer = response.choices[0].message.content

        # Format sources
        sources = [
            {
                "title": chunk["title"],
                "heading": chunk["heading"],
                "score": round(chunk["score"], 3)
            }
            for chunk in context_chunks[:3]  # Top 3 sources
        ]

        return {
            "answer": answer,
            "sources": sources
        }

    async def query_rag(
        self,
        query: str,
        top_k: int = 12,
        conversation_history: Optional[List[Dict]] = None
    ) -> Dict:
        """
        End-to-end RAG query: search + generate.

        Args:
            query: User question
            top_k: Number of context chunks to retrieve (default: 12 for comprehensive answers)
            conversation_history: Previous messages

        Returns:
            Dict with 'answer' and 'sources'
        """
        # Search for relevant context with lower threshold for better recall
        context_chunks = await self.search_similar(
            query=query,
            top_k=top_k,
            score_threshold=0.5
        )

        if not context_chunks:
            return {
                "answer": "I couldn't find relevant information in the course content to answer your question. Please try rephrasing or ask about topics covered in the Physical AI and Robotics course.",
                "sources": []
            }

        # Generate answer with context
        result = await self.generate_answer(
            query=query,
            context_chunks=context_chunks,
            conversation_history=conversation_history
        )

        return result

    def delete_document(self, document_id: str) -> bool:
        """
        Delete all chunks for a document.

        Args:
            document_id: Document identifier

        Returns:
            True if successful
        """
        # Delete by filter
        self.qdrant_client.delete(
            collection_name=self.collection_name,
            points_selector=Filter(
                must=[
                    FieldCondition(
                        key="document_id",
                        match=MatchValue(value=document_id)
                    )
                ]
            )
        )
        return True

    def get_collection_stats(self) -> Dict:
        """Get statistics about the Qdrant collection."""
        collection_info = self.qdrant_client.get_collection(self.collection_name)
        return {
            "vectors_count": collection_info.vectors_count,
            "points_count": collection_info.points_count,
            "status": collection_info.status
        }
