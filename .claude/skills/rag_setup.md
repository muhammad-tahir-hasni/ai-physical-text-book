# RAG Setup Skill

## Purpose

The RAG (Retrieval-Augmented Generation) Setup skill automates the complete workflow for ingesting documentation into a vector database, enabling semantic search and context-aware chatbot responses. This skill handles document chunking, embedding generation, and Qdrant vector database population.

## Overview

This skill transforms a collection of markdown documentation files into a searchable knowledge base that powers the RAG chatbot. It handles:

1. **Document Discovery**: Recursively find all markdown files in a directory
2. **Intelligent Chunking**: Split documents into semantically meaningful chunks
3. **Embedding Generation**: Create vector embeddings using OpenAI's text-embedding-3-small
4. **Vector Storage**: Store embeddings in Qdrant with metadata for retrieval
5. **Validation**: Verify ingestion success and search functionality

## When to Use

- **Initial Setup**: First-time setup of RAG knowledge base
- **Content Updates**: Re-ingest documentation after adding new chapters
- **Collection Reset**: Clear and rebuild vector database from scratch
- **Schema Changes**: Update metadata structure or chunking strategy

## Prerequisites

```bash
# Environment variables required
export OPENAI_API_KEY="sk-..."
export QDRANT_API_KEY="..."
export QDRANT_URL="https://..."
export QDRANT_COLLECTION_NAME="physical_ai_docs"

# Python dependencies
pip install openai qdrant-client python-dotenv
```

## Skill Workflow

### Phase 1: Document Discovery

```python
"""
Recursively scan documentation directory for markdown files.
Filter out non-content files (README, navigation configs).
"""

import os
from pathlib import Path
from typing import List

def discover_documents(docs_dir: str) -> List[Path]:
    """Find all markdown files in documentation directory."""
    docs_path = Path(docs_dir)
    markdown_files = []

    for file in docs_path.rglob("*.md"):
        # Skip navigation and config files
        if file.name.lower() in ["readme.md", "_category_.json"]:
            continue
        markdown_files.append(file)

    print(f"✓ Found {len(markdown_files)} markdown files")
    return markdown_files
```

### Phase 2: Intelligent Chunking

```python
"""
Split documents into chunks with semantic boundaries.
Preserve code blocks, maintain context with overlap.
"""

def chunk_document(content: str, chunk_size: int = 800, overlap: int = 100) -> List[dict]:
    """
    Chunk document intelligently at section boundaries.

    Strategy:
    1. Split on markdown headers (##, ###)
    2. Keep chunks under chunk_size characters
    3. Maintain overlap for context continuity
    4. Preserve code blocks intact
    """
    chunks = []
    sections = content.split('\n## ')  # Split on level-2 headers

    for section in sections:
        if len(section) < chunk_size:
            chunks.append({"text": section, "type": "section"})
        else:
            # Further split large sections
            sub_chunks = split_with_overlap(section, chunk_size, overlap)
            chunks.extend(sub_chunks)

    return chunks
```

### Phase 3: Embedding Generation

```python
"""
Generate vector embeddings for each chunk using OpenAI.
Batch requests for efficiency.
"""

from openai import OpenAI

def generate_embeddings(chunks: List[str], batch_size: int = 100) -> List[List[float]]:
    """Generate embeddings in batches for efficiency."""
    client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    embeddings = []

    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i + batch_size]

        response = client.embeddings.create(
            model="text-embedding-3-small",  # 1536 dimensions
            input=batch
        )

        batch_embeddings = [item.embedding for item in response.data]
        embeddings.extend(batch_embeddings)

        print(f"✓ Generated embeddings for batch {i//batch_size + 1}")

    return embeddings
```

### Phase 4: Vector Storage

```python
"""
Store embeddings in Qdrant with rich metadata.
"""

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

def store_in_qdrant(chunks: List[dict], embeddings: List[List[float]]):
    """Store chunk embeddings in Qdrant collection."""
    client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_docs")

    # Create collection if doesn't exist
    try:
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=1536,  # text-embedding-3-small dimension
                distance=Distance.COSINE
            )
        )
        print(f"✓ Created collection: {collection_name}")
    except Exception as e:
        print(f"Collection already exists: {e}")

    # Prepare points with metadata
    points = []
    for idx, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        point = PointStruct(
            id=idx,
            vector=embedding,
            payload={
                "text": chunk["text"],
                "file_path": chunk["file_path"],
                "chapter_id": chunk.get("chapter_id"),
                "module": chunk.get("module"),
                "section": chunk.get("section"),
                "chunk_index": idx
            }
        )
        points.append(point)

    # Batch upload
    client.upsert(collection_name=collection_name, points=points)
    print(f"✓ Uploaded {len(points)} points to Qdrant")
```

### Phase 5: Validation

```python
"""
Verify ingestion success with test queries.
"""

def validate_ingestion():
    """Test search functionality with sample queries."""
    test_queries = [
        "What is ROS 2?",
        "How to create a ROS 2 node?",
        "Sensor integration in robotics"
    ]

    client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_docs")

    for query in test_queries:
        # Generate query embedding
        response = openai_client.embeddings.create(
            model="text-embedding-3-small",
            input=query
        )
        query_embedding = response.data[0].embedding

        # Search Qdrant
        results = client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=3
        )

        print(f"\nQuery: {query}")
        print(f"✓ Found {len(results)} results")
        for result in results:
            print(f"  - Score: {result.score:.3f} | {result.payload['file_path']}")
```

## Complete Automation Script

```python
#!/usr/bin/env python3
"""
RAG Setup Automation Script

Usage:
    python scripts/setup_rag.py --docs-dir frontend/docs --reset
"""

import argparse
import os
from pathlib import Path
from dotenv import load_dotenv

# Import all helper functions from phases above

def main():
    """Main RAG setup workflow."""
    load_dotenv()

    parser = argparse.ArgumentParser(description="Setup RAG knowledge base")
    parser.add_argument("--docs-dir", required=True, help="Path to documentation directory")
    parser.add_argument("--reset", action="store_true", help="Reset collection before ingestion")
    parser.add_argument("--chunk-size", type=int, default=800, help="Chunk size in characters")
    parser.add_argument("--overlap", type=int, default=100, help="Chunk overlap in characters")
    args = parser.parse_args()

    print("🚀 Starting RAG Setup...")

    # Phase 1: Discovery
    print("\n📁 Phase 1: Document Discovery")
    files = discover_documents(args.docs_dir)

    # Phase 2: Chunking
    print("\n✂️  Phase 2: Intelligent Chunking")
    all_chunks = []
    for file in files:
        content = file.read_text(encoding="utf-8")
        chunks = chunk_document(content, args.chunk_size, args.overlap)

        # Add metadata
        for chunk in chunks:
            chunk["file_path"] = str(file)
            chunk["module"] = extract_module(file)
            chunk["chapter_id"] = extract_chapter_id(file)

        all_chunks.extend(chunks)

    print(f"✓ Created {len(all_chunks)} chunks")

    # Phase 3: Embeddings
    print("\n🔢 Phase 3: Embedding Generation")
    texts = [chunk["text"] for chunk in all_chunks]
    embeddings = generate_embeddings(texts)

    # Phase 4: Storage
    print("\n💾 Phase 4: Vector Storage")
    if args.reset:
        print("⚠️  Resetting collection...")
        # Delete and recreate collection

    store_in_qdrant(all_chunks, embeddings)

    # Phase 5: Validation
    print("\n✅ Phase 5: Validation")
    validate_ingestion()

    print("\n🎉 RAG Setup Complete!")
    print(f"   • Documents: {len(files)}")
    print(f"   • Chunks: {len(all_chunks)}")
    print(f"   • Vector dimension: 1536")
    print(f"   • Collection: {os.getenv('QDRANT_COLLECTION_NAME')}")

if __name__ == "__main__":
    main()
```

## Usage Examples

### Initial Setup

```bash
# First-time RAG knowledge base creation
python scripts/setup_rag.py \
    --docs-dir frontend/docs \
    --chunk-size 800 \
    --overlap 100
```

### Reset and Rebuild

```bash
# Clear existing data and rebuild from scratch
python scripts/setup_rag.py \
    --docs-dir frontend/docs \
    --reset
```

### Update Specific Modules

```bash
# Re-ingest only Module 3 chapters
python scripts/setup_rag.py \
    --docs-dir frontend/docs/module-3 \
    --reset
```

## Best Practices

1. **Chunking Strategy**:
   - Respect markdown section boundaries (##, ###)
   - Keep code blocks intact
   - Maintain 100-200 character overlap for context

2. **Embedding Efficiency**:
   - Batch API requests (100 chunks per batch)
   - Use async/await for parallel processing
   - Cache embeddings for unchanged documents

3. **Metadata Design**:
   - Store file path for source attribution
   - Include chapter_id for filtering
   - Add module for hierarchical search
   - Preserve section headers for context

4. **Error Handling**:
   - Retry failed API calls with exponential backoff
   - Log failed chunks for manual review
   - Validate embeddings dimensions (1536)

## Monitoring and Maintenance

### Collection Statistics

```python
def get_collection_stats():
    """Print Qdrant collection statistics."""
    client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    collection_name = os.getenv("QDRANT_COLLECTION_NAME")
    info = client.get_collection(collection_name)

    print(f"Collection: {collection_name}")
    print(f"Points: {info.points_count}")
    print(f"Vectors: {info.vectors_count}")
    print(f"Dimension: {info.config.params.vectors.size}")
```

### Search Quality Check

```bash
# Test search quality with diverse queries
python scripts/test_rag_quality.py \
    --queries test_queries.txt \
    --threshold 0.7
```

## Integration with Project

This skill is essential for:

- **Phase 4 (RAG Chatbot)**: Powers semantic search for chatbot responses
- **Content Generator**: Validates generated chapters are searchable
- **Personalization**: Ensures personalized content is indexed
- **Quality Assurance**: Verifies documentation completeness

## Troubleshooting

### Common Issues

1. **Rate Limit Errors (429)**:
   - Reduce batch size
   - Add sleep between batches
   - Use exponential backoff

2. **Dimension Mismatch**:
   - Verify embedding model is text-embedding-3-small (1536d)
   - Check Qdrant collection vector size

3. **Low Search Quality**:
   - Increase chunk overlap
   - Adjust chunk size (600-1000 optimal)
   - Add more metadata for filtering

## Related Skills

- **Auth Setup**: Similar automation pattern for authentication
- **Content Generator**: Produces documents to be ingested
- **Diagram Describer**: Diagrams can be included in searchable content
