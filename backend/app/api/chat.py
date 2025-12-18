"""
Chat API endpoints.

Handles RAG-powered chatbot interactions and conversation history.
"""

import uuid
from typing import Optional
from datetime import datetime, timedelta
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, and_, desc

from app.schemas.chat import (
    ChatRequest,
    ChatResponse,
    ChatHistoryResponse,
    ChatHistoryMessage,
    TextSelectionRequest,
    ErrorResponse,
    Source
)
from app.services.rag_service import RAGService
from app.models.chat_message import ChatMessage
from app.core.database import get_db


router = APIRouter()


def get_rag_service() -> RAGService:
    """Dependency for RAG service."""
    return RAGService()


@router.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    db: AsyncSession = Depends(get_db),
    rag_service: RAGService = Depends(get_rag_service)
):
    """
    Process chat message and return RAG-powered response.

    Args:
        request: Chat request with message and optional session_id
        db: Database session
        rag_service: RAG service instance

    Returns:
        ChatResponse with answer and sources

    Raises:
        HTTPException: If RAG query fails
    """
    try:
        # Generate session_id if not provided
        session_id = request.session_id or str(uuid.uuid4())

        # Get conversation history if requested
        conversation_history = None
        if request.include_history:
            conversation_history = await _get_conversation_history(
                db=db,
                session_id=session_id,
                limit=6  # Last 3 exchanges
            )

        # Query RAG system
        result = await rag_service.query_rag(
            query=request.message,
            top_k=request.top_k,
            conversation_history=conversation_history
        )

        # Store message in database
        chat_message = ChatMessage(
            user_id=None,  # Anonymous for now
            session_id=session_id,
            message=request.message,
            response=result["answer"],
            sources=result["sources"],
            created_at=datetime.utcnow()
        )
        db.add(chat_message)
        await db.commit()

        # Build response
        response = ChatResponse(
            answer=result["answer"],
            sources=[Source(**source) for source in result["sources"]],
            session_id=session_id,
            timestamp=datetime.utcnow()
        )

        return response

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to process chat message: {str(e)}"
        )


@router.get("/chat/history", response_model=ChatHistoryResponse)
async def get_chat_history(
    session_id: str,
    limit: int = 20,
    db: AsyncSession = Depends(get_db)
):
    """
    Retrieve chat history for a session.

    Args:
        session_id: Session identifier
        limit: Maximum number of messages to return (default: 20)
        db: Database session

    Returns:
        ChatHistoryResponse with conversation messages

    Raises:
        HTTPException: If query fails
    """
    try:
        # Query messages for session
        query = (
            select(ChatMessage)
            .where(ChatMessage.session_id == session_id)
            .order_by(desc(ChatMessage.created_at))
            .limit(limit)
        )
        result = await db.execute(query)
        messages_db = result.scalars().all()

        # Convert to response format (reverse to chronological order)
        messages = []
        for msg in reversed(messages_db):
            # User message
            messages.append(
                ChatHistoryMessage(
                    role="user",
                    content=msg.message,
                    timestamp=msg.created_at
                )
            )
            # Assistant response
            sources = [Source(**src) for src in msg.sources] if msg.sources else []
            messages.append(
                ChatHistoryMessage(
                    role="assistant",
                    content=msg.response,
                    timestamp=msg.created_at,
                    sources=sources if sources else None
                )
            )

        return ChatHistoryResponse(
            messages=messages,
            session_id=session_id,
            total_messages=len(messages)
        )

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to retrieve chat history: {str(e)}"
        )


@router.delete("/chat/history/{session_id}")
async def clear_chat_history(
    session_id: str,
    db: AsyncSession = Depends(get_db)
):
    """
    Clear chat history for a session.

    Args:
        session_id: Session identifier to clear
        db: Database session

    Returns:
        Success message

    Raises:
        HTTPException: If deletion fails
    """
    try:
        # Delete all messages for session
        query = select(ChatMessage).where(ChatMessage.session_id == session_id)
        result = await db.execute(query)
        messages = result.scalars().all()

        for msg in messages:
            await db.delete(msg)

        await db.commit()

        return {
            "message": f"Cleared {len(messages)} messages for session {session_id}",
            "deleted_count": len(messages)
        }

    except Exception as e:
        await db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to clear chat history: {str(e)}"
        )


@router.post("/chat/selection", response_model=ChatResponse)
async def query_text_selection(
    request: TextSelectionRequest,
    db: AsyncSession = Depends(get_db),
    rag_service: RAGService = Depends(get_rag_service)
):
    """
    Process text selection query with additional context.

    Args:
        request: Text selection request with selected text and context
        db: Database session
        rag_service: RAG service instance

    Returns:
        ChatResponse with answer about selected text

    Raises:
        HTTPException: If query fails
    """
    try:
        # Build query from selected text
        query = f"Explain or elaborate on: {request.selected_text}"

        if request.surrounding_context:
            query += f"\n\nContext: {request.surrounding_context}"

        # Generate session_id if not provided
        session_id = request.session_id or str(uuid.uuid4())

        # Query RAG system
        result = await rag_service.query_rag(
            query=query,
            top_k=3,  # Fewer results for text selection
            conversation_history=None
        )

        # Store message in database
        chat_message = ChatMessage(
            user_id=None,
            session_id=session_id,
            message=f"[Text Selection] {request.selected_text}",
            response=result["answer"],
            sources=result["sources"],
            created_at=datetime.utcnow()
        )
        db.add(chat_message)
        await db.commit()

        # Build response
        response = ChatResponse(
            answer=result["answer"],
            sources=[Source(**source) for source in result["sources"]],
            session_id=session_id,
            timestamp=datetime.utcnow()
        )

        return response

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to process text selection query: {str(e)}"
        )


async def _get_conversation_history(
    db: AsyncSession,
    session_id: str,
    limit: int = 6
) -> list:
    """
    Internal helper to get conversation history in OpenAI format.

    Args:
        db: Database session
        session_id: Session identifier
        limit: Maximum number of messages to return

    Returns:
        List of message dicts in OpenAI format
    """
    query = (
        select(ChatMessage)
        .where(ChatMessage.session_id == session_id)
        .order_by(desc(ChatMessage.created_at))
        .limit(limit // 2)  # Divide by 2 since each record has user + assistant
    )
    result = await db.execute(query)
    messages_db = result.scalars().all()

    # Convert to OpenAI message format
    history = []
    for msg in reversed(messages_db):
        history.append({"role": "user", "content": msg.message})
        history.append({"role": "assistant", "content": msg.response})

    return history
