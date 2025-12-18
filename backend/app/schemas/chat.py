"""
Pydantic schemas for chat API.

Defines request/response models for the RAG chatbot endpoints.
"""

from typing import List, Optional, Dict, Any
from datetime import datetime
from pydantic import BaseModel, Field, field_validator


class Source(BaseModel):
    """Source citation for RAG response."""

    title: str = Field(..., description="Chapter title")
    heading: str = Field(..., description="Section heading within chapter")
    score: float = Field(..., ge=0, le=1, description="Relevance score (0-1)")


class ChatRequest(BaseModel):
    """Request for chat endpoint."""

    message: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User's question or message"
    )
    session_id: Optional[str] = Field(
        None,
        description="Session ID for conversation continuity (generated if not provided)"
    )
    include_history: bool = Field(
        True,
        description="Whether to include conversation history for context"
    )
    top_k: int = Field(
        5,
        ge=1,
        le=10,
        description="Number of relevant chunks to retrieve"
    )

    @field_validator("message")
    @classmethod
    def validate_message(cls, v: str) -> str:
        """Validate and clean message."""
        v = v.strip()
        if not v:
            raise ValueError("Message cannot be empty or whitespace only")
        return v


class ChatResponse(BaseModel):
    """Response from chat endpoint."""

    answer: str = Field(..., description="Generated answer from RAG system")
    sources: List[Source] = Field(
        default_factory=list,
        description="Source citations used to generate answer"
    )
    session_id: str = Field(..., description="Session ID for this conversation")
    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Response timestamp"
    )


class ChatHistoryMessage(BaseModel):
    """Single message in chat history."""

    role: str = Field(..., description="Role: 'user' or 'assistant'")
    content: str = Field(..., description="Message content")
    timestamp: datetime = Field(..., description="Message timestamp")
    sources: Optional[List[Source]] = Field(
        None,
        description="Sources for assistant messages"
    )


class ChatHistoryResponse(BaseModel):
    """Response for chat history endpoint."""

    messages: List[ChatHistoryMessage] = Field(
        default_factory=list,
        description="Conversation history"
    )
    session_id: str = Field(..., description="Session ID")
    total_messages: int = Field(..., ge=0, description="Total messages in history")


class TextSelectionRequest(BaseModel):
    """Request for text selection query."""

    selected_text: str = Field(
        ...,
        min_length=1,
        max_length=500,
        description="Text selected by user"
    )
    surrounding_context: Optional[str] = Field(
        None,
        max_length=1000,
        description="Surrounding text for additional context"
    )
    page_title: Optional[str] = Field(
        None,
        description="Title of the page where text was selected"
    )
    session_id: Optional[str] = Field(
        None,
        description="Session ID for conversation continuity"
    )


class ErrorResponse(BaseModel):
    """Error response schema."""

    error: str = Field(..., description="Error message")
    detail: Optional[str] = Field(None, description="Detailed error information")
    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Error timestamp"
    )


class HealthCheckResponse(BaseModel):
    """Health check response."""

    status: str = Field(..., description="Overall health status")
    services: Dict[str, Dict[str, Any]] = Field(
        ...,
        description="Status of individual services"
    )
    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Health check timestamp"
    )
