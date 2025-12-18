"""Pydantic schemas for content personalization requests and responses."""

from pydantic import BaseModel, Field
from typing import Literal


class PersonalizeRequest(BaseModel):
    """Request schema for personalizing chapter content."""
    chapter_id: str = Field(
        ...,
        description="Chapter identifier (e.g., '1-1', '2-2')",
        pattern=r"^\d+-\d+$",
        examples=["1-1", "2-2", "3-1"]
    )
    complexity_level: Literal["beginner", "intermediate", "advanced"] = Field(
        ...,
        description="Desired complexity level for personalized content"
    )


class PersonalizeResponse(BaseModel):
    """Response schema for personalized chapter content."""
    content: str = Field(
        ...,
        description="Personalized chapter content in Markdown format"
    )
    cached: bool = Field(
        default=False,
        description="Whether content was served from cache"
    )
    generation_time_ms: int = Field(
        default=0,
        description="Time taken to generate content (0 if cached)"
    )
    chapter_id: str = Field(
        ...,
        description="Chapter identifier that was personalized"
    )
    complexity_level: str = Field(
        ...,
        description="Complexity level applied"
    )
