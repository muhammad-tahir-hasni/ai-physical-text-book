"""
Personalization API endpoints.

Handles dynamic content personalization based on user skill level.
"""

from fastapi import APIRouter, HTTPException, status
from app.schemas.personalization import PersonalizeRequest, PersonalizeResponse
from app.services.personalization_service import PersonalizationService

router = APIRouter()


@router.post("/personalize-chapter", response_model=PersonalizeResponse)
async def personalize_chapter(request: PersonalizeRequest):
    """
    Personalize chapter content to specified complexity level.

    Generates beginner/advanced versions using OpenAI GPT-3.5-turbo-16k.
    Results are cached for 24 hours to reduce API calls.

    Args:
        request: PersonalizeRequest with chapter_id and complexity_level

    Returns:
        PersonalizeResponse with personalized content and metadata

    Raises:
        HTTPException 404: If chapter not found
        HTTPException 400: If complexity level invalid
        HTTPException 500: If OpenAI API call fails
    """
    try:
        # Call personalization service
        content, was_cached, generation_time = await PersonalizationService.personalize_chapter(
            chapter_id=request.chapter_id,
            complexity_level=request.complexity_level
        )

        # Build response
        return PersonalizeResponse(
            content=content,
            cached=was_cached,
            generation_time_ms=generation_time,
            chapter_id=request.chapter_id,
            complexity_level=request.complexity_level
        )

    except FileNotFoundError as e:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=str(e)
        )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to personalize chapter: {str(e)}"
        )


@router.delete("/personalize-chapter/cache/{chapter_id}")
async def clear_chapter_cache(chapter_id: str):
    """
    Clear cached personalized versions for a chapter.

    Useful when chapter content is updated and cache needs invalidation.

    Args:
        chapter_id: Chapter identifier (e.g., "1-1", "2-2")

    Returns:
        Success message with number of entries cleared
    """
    try:
        cleared_count = await PersonalizationService.clear_cache_for_chapter(chapter_id)

        return {
            "message": f"Cleared cache for chapter {chapter_id}",
            "cleared_count": cleared_count
        }

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to clear cache: {str(e)}"
        )
