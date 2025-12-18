"""User profile API endpoints for viewing and updating user profiles."""

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from app.core.database import get_db
from app.models.user import User
from app.models.user_profile import UserProfile
from app.middleware.auth import get_current_user
from pydantic import BaseModel
from typing import Optional

router = APIRouter()


class UserProfileResponse(BaseModel):
    """Response schema for user profile."""
    user_id: int
    email: str
    profile: Optional[dict] = None

    class Config:
        from_attributes = True


class UpdateProfileRequest(BaseModel):
    """Request schema for updating user profile."""
    software_experience: Optional[str] = None
    python_familiarity: Optional[str] = None
    robotics_experience: Optional[str] = None
    hardware_background: Optional[str] = None
    learning_goals: Optional[str] = None
    preferred_complexity: Optional[str] = None


@router.get("/user/profile", response_model=UserProfileResponse)
async def get_profile(
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """
    Get current user's profile.

    Requires authentication (JWT token in Authorization header or cookie).

    Returns user data and profile information.
    """
    try:
        # Get user profile
        result = await db.execute(
            select(UserProfile).where(UserProfile.user_id == current_user.id)
        )
        profile = result.scalar_one_or_none()

        profile_data = None
        if profile:
            profile_data = {
                "software_experience": profile.software_experience,
                "python_familiarity": profile.python_familiarity,
                "robotics_experience": profile.robotics_experience,
                "hardware_background": profile.hardware_background,
                "learning_goals": profile.learning_goals,
                "preferred_complexity": profile.preferred_complexity
            }

        return UserProfileResponse(
            user_id=current_user.id,
            email=current_user.email,
            profile=profile_data
        )

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "server_error", "message": str(e)}
        )


@router.patch("/user/profile", response_model=UserProfileResponse)
async def update_profile(
    request: UpdateProfileRequest,
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """
    Update current user's profile.

    Requires authentication (JWT token in Authorization header or cookie).

    All fields are optional - only provided fields will be updated.
    """
    try:
        # Get existing profile
        result = await db.execute(
            select(UserProfile).where(UserProfile.user_id == current_user.id)
        )
        profile = result.scalar_one_or_none()

        # Create profile if doesn't exist
        if not profile:
            profile = UserProfile(user_id=current_user.id)
            db.add(profile)

        # Update fields (only if provided)
        update_data = request.model_dump(exclude_unset=True)
        for field, value in update_data.items():
            setattr(profile, field, value)

        await db.commit()
        await db.refresh(profile)

        return UserProfileResponse(
            user_id=current_user.id,
            email=current_user.email,
            profile={
                "software_experience": profile.software_experience,
                "python_familiarity": profile.python_familiarity,
                "robotics_experience": profile.robotics_experience,
                "hardware_background": profile.hardware_background,
                "learning_goals": profile.learning_goals,
                "preferred_complexity": profile.preferred_complexity
            }
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "server_error", "message": str(e)}
        )
