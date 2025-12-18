"""Authentication API endpoints for signup, login, logout, and token refresh."""

from fastapi import APIRouter, Depends, HTTPException, status, Response
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from app.core.database import get_db
from app.models.user import User
from app.models.user_profile import UserProfile
from app.schemas.auth import SignupRequest, LoginRequest, TokenResponse
from app.services.auth_service import AuthService
from app.utils.email_validator import validate_email_domain, is_valid_professional_email
from pydantic import BaseModel, EmailStr, Field
from typing import Optional

router = APIRouter()
security = HTTPBearer()


class SignupWithProfileRequest(BaseModel):
    """Extended signup request with profile data."""
    email: EmailStr
    password: str = Field(..., min_length=8, max_length=100)
    profile: Optional[dict] = None  # Optional profile data


class UserProfileData(BaseModel):
    """User profile data for background questionnaire."""
    software_experience: Optional[str] = "intermediate"
    python_familiarity: Optional[str] = "basic"
    robotics_experience: Optional[str] = "none"
    hardware_background: Optional[str] = "none"
    learning_goals: Optional[str] = ""
    preferred_complexity: str = "intermediate"


@router.post("/auth/signup", response_model=dict, status_code=status.HTTP_201_CREATED)
async def signup(
    request: SignupWithProfileRequest,
    response: Response,
    db: AsyncSession = Depends(get_db)
):
    """
    Create new user account with optional profile data.

    - **email**: Valid email address (unique)
    - **password**: Min 8 characters, 1 uppercase, 1 number
    - **profile**: Optional background questionnaire data

    Returns JWT access token and refresh token.
    """
    try:
        # Validate email domain is real and not disposable
        is_valid, error_msg = validate_email_domain(request.email)
        if not is_valid:
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail={"error": "invalid_email", "message": error_msg}
            )

        # Check if email already exists
        result = await db.execute(select(User).where(User.email == request.email))
        existing_user = result.scalar_one_or_none()

        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail={"error": "email_exists", "message": "Email already registered"}
            )

        # Validate password strength (min 8 chars, 1 uppercase, 1 number)
        if not any(c.isupper() for c in request.password):
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail={"error": "weak_password", "message": "Password must contain at least one uppercase letter"}
            )
        if not any(c.isdigit() for c in request.password):
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail={"error": "weak_password", "message": "Password must contain at least one number"}
            )

        # Hash password and create user
        hashed_password = AuthService.hash_password(request.password)
        new_user = User(
            email=request.email,
            password_hash=hashed_password
        )

        db.add(new_user)
        await db.flush()  # Get user ID before creating profile

        # Create user profile if provided
        if request.profile:
            profile_data = UserProfileData(**request.profile)
            user_profile = UserProfile(
                user_id=new_user.id,
                software_experience=profile_data.software_experience,
                python_familiarity=profile_data.python_familiarity,
                robotics_experience=profile_data.robotics_experience,
                hardware_background=profile_data.hardware_background,
                learning_goals=profile_data.learning_goals,
                preferred_complexity=profile_data.preferred_complexity
            )
            db.add(user_profile)

        await db.commit()
        await db.refresh(new_user)

        # Generate JWT tokens
        access_token = AuthService.create_access_token(data={"sub": str(new_user.id)})
        refresh_token = AuthService.create_refresh_token(data={"sub": str(new_user.id)})

        # Set httpOnly cookie for access token (optional, for added security)
        response.set_cookie(
            key="access_token",
            value=f"Bearer {access_token}",
            httponly=True,
            secure=True,  # HTTPS only
            samesite="lax",
            max_age=3600  # 1 hour
        )

        return {
            "user": {
                "id": new_user.id,
                "email": new_user.email,
                "created_at": new_user.created_at.isoformat()
            },
            "access_token": access_token,
            "refresh_token": refresh_token,
            "token_type": "bearer",
            "expires_in": 3600
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "server_error", "message": str(e)}
        )


@router.post("/auth/login", response_model=dict)
async def login(
    request: LoginRequest,
    response: Response,
    db: AsyncSession = Depends(get_db)
):
    """
    Authenticate user and return JWT tokens.

    - **email**: User email address
    - **password**: User password
    - **remember_me**: Extend session to 7 days (optional)

    Returns access token and refresh token.
    """
    try:
        # Find user by email
        result = await db.execute(select(User).where(User.email == request.email))
        user = result.scalar_one_or_none()

        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={"error": "invalid_credentials", "message": "Incorrect email or password"}
            )

        # Verify password
        password_valid = AuthService.verify_password(request.password, user.password_hash)
        print(f"🔐 Password verification for {request.email}: {password_valid}")  # Debug log

        if not password_valid:
            print(f"❌ Login failed for {request.email}: Invalid password")  # Debug log
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={"error": "invalid_credentials", "message": "Incorrect email or password"}
            )

        # Generate JWT tokens
        access_token = AuthService.create_access_token(data={"sub": str(user.id)})
        refresh_token = AuthService.create_refresh_token(data={"sub": str(user.id)})

        # Set httpOnly cookie
        response.set_cookie(
            key="access_token",
            value=f"Bearer {access_token}",
            httponly=True,
            secure=True,
            samesite="lax",
            max_age=3600
        )

        return {
            "user": {
                "id": user.id,
                "email": user.email
            },
            "access_token": access_token,
            "refresh_token": refresh_token,
            "token_type": "bearer",
            "expires_in": 3600
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "server_error", "message": str(e)}
        )


@router.post("/auth/logout", status_code=status.HTTP_204_NO_CONTENT)
async def logout(response: Response):
    """
    Logout user by clearing httpOnly cookie.

    Note: JWTs are stateless, so this only clears the cookie.
    For true token blacklisting, implement a token blacklist database.
    """
    response.delete_cookie(key="access_token")
    return None


@router.post("/auth/refresh", response_model=dict)
async def refresh_token(
    refresh_token: str,
    response: Response
):
    """
    Refresh access token using valid refresh token.

    - **refresh_token**: Valid JWT refresh token

    Returns new access token and refresh token.
    """
    try:
        # Verify refresh token
        payload = AuthService.verify_token(refresh_token, token_type="refresh")
        if not payload:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={"error": "invalid_token", "message": "Invalid or expired refresh token"}
            )

        user_id = payload.get("sub")
        if not user_id:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail={"error": "invalid_token", "message": "Invalid token payload"}
            )

        # Generate new tokens
        new_access_token = AuthService.create_access_token(data={"sub": user_id})
        new_refresh_token = AuthService.create_refresh_token(data={"sub": user_id})

        # Update cookie
        response.set_cookie(
            key="access_token",
            value=f"Bearer {new_access_token}",
            httponly=True,
            secure=True,
            samesite="lax",
            max_age=3600
        )

        return {
            "access_token": new_access_token,
            "refresh_token": new_refresh_token,
            "token_type": "bearer",
            "expires_in": 3600
        }

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "server_error", "message": str(e)}
        )
