"""Authentication middleware for protecting routes with JWT verification."""

from fastapi import Depends, HTTPException, status, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from typing import Optional
from app.core.database import get_db
from app.models.user import User
from app.services.auth_service import AuthService

security = HTTPBearer(auto_error=False)


async def get_token_from_request(
    request: Request,
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security)
) -> Optional[str]:
    """
    Extract JWT token from Authorization header or httpOnly cookie.

    Priority:
    1. Authorization header (Bearer token)
    2. access_token cookie

    Returns:
        JWT token string or None if not found
    """
    # Try Authorization header first
    if credentials and credentials.scheme == "Bearer":
        return credentials.credentials

    # Try cookie
    cookie_token = request.cookies.get("access_token")
    if cookie_token:
        # Cookie format: "Bearer <token>"
        if cookie_token.startswith("Bearer "):
            return cookie_token[7:]  # Remove "Bearer " prefix
        return cookie_token

    return None


async def get_current_user(
    token: Optional[str] = Depends(get_token_from_request),
    db: AsyncSession = Depends(get_db)
) -> User:
    """
    Get current authenticated user from JWT token.

    Validates token and returns User instance.

    Raises:
        HTTPException 401: If token is missing, invalid, or user not found
    """
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"error": "unauthorized", "message": "Authentication required"},
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Verify token
    payload = AuthService.verify_token(token, token_type="access")
    if not payload:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"error": "invalid_token", "message": "Invalid or expired token"},
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Extract user ID from payload
    user_id_str = payload.get("sub")
    if not user_id_str:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"error": "invalid_token", "message": "Invalid token payload"},
            headers={"WWW-Authenticate": "Bearer"},
        )

    try:
        user_id = int(user_id_str)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"error": "invalid_token", "message": "Invalid user ID in token"},
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Get user from database
    result = await db.execute(select(User).where(User.id == user_id))
    user = result.scalar_one_or_none()

    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"error": "user_not_found", "message": "User not found"},
            headers={"WWW-Authenticate": "Bearer"},
        )

    return user


async def get_current_user_optional(
    token: Optional[str] = Depends(get_token_from_request),
    db: AsyncSession = Depends(get_db)
) -> Optional[User]:
    """
    Get current user if authenticated, otherwise None.

    Useful for endpoints that work for both authenticated and anonymous users.

    Returns:
        User instance if authenticated, None otherwise
    """
    if not token:
        return None

    try:
        payload = AuthService.verify_token(token, token_type="access")
        if not payload:
            return None

        user_id_str = payload.get("sub")
        if not user_id_str:
            return None

        user_id = int(user_id_str)
        result = await db.execute(select(User).where(User.id == user_id))
        user = result.scalar_one_or_none()

        return user
    except Exception:
        return None
