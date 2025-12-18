"""Authentication service for user signup, login, and JWT management."""

from datetime import datetime, timedelta
from typing import Optional
from jose import JWTError, jwt
from passlib.context import CryptContext
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from app.core.config import settings
from app.models.user import User
from app.schemas.auth import SignupRequest, LoginRequest, TokenResponse, UserResponse

# Password hashing context (bcrypt with 12 rounds)
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto", bcrypt__rounds=12)


class AuthService:
    """Service for authentication operations."""

    @staticmethod
    def hash_password(password: str) -> str:
        """Hash password using bcrypt (12 rounds)."""
        return pwd_context.hash(password)

    @staticmethod
    def verify_password(plain_password: str, hashed_password: str) -> bool:
        """Verify password against hash."""
        return pwd_context.verify(plain_password, hashed_password)

    @staticmethod
    def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
        """
        Create JWT access token.

        Args:
            data: Payload data to encode (should include 'sub' for user ID)
            expires_delta: Token expiration time (default: 1 hour from settings)

        Returns:
            Encoded JWT token
        """
        to_encode = data.copy()
        if expires_delta:
            expire = datetime.utcnow() + expires_delta
        else:
            expire = datetime.utcnow() + timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)

        to_encode.update({"exp": expire, "type": "access"})
        encoded_jwt = jwt.encode(to_encode, settings.JWT_SECRET_KEY, algorithm=settings.JWT_ALGORITHM)
        return encoded_jwt

    @staticmethod
    def create_refresh_token(data: dict) -> str:
        """
        Create JWT refresh token (7 days expiration).

        Args:
            data: Payload data to encode

        Returns:
            Encoded JWT refresh token
        """
        to_encode = data.copy()
        expire = datetime.utcnow() + timedelta(days=settings.REFRESH_TOKEN_EXPIRE_DAYS)
        to_encode.update({"exp": expire, "type": "refresh"})
        encoded_jwt = jwt.encode(to_encode, settings.REFRESH_SECRET_KEY, algorithm=settings.JWT_ALGORITHM)
        return encoded_jwt

    @staticmethod
    def verify_token(token: str, token_type: str = "access") -> Optional[dict]:
        """
        Verify JWT token and extract payload.

        Args:
            token: JWT token string
            token_type: "access" or "refresh"

        Returns:
            Decoded payload or None if invalid
        """
        try:
            secret_key = settings.JWT_SECRET_KEY if token_type == "access" else settings.REFRESH_SECRET_KEY
            payload = jwt.decode(token, secret_key, algorithms=[settings.JWT_ALGORITHM])

            if payload.get("type") != token_type:
                return None

            return payload
        except JWTError:
            return None

    @staticmethod
    async def signup(db: AsyncSession, signup_data: SignupRequest) -> User:
        """
        Create new user account.

        Args:
            db: Database session
            signup_data: User signup data (email, password)

        Returns:
            Created user instance

        Raises:
            ValueError: If email already exists
        """
        # Check if email already exists
        result = await db.execute(select(User).where(User.email == signup_data.email))
        existing_user = result.scalar_one_or_none()

        if existing_user:
            raise ValueError("Email already registered")

        # Create new user
        hashed_password = AuthService.hash_password(signup_data.password)
        new_user = User(
            email=signup_data.email,
            password_hash=hashed_password,
            role="user"
        )

        db.add(new_user)
        await db.commit()
        await db.refresh(new_user)

        return new_user

    @staticmethod
    async def login(db: AsyncSession, login_data: LoginRequest) -> TokenResponse:
        """
        Authenticate user and generate JWT tokens.

        Args:
            db: Database session
            login_data: User login credentials

        Returns:
            TokenResponse with access and refresh tokens

        Raises:
            ValueError: If credentials are invalid
        """
        # Find user by email
        result = await db.execute(select(User).where(User.email == login_data.email))
        user = result.scalar_one_or_none()

        if not user:
            raise ValueError("Invalid email or password")

        # Verify password
        if not AuthService.verify_password(login_data.password, user.password_hash):
            raise ValueError("Invalid email or password")

        # Check if user is active
        if not user.is_active:
            raise ValueError("Account is disabled")

        # Update last login
        user.last_login = datetime.utcnow()
        await db.commit()

        # Generate tokens
        access_token = AuthService.create_access_token(data={"sub": str(user.id), "role": user.role.value})
        refresh_token = AuthService.create_refresh_token(data={"sub": str(user.id)})

        return TokenResponse(
            access_token=access_token,
            refresh_token=refresh_token,
            token_type="bearer",
            expires_in=settings.ACCESS_TOKEN_EXPIRE_MINUTES * 60
        )

    @staticmethod
    async def get_current_user(db: AsyncSession, token: str) -> Optional[User]:
        """
        Get current user from JWT token.

        Args:
            db: Database session
            token: JWT access token

        Returns:
            User instance or None if invalid
        """
        payload = AuthService.verify_token(token, token_type="access")
        if not payload:
            return None

        user_id = payload.get("sub")
        if not user_id:
            return None

        result = await db.execute(select(User).where(User.id == int(user_id)))
        user = result.scalar_one_or_none()

        return user
