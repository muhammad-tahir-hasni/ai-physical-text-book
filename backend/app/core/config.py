"""Application configuration using Pydantic Settings."""

from typing import List
from pathlib import Path
from pydantic_settings import BaseSettings
from pydantic import field_validator


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Database
    DATABASE_URL: str

    # Redis
    REDIS_URL: str

    # JWT
    JWT_SECRET_KEY: str
    REFRESH_SECRET_KEY: str
    JWT_ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 60
    REFRESH_TOKEN_EXPIRE_DAYS: int = 7

    # Qdrant
    QDRANT_URL: str
    QDRANT_API_KEY: str
    QDRANT_COLLECTION_NAME: str = "hackathon_content"

    # Cohere
    COHERE_API_KEY: str
    COHERE_MODEL: str = "embed-english-v3.0"

    # LLM (Claude primary, OpenAI fallback)
    CLAUDE_API_KEY: str | None = None
    OPENAI_API_KEY: str | None = None

    # CORS
    CORS_ORIGINS: str = "http://localhost:3000,http://localhost:8000"

    # Application
    APP_NAME: str = "Physical AI Hackathon Platform"
    APP_VERSION: str = "1.0.0"
    DEBUG: bool = True

    @field_validator("CORS_ORIGINS")
    @classmethod
    def parse_cors_origins(cls, v: str) -> List[str]:
        """Parse comma-separated CORS origins."""
        return [origin.strip() for origin in v.split(",")]

    class Config:
        # Look for .env in the backend directory
        backend_dir = Path(__file__).parent.parent.parent
        env_file = backend_dir / ".env"
        case_sensitive = True


# Global settings instance
settings = Settings()
