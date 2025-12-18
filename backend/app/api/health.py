"""Health check endpoint with service status monitoring."""

from fastapi import APIRouter, HTTPException, Depends
from sqlalchemy import text
from sqlalchemy.ext.asyncio import AsyncSession
from qdrant_client import QdrantClient
from qdrant_client.http.exceptions import UnexpectedResponse

from app.core.config import settings
from app.core.database import get_db

router = APIRouter()


async def check_database(db: AsyncSession) -> dict:
    """Check database connectivity."""
    try:
        await db.execute(text("SELECT 1"))
        return {"status": "healthy", "message": "Database connection successful"}
    except Exception as e:
        return {"status": "unhealthy", "message": f"Database error: {str(e)}"}


async def check_qdrant() -> dict:
    """Check Qdrant vector database connectivity."""
    try:
        client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
        )
        collections = client.get_collections()

        # Check if our collection exists
        collection_names = [col.name for col in collections.collections]
        collection_exists = settings.QDRANT_COLLECTION_NAME in collection_names

        return {
            "status": "healthy",
            "message": "Qdrant connection successful",
            "collection_exists": collection_exists,
            "total_collections": len(collection_names)
        }
    except UnexpectedResponse as e:
        return {"status": "unhealthy", "message": f"Qdrant error: {str(e)}"}
    except Exception as e:
        return {"status": "unhealthy", "message": f"Unexpected error: {str(e)}"}


@router.get("/health")
async def health_check(db: AsyncSession = Depends(get_db)):
    """
    Comprehensive health check endpoint.

    Checks:
    - API service status
    - Database connectivity
    - Qdrant vector database connectivity

    Returns:
        dict: Health status of all services
    """
    # Check database
    db_status = await check_database(db)

    # Check Qdrant
    qdrant_status = await check_qdrant()

    # Determine overall status
    all_healthy = (
        db_status["status"] == "healthy" and
        qdrant_status["status"] == "healthy"
    )

    response = {
        "status": "healthy" if all_healthy else "degraded",
        "services": {
            "api": {
                "status": "healthy",
                "version": settings.APP_VERSION
            },
            "database": db_status,
            "qdrant": qdrant_status
        }
    }

    # Return 503 if any service is unhealthy
    if not all_healthy:
        raise HTTPException(status_code=503, detail=response)

    return response


@router.get("/health/simple")
async def simple_health_check():
    """Simple health check without external service checks."""
    return {"status": "ok"}
