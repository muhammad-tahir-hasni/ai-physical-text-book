"""Qdrant collection initialization utility."""

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, OptimizersConfigDiff, PointStruct
from qdrant_client.http.exceptions import UnexpectedResponse
from app.core.config import settings
import logging

logger = logging.getLogger(__name__)


def get_qdrant_client() -> QdrantClient:
    """Get Qdrant client instance."""
    return QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
    )


def init_qdrant_collection() -> bool:
    """
    Initialize Qdrant collection for Physical AI textbook embeddings.

    Creates a collection with:
    - Vector size: 1536 (OpenAI text-embedding-3-small)
    - Distance metric: Cosine similarity
    - Optimized for free-tier (1GB limit)

    Returns:
        bool: True if collection was created or already exists, False on error
    """
    try:
        client = get_qdrant_client()

        # Check if collection already exists
        collections = client.get_collections()
        collection_names = [col.name for col in collections.collections]

        if settings.QDRANT_COLLECTION_NAME in collection_names:
            logger.info(f"Collection '{settings.QDRANT_COLLECTION_NAME}' already exists")
            return True

        # Create collection with optimized settings for free tier
        client.create_collection(
            collection_name=settings.QDRANT_COLLECTION_NAME,
            vectors_config=VectorParams(
                size=1536,  # text-embedding-3-small dimension
                distance=Distance.COSINE,
            ),
            optimizers_config=OptimizersConfigDiff(
                indexing_threshold=10000,  # Start indexing after 10k points
            ),
        )

        logger.info(f"Created collection '{settings.QDRANT_COLLECTION_NAME}'")
        return True

    except UnexpectedResponse as e:
        logger.error(f"Qdrant error: {e}")
        return False
    except Exception as e:
        logger.error(f"Unexpected error creating Qdrant collection: {e}")
        return False


def get_collection_info() -> dict:
    """Get information about the Qdrant collection."""
    try:
        client = get_qdrant_client()
        collection_info = client.get_collection(settings.QDRANT_COLLECTION_NAME)

        return {
            "name": collection_info.name,
            "vectors_count": collection_info.vectors_count,
            "points_count": collection_info.points_count,
            "status": collection_info.status,
        }
    except Exception as e:
        logger.error(f"Error getting collection info: {e}")
        return {"error": str(e)}


if __name__ == "__main__":
    # Run as script to initialize collection
    logging.basicConfig(level=logging.INFO)
    success = init_qdrant_collection()

    if success:
        print("✅ Qdrant collection initialized successfully")
        info = get_collection_info()
        print(f"📊 Collection info: {info}")
    else:
        print("❌ Failed to initialize Qdrant collection")
        exit(1)
