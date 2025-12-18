"""Test Qdrant connection."""
import os
from pathlib import Path
from dotenv import load_dotenv

# Load .env from backend directory
backend_dir = Path(__file__).parent
env_path = backend_dir / ".env"
print(f"Loading .env from: {env_path}")
print(f".env exists: {env_path.exists()}")
load_dotenv(env_path, override=True)

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

print(f"\nTesting connection to Qdrant...")
print(f"URL: {QDRANT_URL}")
print(f"API Key: {QDRANT_API_KEY[:20]}..." if QDRANT_API_KEY else "API Key: None")

try:
    from qdrant_client import QdrantClient

    client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
    )

    # Test connection
    collections = client.get_collections()
    print(f"\n✅ Successfully connected to Qdrant!")
    print(f"📊 Found {len(collections.collections)} collections:")
    for col in collections.collections:
        print(f"  - {col.name}")

except Exception as e:
    print(f"\n❌ Connection failed: {e}")
    print("\nPossible issues:")
    print("1. Check if QDRANT_URL is correct in .env")
    print("2. Verify QDRANT_API_KEY is valid")
    print("3. Check network/firewall settings")
    print("4. Verify the Qdrant cluster still exists")
