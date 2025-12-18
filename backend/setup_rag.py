"""
Setup script for RAG system.
This script:
1. Initializes the Qdrant collection
2. Generates embeddings for all course content
3. Uploads embeddings to Qdrant
"""

import asyncio
import sys
from pathlib import Path
from dotenv import load_dotenv

# Load backend .env
backend_dir = Path(__file__).parent
load_dotenv(backend_dir / ".env", override=True)

from app.utils.qdrant_init import init_qdrant_collection, get_collection_info
from app.services.rag_service import RAGService


async def ingest_all_docs():
    """Ingest all markdown files from the docs directory."""
    docs_dir = backend_dir.parent / "frontend" / "docs"

    if not docs_dir.exists():
        print(f"❌ Docs directory not found: {docs_dir}")
        return 0

    print(f"📁 Scanning docs directory: {docs_dir}")

    # Find all markdown files
    md_files = list(docs_dir.rglob("*.md"))
    print(f"📄 Found {len(md_files)} markdown files")

    if not md_files:
        print("❌ No markdown files found!")
        return 0

    # Initialize RAG service
    rag_service = RAGService()

    total_chunks = 0
    for md_file in md_files:
        try:
            # Read markdown content
            content = md_file.read_text(encoding='utf-8')

            # Extract title from path or content
            title = md_file.stem.replace("-", " ").title()

            # Extract module/chapter info from path
            parts = md_file.parts
            metadata = {}
            if "module-" in str(md_file):
                for part in parts:
                    if part.startswith("module-"):
                        metadata["module"] = part
                        break

            # Ingest document
            print(f"  Processing: {md_file.relative_to(docs_dir)}")
            num_chunks = await rag_service.ingest_document(
                document_id=str(md_file.relative_to(docs_dir)),
                title=title,
                markdown_content=content,
                metadata=metadata
            )

            total_chunks += num_chunks
            print(f"    ✅ Added {num_chunks} chunks")

        except Exception as e:
            print(f"    ❌ Error processing {md_file.name}: {e}")
            continue

    return total_chunks


async def main():
    """Main setup function."""
    print("=" * 60)
    print("RAG System Setup")
    print("=" * 60)

    # Step 1: Initialize Qdrant collection
    print("\n1️⃣  Initializing Qdrant collection...")
    success = init_qdrant_collection()

    if not success:
        print("❌ Failed to initialize Qdrant collection!")
        print("Please check:")
        print("  - QDRANT_URL is correct in backend/.env")
        print("  - QDRANT_API_KEY is valid")
        print("  - Network connection to Qdrant")
        sys.exit(1)

    print("✅ Qdrant collection ready")

    # Step 2: Check collection info
    print("\n2️⃣  Collection information:")
    try:
        info = get_collection_info()
        for key, value in info.items():
            print(f"  {key}: {value}")
    except Exception as e:
        print(f"  ⚠️  Could not get collection info (non-critical): {type(e).__name__}")
        print(f"  Collection exists and is ready to use")

    # Step 3: Ingest documents
    print("\n3️⃣  Ingesting course documents...")
    total_chunks = await ingest_all_docs()

    if total_chunks == 0:
        print("❌ No content was ingested!")
        sys.exit(1)

    print(f"\n✅ Successfully ingested {total_chunks} chunks")

    # Step 4: Final collection info
    print("\n4️⃣  Final collection status:")
    try:
        info = get_collection_info()
        for key, value in info.items():
            print(f"  {key}: {value}")
    except Exception as e:
        print(f"  ⚠️  Could not get detailed info, but {total_chunks} chunks were uploaded successfully")

    print("\n" + "=" * 60)
    print("✅ RAG System Setup Complete!")
    print("=" * 60)
    print("\nYou can now:")
    print("  1. Start the backend: uvicorn app.main:app --reload")
    print("  2. Use the chat API at: POST /api/v1/chat")
    print("  3. Test in the frontend chat widget")


if __name__ == "__main__":
    asyncio.run(main())
