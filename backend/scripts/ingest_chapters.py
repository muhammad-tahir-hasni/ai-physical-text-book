#!/usr/bin/env python3
"""
Document ingestion script for Physical AI Textbook.

Reads all chapter markdown files and ingests them into Qdrant for RAG.

Usage:
    python backend/scripts/ingest_chapters.py
"""

import asyncio
import os
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services.rag_service import RAGService


# Chapter metadata
CHAPTERS = [
    {
        "file": "frontend/docs/module-1/chapter-1-1.md",
        "document_id": "module-1-chapter-1-1",
        "title": "Module 1 Chapter 1.1: ROS 2 Architecture & Core Concepts",
        "metadata": {
            "module": "Module 1",
            "module_name": "ROS 2 Fundamentals",
            "chapter": "1.1",
            "topics": ["ROS 2", "DDS", "Nodes", "Topics", "Services", "Actions"]
        }
    },
    {
        "file": "frontend/docs/module-1/chapter-1-2.md",
        "document_id": "module-1-chapter-1-2",
        "title": "Module 1 Chapter 1.2: Building ROS 2 Applications",
        "metadata": {
            "module": "Module 1",
            "module_name": "ROS 2 Fundamentals",
            "chapter": "1.2",
            "topics": ["Package Structure", "Launch Files", "Parameters", "CMake"]
        }
    },
    {
        "file": "frontend/docs/module-2/chapter-2-1.md",
        "document_id": "module-2-chapter-2-1",
        "title": "Module 2 Chapter 2.1: Gazebo Simulation Fundamentals",
        "metadata": {
            "module": "Module 2",
            "module_name": "Simulation & URDF",
            "chapter": "2.1",
            "topics": ["Gazebo", "SDF", "Physics Engines", "Sensor Simulation"]
        }
    },
    {
        "file": "frontend/docs/module-2/chapter-2-2.md",
        "document_id": "module-2-chapter-2-2",
        "title": "Module 2 Chapter 2.2: Robot Description (URDF)",
        "metadata": {
            "module": "Module 2",
            "module_name": "Simulation & URDF",
            "chapter": "2.2",
            "topics": ["URDF", "Xacro", "Links", "Joints", "Inertia"]
        }
    },
    {
        "file": "frontend/docs/module-3/chapter-3-1.md",
        "document_id": "module-3-chapter-3-1",
        "title": "Module 3 Chapter 3.1: NVIDIA Isaac Sim Introduction",
        "metadata": {
            "module": "Module 3",
            "module_name": "NVIDIA Isaac Sim",
            "chapter": "3.1",
            "topics": ["Isaac Sim", "USD", "PhysX", "RTX", "Omniverse"]
        }
    },
    {
        "file": "frontend/docs/module-3/chapter-3-2.md",
        "document_id": "module-3-chapter-3-2",
        "title": "Module 3 Chapter 3.2: AI-Powered Perception",
        "metadata": {
            "module": "Module 3",
            "module_name": "NVIDIA Isaac Sim",
            "chapter": "3.2",
            "topics": ["Computer Vision", "Object Detection", "Semantic Segmentation", "SLAM"]
        }
    },
    {
        "file": "frontend/docs/module-4/chapter-4-1.md",
        "document_id": "module-4-chapter-4-1",
        "title": "Module 4 Chapter 4.1: Voice-to-Action Systems",
        "metadata": {
            "module": "Module 4",
            "module_name": "Vision-Language-Action",
            "chapter": "4.1",
            "topics": ["Speech Recognition", "NLU", "Voice Control", "Whisper"]
        }
    },
    {
        "file": "frontend/docs/module-4/chapter-4-2.md",
        "document_id": "module-4-chapter-4-2",
        "title": "Module 4 Chapter 4.2: LLM-Based Cognitive Planning",
        "metadata": {
            "module": "Module 4",
            "module_name": "Vision-Language-Action",
            "chapter": "4.2",
            "topics": ["LLM", "Task Planning", "GPT-4", "Chain-of-Thought", "VLA"]
        }
    }
]


async def ingest_all_chapters():
    """Ingest all textbook chapters into Qdrant."""
    # Get project root
    project_root = Path(__file__).parent.parent.parent

    # Initialize RAG service
    print("Initializing RAG service...")
    rag_service = RAGService()

    # Check collection stats before ingestion
    try:
        stats_before = rag_service.get_collection_stats()
        print(f"Collection stats before ingestion: {stats_before}")
    except Exception as e:
        print(f"Warning: Could not get collection stats: {e}")

    total_chunks = 0
    successful = 0
    failed = 0

    for chapter in CHAPTERS:
        try:
            # Read chapter file
            chapter_path = project_root / chapter["file"]

            if not chapter_path.exists():
                print(f"❌ File not found: {chapter_path}")
                failed += 1
                continue

            print(f"\n📄 Processing: {chapter['title']}")
            print(f"   File: {chapter['file']}")

            with open(chapter_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Ingest document
            num_chunks = await rag_service.ingest_document(
                document_id=chapter["document_id"],
                title=chapter["title"],
                markdown_content=content,
                metadata=chapter["metadata"]
            )

            total_chunks += num_chunks
            successful += 1
            print(f"   ✅ Ingested {num_chunks} chunks")

        except Exception as e:
            print(f"   ❌ Error: {str(e)}")
            failed += 1

    # Check collection stats after ingestion
    try:
        stats_after = rag_service.get_collection_stats()
        print(f"\n📊 Collection stats after ingestion:")
        print(f"   Total vectors: {stats_after['vectors_count']}")
        print(f"   Total points: {stats_after['points_count']}")
        print(f"   Status: {stats_after['status']}")
    except Exception as e:
        print(f"Warning: Could not get final collection stats: {e}")

    # Summary
    print(f"\n{'='*60}")
    print(f"📚 Ingestion Summary:")
    print(f"   Chapters processed: {successful + failed}")
    print(f"   Successful: {successful}")
    print(f"   Failed: {failed}")
    print(f"   Total chunks: {total_chunks}")
    print(f"{'='*60}")

    return successful, failed, total_chunks


async def test_search():
    """Test search functionality after ingestion."""
    print("\n🔍 Testing search functionality...")

    rag_service = RAGService()

    test_queries = [
        "What is ROS 2?",
        "How does Gazebo simulation work?",
        "Explain NVIDIA Isaac Sim",
        "What are Vision-Language-Action models?"
    ]

    for query in test_queries:
        print(f"\nQuery: {query}")
        try:
            results = await rag_service.search_similar(query, top_k=2)
            for i, result in enumerate(results, 1):
                print(f"  {i}. [{result['title']}] Score: {result['score']:.3f}")
                print(f"     {result['text'][:100]}...")
        except Exception as e:
            print(f"  ❌ Error: {str(e)}")


def main():
    """Main entry point."""
    print("="*60)
    print("Physical AI Textbook - Document Ingestion")
    print("="*60)

    # Check environment variables
    required_vars = ["OPENAI_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        print(f"\n❌ Missing required environment variables: {', '.join(missing_vars)}")
        print("Please set them in your .env file or environment.")
        sys.exit(1)

    # Run ingestion
    successful, failed, total_chunks = asyncio.run(ingest_all_chapters())

    # Run test search if ingestion was successful
    if successful > 0:
        asyncio.run(test_search())

    # Exit with appropriate code
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
