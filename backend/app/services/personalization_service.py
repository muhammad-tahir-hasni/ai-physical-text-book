"""
Personalization service for adjusting chapter content difficulty.

Uses OpenAI GPT-3.5-turbo-16k to regenerate chapters at different complexity levels.
Implements caching to avoid redundant LLM calls.
"""

import os
import time
from pathlib import Path
from typing import Optional, Tuple
from openai import AsyncOpenAI
from app.core.config import settings
from app.prompts.personalization import get_personalization_prompt
from app.utils.cache import cache

# Initialize OpenAI client
client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)

# Path to chapter markdown files (relative to backend directory)
CHAPTERS_BASE_PATH = Path(__file__).parent.parent.parent.parent / "frontend" / "docs"


class PersonalizationService:
    """Service for personalizing chapter content based on user skill level."""

    @staticmethod
    def _get_chapter_path(chapter_id: str) -> Optional[Path]:
        """
        Get the file path for a chapter by its ID.

        Args:
            chapter_id: Chapter identifier (e.g., "1-1", "2-2")

        Returns:
            Path to chapter markdown file, or None if not found

        Examples:
            "1-1" -> frontend/docs/module-1/chapter-1-1.md
            "3-2" -> frontend/docs/module-3/chapter-3-2.md
        """
        module_num, chapter_num = chapter_id.split("-")

        # Try different naming patterns
        possible_paths = [
            CHAPTERS_BASE_PATH / f"module-{module_num}" / f"chapter-{chapter_id}.md",
            CHAPTERS_BASE_PATH / f"module-{module_num}" / f"chapter-{module_num}-{chapter_num}.md",
            CHAPTERS_BASE_PATH / f"module{module_num}" / f"chapter-{chapter_id}.md",
        ]

        for path in possible_paths:
            if path.exists():
                return path

        return None

    @staticmethod
    async def load_chapter_content(chapter_id: str) -> str:
        """
        Load original chapter content from markdown file.

        Args:
            chapter_id: Chapter identifier (e.g., "1-1")

        Returns:
            Chapter markdown content

        Raises:
            FileNotFoundError: If chapter file not found
        """
        chapter_path = PersonalizationService._get_chapter_path(chapter_id)

        if not chapter_path:
            raise FileNotFoundError(
                f"Chapter {chapter_id} not found. "
                f"Searched in: {CHAPTERS_BASE_PATH}"
            )

        with open(chapter_path, "r", encoding="utf-8") as f:
            content = f.read()

        return content

    @staticmethod
    async def personalize_chapter(
        chapter_id: str,
        complexity_level: str
    ) -> Tuple[str, bool, int]:
        """
        Personalize chapter content to the specified complexity level.

        Uses caching to avoid regenerating the same content multiple times.
        Cache TTL: 24 hours.

        Args:
            chapter_id: Chapter identifier (e.g., "1-1")
            complexity_level: "beginner", "intermediate", or "advanced"

        Returns:
            Tuple of (personalized_content, was_cached, generation_time_ms)

        Raises:
            FileNotFoundError: If chapter not found
            ValueError: If complexity level invalid
            Exception: If OpenAI API call fails
        """
        # Check cache first
        cache_key = f"personalized:{chapter_id}:{complexity_level}"
        cached_content = cache.get(cache_key)

        if cached_content:
            return cached_content, True, 0

        # Load original chapter
        original_content = await PersonalizationService.load_chapter_content(chapter_id)

        # For intermediate, return original without LLM call
        if complexity_level == "intermediate":
            # Cache the original
            cache.set(cache_key, original_content, ttl=86400)  # 24 hours
            return original_content, False, 0

        # Generate personalized version
        start_time = time.time()

        try:
            prompt = get_personalization_prompt(complexity_level, original_content)

            response = await client.chat.completions.create(
                model="gpt-3.5-turbo-16k",  # Longer context for full chapters
                messages=[
                    {"role": "user", "content": prompt}
                ],
                max_tokens=2000,
                temperature=0.7,
                timeout=30.0  # 30 second timeout
            )

            personalized_content = response.choices[0].message.content

            generation_time_ms = int((time.time() - start_time) * 1000)

            # Cache for 24 hours
            cache.set(cache_key, personalized_content, ttl=86400)

            return personalized_content, False, generation_time_ms

        except Exception as e:
            raise Exception(f"Failed to personalize chapter: {str(e)}")

    @staticmethod
    async def clear_cache_for_chapter(chapter_id: str) -> int:
        """
        Clear all cached personalized versions for a chapter.

        Args:
            chapter_id: Chapter identifier

        Returns:
            Number of cache entries cleared
        """
        cleared_count = 0
        for level in ["beginner", "intermediate", "advanced"]:
            cache_key = f"personalized:{chapter_id}:{level}"
            if cache.delete(cache_key):
                cleared_count += 1

        return cleared_count
