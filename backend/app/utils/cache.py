"""
Caching utility for RAG queries.

Implements LRU (Least Recently Used) cache for common queries to reduce
OpenAI API calls and improve response times.
"""

import time
import hashlib
from typing import Dict, Any, Optional, Tuple
from collections import OrderedDict


class LRUCache:
    """
    LRU (Least Recently Used) cache implementation.

    Stores query results with automatic eviction of least recently used items
    when capacity is exceeded.
    """

    def __init__(self, capacity: int = 100, ttl: int = 3600):
        """
        Initialize LRU cache.

        Args:
            capacity: Maximum number of items to store
            ttl: Time to live in seconds (default: 1 hour)
        """
        self.capacity = capacity
        self.ttl = ttl
        # OrderedDict maintains insertion order
        self.cache: OrderedDict[str, Tuple[Any, float]] = OrderedDict()
        self.hits = 0
        self.misses = 0

    def get(self, key: str) -> Optional[Any]:
        """
        Retrieve item from cache.

        Args:
            key: Cache key

        Returns:
            Cached value if found and not expired, None otherwise
        """
        if key not in self.cache:
            self.misses += 1
            return None

        cache_entry = self.cache[key]

        # Handle both old format (value, timestamp) and new format (value, timestamp, ttl)
        if len(cache_entry) == 2:
            value, timestamp = cache_entry
            item_ttl = self.ttl
        else:
            value, timestamp, item_ttl = cache_entry

        # Check if expired
        if time.time() - timestamp > item_ttl:
            del self.cache[key]
            self.misses += 1
            return None

        # Move to end (mark as recently used)
        self.cache.move_to_end(key)
        self.hits += 1
        return value

    def set(self, key: str, value: Any, ttl: Optional[int] = None) -> None:
        """
        Store item in cache.

        Args:
            key: Cache key
            value: Value to cache
            ttl: Optional TTL override for this item (seconds)
        """
        # Update existing key
        if key in self.cache:
            self.cache.move_to_end(key)

        # Add new key with timestamp and optional custom TTL
        item_ttl = ttl if ttl is not None else self.ttl
        self.cache[key] = (value, time.time(), item_ttl)

        # Evict oldest if over capacity
        if len(self.cache) > self.capacity:
            self.cache.popitem(last=False)  # Remove first (oldest) item

    def delete(self, key: str) -> bool:
        """
        Delete item from cache.

        Args:
            key: Cache key

        Returns:
            True if item was deleted, False if not found
        """
        if key in self.cache:
            del self.cache[key]
            return True
        return False

    def clear(self) -> None:
        """Clear all items from cache."""
        self.cache.clear()
        self.hits = 0
        self.misses = 0

    def size(self) -> int:
        """Get current cache size."""
        return len(self.cache)

    def stats(self) -> Dict[str, Any]:
        """
        Get cache statistics.

        Returns:
            Dict with hits, misses, size, hit_rate
        """
        total_requests = self.hits + self.misses
        hit_rate = self.hits / total_requests if total_requests > 0 else 0

        return {
            "hits": self.hits,
            "misses": self.misses,
            "size": len(self.cache),
            "capacity": self.capacity,
            "hit_rate": round(hit_rate, 3),
            "ttl": self.ttl
        }

    def cleanup_expired(self) -> int:
        """
        Remove all expired items from cache.

        Returns:
            Number of items removed
        """
        current_time = time.time()
        expired_keys = [
            key
            for key, (_, timestamp) in self.cache.items()
            if current_time - timestamp > self.ttl
        ]

        for key in expired_keys:
            del self.cache[key]

        return len(expired_keys)


class QueryCache:
    """
    Specialized cache for RAG query results.

    Handles query normalization and result caching.
    """

    def __init__(self, capacity: int = 100, ttl: int = 3600):
        """
        Initialize query cache.

        Args:
            capacity: Maximum number of queries to cache
            ttl: Time to live in seconds
        """
        self.cache = LRUCache(capacity=capacity, ttl=ttl)

    def _normalize_query(self, query: str) -> str:
        """
        Normalize query for consistent caching.

        Args:
            query: Raw query string

        Returns:
            Normalized query
        """
        # Lowercase and strip whitespace
        normalized = query.lower().strip()

        # Remove extra whitespace
        normalized = " ".join(normalized.split())

        return normalized

    def _generate_key(self, query: str, **kwargs) -> str:
        """
        Generate cache key from query and parameters.

        Args:
            query: Query string
            **kwargs: Additional parameters (top_k, filters, etc.)

        Returns:
            Cache key (hash)
        """
        normalized_query = self._normalize_query(query)

        # Include relevant parameters in key
        key_parts = [normalized_query]

        # Sort kwargs for consistent hashing
        for k, v in sorted(kwargs.items()):
            key_parts.append(f"{k}={v}")

        key_string = "|".join(key_parts)

        # Generate hash
        return hashlib.md5(key_string.encode()).hexdigest()

    def get_cached_result(self, query: str, **kwargs) -> Optional[Dict]:
        """
        Get cached result for query.

        Args:
            query: Query string
            **kwargs: Query parameters

        Returns:
            Cached result if found, None otherwise
        """
        key = self._generate_key(query, **kwargs)
        return self.cache.get(key)

    def cache_result(self, query: str, result: Dict, **kwargs) -> None:
        """
        Cache query result.

        Args:
            query: Query string
            result: Result to cache
            **kwargs: Query parameters
        """
        key = self._generate_key(query, **kwargs)
        self.cache.set(key, result)

    def invalidate_query(self, query: str, **kwargs) -> bool:
        """
        Invalidate cached result for query.

        Args:
            query: Query string
            **kwargs: Query parameters

        Returns:
            True if cache entry was deleted
        """
        key = self._generate_key(query, **kwargs)
        return self.cache.delete(key)

    def clear(self) -> None:
        """Clear all cached results."""
        self.cache.clear()

    def stats(self) -> Dict[str, Any]:
        """Get cache statistics."""
        return self.cache.stats()


# Global cache instances
_query_cache: Optional[QueryCache] = None
_personalization_cache: Optional[LRUCache] = None


def get_query_cache() -> QueryCache:
    """
    Get global query cache instance (singleton).

    Returns:
        QueryCache instance
    """
    global _query_cache
    if _query_cache is None:
        _query_cache = QueryCache(capacity=100, ttl=3600)
    return _query_cache


def get_personalization_cache() -> LRUCache:
    """
    Get global personalization cache instance (singleton).

    Returns:
        LRUCache instance with 24-hour TTL and larger capacity
    """
    global _personalization_cache
    if _personalization_cache is None:
        # Larger capacity for personalized chapters (3 levels × 8 chapters = 24 items)
        _personalization_cache = LRUCache(capacity=50, ttl=86400)  # 24 hours
    return _personalization_cache


# Simple global cache reference for convenience
cache = get_personalization_cache()
