"""
Rate limiting middleware for API protection.

Implements token bucket algorithm for rate limiting based on IP address.
"""

import time
from collections import defaultdict
from typing import Dict, Tuple
from fastapi import Request, HTTPException, status
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response


class RateLimitMiddleware(BaseHTTPMiddleware):
    """
    Rate limiting middleware using token bucket algorithm.

    Limits:
    - Anonymous users (no auth): 5 requests/minute
    - Authenticated users: 10 requests/minute
    """

    def __init__(self, app, anonymous_limit: int = 5, authenticated_limit: int = 10):
        super().__init__(app)
        self.anonymous_limit = anonymous_limit
        self.authenticated_limit = authenticated_limit

        # Storage: {client_id: (tokens, last_update_time)}
        self.buckets: Dict[str, Tuple[float, float]] = defaultdict(
            lambda: (self.anonymous_limit, time.time())
        )

        # Window duration in seconds
        self.window = 60  # 1 minute

    async def dispatch(self, request: Request, call_next):
        """Process request with rate limiting."""
        # Skip rate limiting for health check and docs
        if request.url.path in ["/", "/health", "/api/v1/health", "/docs", "/redoc", "/openapi.json"]:
            return await call_next(request)

        # Get client identifier (IP address or user ID if authenticated)
        client_id = self._get_client_id(request)

        # Check rate limit
        allowed, remaining = self._check_rate_limit(client_id, request)

        if not allowed:
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail="Rate limit exceeded. Please try again later.",
                headers={
                    "X-RateLimit-Limit": str(self._get_limit(request)),
                    "X-RateLimit-Remaining": "0",
                    "X-RateLimit-Reset": str(int(time.time() + self.window)),
                    "Retry-After": str(self.window)
                }
            )

        # Process request
        response = await call_next(request)

        # Add rate limit headers to response
        response.headers["X-RateLimit-Limit"] = str(self._get_limit(request))
        response.headers["X-RateLimit-Remaining"] = str(int(remaining))
        response.headers["X-RateLimit-Reset"] = str(int(time.time() + self.window))

        return response

    def _get_client_id(self, request: Request) -> str:
        """
        Get unique client identifier.

        Args:
            request: FastAPI request

        Returns:
            Client ID (IP address or user ID)
        """
        # TODO: Check for user_id in request.state.user when auth is implemented
        # if hasattr(request.state, "user") and request.state.user:
        #     return f"user_{request.state.user.id}"

        # Use IP address for anonymous users
        forwarded = request.headers.get("X-Forwarded-For")
        if forwarded:
            return f"ip_{forwarded.split(',')[0].strip()}"

        client_host = request.client.host if request.client else "unknown"
        return f"ip_{client_host}"

    def _get_limit(self, request: Request) -> int:
        """
        Get rate limit for request.

        Args:
            request: FastAPI request

        Returns:
            Request limit per window
        """
        # TODO: Return authenticated_limit if user is authenticated
        # if hasattr(request.state, "user") and request.state.user:
        #     return self.authenticated_limit

        return self.anonymous_limit

    def _check_rate_limit(self, client_id: str, request: Request) -> Tuple[bool, float]:
        """
        Check if request is within rate limit using token bucket algorithm.

        Args:
            client_id: Client identifier
            request: FastAPI request

        Returns:
            Tuple of (allowed, remaining_tokens)
        """
        current_time = time.time()
        limit = self._get_limit(request)

        # Get or initialize bucket
        tokens, last_update = self.buckets[client_id]

        # Calculate time elapsed since last update
        time_elapsed = current_time - last_update

        # Refill tokens based on time elapsed
        # Tokens refill at rate of limit per window
        refill_rate = limit / self.window
        tokens = min(limit, tokens + time_elapsed * refill_rate)

        # Check if we have at least 1 token
        if tokens >= 1:
            # Consume 1 token
            tokens -= 1
            self.buckets[client_id] = (tokens, current_time)
            return True, tokens
        else:
            # Not enough tokens, rate limit exceeded
            self.buckets[client_id] = (tokens, current_time)
            return False, 0

    def cleanup_old_buckets(self, max_age: int = 3600):
        """
        Clean up old bucket entries to prevent memory leaks.

        Should be called periodically by a background task.

        Args:
            max_age: Maximum age in seconds before bucket is removed
        """
        current_time = time.time()
        expired_clients = [
            client_id
            for client_id, (_, last_update) in self.buckets.items()
            if current_time - last_update > max_age
        ]

        for client_id in expired_clients:
            del self.buckets[client_id]
