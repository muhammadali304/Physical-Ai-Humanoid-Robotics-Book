"""
Rate limiting middleware for the RAG Ingestion Pipeline API.
Implements rate limiting to prevent API abuse and ensure service stability.
"""

import time
import hashlib
from typing import Dict, Optional, Callable, Awaitable
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime, timedelta

from fastapi import Request, Response, HTTPException
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.types import ASGIApp

from src.utils.logging import get_logger


@dataclass
class RateLimit:
    """
    Rate limit configuration.
    """
    requests: int  # Number of requests allowed
    window: int    # Time window in seconds
    burst: int     # Burst limit (default to requests if not specified)


class RateLimitStore:
    """
    In-memory store for rate limit counters.
    In production, this would use Redis or another distributed store.
    """

    def __init__(self):
        self.counters: Dict[str, Dict[str, int]] = defaultdict(dict)  # {identifier: {timestamp: count}}
        self.logger = get_logger("rate_limit_store")

    def get_key_timestamp(self, timestamp: int, window: int) -> str:
        """
        Get the key timestamp for a given timestamp and window.

        Args:
            timestamp: Unix timestamp
            window: Time window in seconds

        Returns:
            Formatted timestamp key
        """
        return str((timestamp // window) * window)

    def increment(self, identifier: str, window: int) -> int:
        """
        Increment the request count for an identifier in the current time window.

        Args:
            identifier: Unique identifier for the client/route
            window: Time window in seconds

        Returns:
            Current count for the window
        """
        current_time = int(time.time())
        key_timestamp = self.get_key_timestamp(current_time, window)

        # Reset counters for old windows
        self._cleanup_old_windows(identifier, window)

        # Increment counter
        self.counters[identifier][key_timestamp] = self.counters[identifier].get(key_timestamp, 0) + 1

        return self.counters[identifier][key_timestamp]

    def get_count(self, identifier: str, window: int) -> int:
        """
        Get the current request count for an identifier in the current time window.

        Args:
            identifier: Unique identifier for the client/route
            window: Time window in seconds

        Returns:
            Current count for the window
        """
        current_time = int(time.time())
        key_timestamp = self.get_key_timestamp(current_time, window)

        return self.counters[identifier].get(key_timestamp, 0)

    def _cleanup_old_windows(self, identifier: str, window: int):
        """
        Clean up old time windows to prevent memory leaks.

        Args:
            identifier: Unique identifier for the client/route
            window: Time window in seconds
        """
        current_time = int(time.time())
        current_window = self.get_key_timestamp(current_time, window)
        cutoff_time = int(current_window) - (window * 2)  # Keep 2 windows worth of data

        # Remove old windows
        old_keys = [
            key for key in self.counters[identifier].keys()
            if int(key) < cutoff_time
        ]

        for key in old_keys:
            del self.counters[identifier][key]


class RateLimitMiddleware(BaseHTTPMiddleware):
    """
    Rate limiting middleware for FastAPI applications.
    Supports different rate limits for different routes and clients.
    """

    def __init__(
        self,
        app: ASGIApp,
        default_limit: RateLimit = None,
        route_limits: Dict[str, RateLimit] = None,
        client_limits: Dict[str, RateLimit] = None,
        enable_logging: bool = True
    ):
        super().__init__(app)
        self.store = RateLimitStore()
        self.default_limit = default_limit or RateLimit(requests=100, window=3600, burst=10)  # 100 requests per hour
        self.route_limits = route_limits or {}
        self.client_limits = client_limits or {}
        self.enable_logging = enable_logging
        self.logger = get_logger("rate_limit_middleware")

    def get_client_identifier(self, request: Request) -> str:
        """
        Get a unique identifier for the client making the request.
        Uses IP address, but can be extended to use API keys, etc.

        Args:
            request: The incoming request

        Returns:
            Unique client identifier
        """
        # Check for forwarded IP headers first (for apps behind proxies)
        forwarded_for = request.headers.get("x-forwarded-for")
        if forwarded_for:
            # Take the first IP from the list (client's original IP)
            client_ip = forwarded_for.split(",")[0].strip()
        elif request.client:
            client_ip = request.client.host
        else:
            client_ip = "unknown"

        # You could also use API key, user ID, etc. if available
        # For example: api_key = request.headers.get("x-api-key")

        return f"ip:{client_ip}"

    def get_route_identifier(self, request: Request) -> str:
        """
        Get an identifier for the route being accessed.

        Args:
            request: The incoming request

        Returns:
            Route identifier
        """
        return f"route:{request.method}:{request.url.path}"

    def get_rate_limit_for_request(self, request: Request) -> RateLimit:
        """
        Get the appropriate rate limit for the given request.

        Args:
            request: The incoming request

        Returns:
            Rate limit configuration
        """
        # Check for route-specific limits first
        route_identifier = self.get_route_identifier(request)
        if route_identifier in self.route_limits:
            return self.route_limits[route_identifier]

        # Check for client-specific limits
        client_identifier = self.get_client_identifier(request)
        if client_identifier in self.client_limits:
            return self.client_limits[client_identifier]

        # Use default limit
        return self.default_limit

    async def dispatch(self, request: Request, call_next: Callable[[Request], Awaitable[Response]]) -> Response:
        """
        Process the request and apply rate limiting.

        Args:
            request: The incoming request
            call_next: Function to call the next middleware/route handler

        Returns:
            Response from the next handler or rate limit response
        """
        client_identifier = self.get_client_identifier(request)
        route_identifier = self.get_route_identifier(request)
        rate_limit = self.get_rate_limit_for_request(request)

        # Increment the counter
        current_count = self.store.increment(client_identifier, rate_limit.window)

        # Check if rate limit is exceeded
        if current_count > rate_limit.requests:
            retry_after = rate_limit.window - (int(time.time()) % rate_limit.window)

            if self.enable_logging:
                self.logger.warning(
                    f"Rate limit exceeded for {client_identifier} on {route_identifier}",
                    client_id=client_identifier,
                    route=route_identifier,
                    limit=rate_limit.requests,
                    window=rate_limit.window,
                    current_count=current_count,
                    retry_after=retry_after
                )

            # Return rate limit exceeded response
            raise HTTPException(
                status_code=429,
                detail={
                    "error": "Rate limit exceeded",
                    "message": f"Too many requests. Please try again in {retry_after} seconds.",
                    "retry_after": retry_after,
                    "limit": rate_limit.requests,
                    "window": rate_limit.window
                }
            )

        # Add rate limit headers to the request for potential use by handlers
        request.state.rate_limit_remaining = rate_limit.requests - current_count
        request.state.rate_limit_limit = rate_limit.requests
        request.state.rate_limit_reset = int(time.time()) + (rate_limit.window - (int(time.time()) % rate_limit.window))

        # Process the request
        response = await call_next(request)

        # Add rate limit headers to the response
        response.headers["X-RateLimit-Limit"] = str(rate_limit.requests)
        response.headers["X-RateLimit-Remaining"] = str(rate_limit.requests - current_count)
        response.headers["X-RateLimit-Reset"] = str(request.state.rate_limit_reset)

        if current_count > rate_limit.requests:
            response.headers["X-RateLimit-OverLimit"] = "true"

        return response


class SlidingWindowRateLimitMiddleware(BaseHTTPMiddleware):
    """
    Advanced rate limiting middleware using sliding window algorithm.
    More accurate than fixed window approach.
    """

    def __init__(
        self,
        app: ASGIApp,
        default_limit: RateLimit = None,
        route_limits: Dict[str, RateLimit] = None,
        enable_logging: bool = True
    ):
        super().__init__(app)
        self.requests: Dict[str, list] = defaultdict(list)  # {identifier: [timestamps]}
        self.default_limit = default_limit or RateLimit(requests=100, window=3600, burst=10)
        self.route_limits = route_limits or {}
        self.enable_logging = enable_logging
        self.logger = get_logger("sliding_window_rate_limit")

    def get_client_identifier(self, request: Request) -> str:
        """
        Get a unique identifier for the client making the request.
        """
        forwarded_for = request.headers.get("x-forwarded-for")
        if forwarded_for:
            client_ip = forwarded_for.split(",")[0].strip()
        elif request.client:
            client_ip = request.client.host
        else:
            client_ip = "unknown"

        return f"ip:{client_ip}"

    def get_route_identifier(self, request: Request) -> str:
        """
        Get an identifier for the route being accessed.
        """
        return f"route:{request.method}:{request.url.path}"

    def is_allowed(self, identifier: str, limit: RateLimit) -> tuple[bool, int]:
        """
        Check if a request is allowed based on sliding window algorithm.

        Args:
            identifier: Unique identifier for the client
            limit: Rate limit configuration

        Returns:
            Tuple of (is_allowed, retry_after_seconds)
        """
        current_time = time.time()

        # Clean old requests outside the window
        self.requests[identifier] = [
            timestamp for timestamp in self.requests[identifier]
            if current_time - timestamp < limit.window
        ]

        # Check if we're under the limit
        if len(self.requests[identifier]) < limit.requests:
            # Add current request
            self.requests[identifier].append(current_time)
            return True, 0

        # Rate limit exceeded - calculate when the oldest request will expire
        oldest_request = min(self.requests[identifier])
        retry_after = int(oldest_request + limit.window - current_time)

        return False, max(0, retry_after)

    async def dispatch(self, request: Request, call_next: Callable[[Request], Awaitable[Response]]) -> Response:
        """
        Process the request and apply sliding window rate limiting.
        """
        client_identifier = self.get_client_identifier(request)
        route_identifier = self.get_route_identifier(request)

        # Get appropriate rate limit
        rate_limit = self.route_limits.get(route_identifier, self.default_limit)

        # Check if request is allowed
        is_allowed, retry_after = self.is_allowed(client_identifier, rate_limit)

        if not is_allowed:
            if self.enable_logging:
                self.logger.warning(
                    f"Sliding window rate limit exceeded for {client_identifier}",
                    client_id=client_identifier,
                    route=route_identifier,
                    limit=rate_limit.requests,
                    window=rate_limit.window,
                    retry_after=retry_after
                )

            raise HTTPException(
                status_code=429,
                detail={
                    "error": "Rate limit exceeded",
                    "message": f"Too many requests. Please try again in {retry_after} seconds.",
                    "retry_after": retry_after,
                    "limit": rate_limit.requests,
                    "window": rate_limit.window
                }
            )

        # Process the request
        response = await call_next(request)

        return response


# Common rate limit configurations
class RateLimitPresets:
    """
    Common rate limit configurations for different use cases.
    """

    # Conservative limits for free tier users
    FREE_TIER = RateLimit(requests=100, window=3600, burst=10)  # 100/hour

    # Moderate limits for standard users
    STANDARD = RateLimit(requests=1000, window=3600, burst=50)  # 1000/hour

    # Generous limits for premium users
    PREMIUM = RateLimit(requests=10000, window=3600, burst=100)  # 10000/hour

    # High limits for internal services
    INTERNAL = RateLimit(requests=10000, window=3600, burst=1000)  # 10000/hour

    # Low limits for expensive operations (like embedding generation)
    EMBEDDING_API = RateLimit(requests=100, window=3600, burst=5)  # 100/hour


def add_rate_limiting_middleware(
    app,
    default_limit: RateLimit = None,
    route_limits: Dict[str, RateLimit] = None,
    use_sliding_window: bool = False
):
    """
    Add rate limiting middleware to the FastAPI application.

    Args:
        app: FastAPI application instance
        default_limit: Default rate limit configuration
        route_limits: Route-specific rate limits
        use_sliding_window: Whether to use sliding window algorithm
    """
    default_limit = default_limit or RateLimitPresets.STANDARD
    route_limits = route_limits or {}

    if use_sliding_window:
        app.add_middleware(
            SlidingWindowRateLimitMiddleware,
            default_limit=default_limit,
            route_limits=route_limits
        )
    else:
        app.add_middleware(
            RateLimitMiddleware,
            default_limit=default_limit,
            route_limits=route_limits
        )