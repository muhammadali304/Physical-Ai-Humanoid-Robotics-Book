"""
Rate limiting middleware for the RAG Agent Backend.

This module provides rate limiting functionality for API endpoints
following the implementation plan requirements.
"""

import asyncio
import time
from typing import Optional, Callable, Any
from fastapi import Request, HTTPException, status
from collections import defaultdict, deque
from dataclasses import dataclass
from threading import Lock
import logging
from src.config.settings import settings


@dataclass
class RateLimitInfo:
    """Information about rate limiting for a client"""
    requests: deque
    count: int
    reset_time: float


class RateLimitMiddleware:
    """Middleware class for handling rate limiting"""

    def __init__(self):
        """Initialize the rate limiting middleware"""
        self.logger = logging.getLogger(__name__)
        self.storage = defaultdict(lambda: RateLimitInfo(deque(), 0, 0))
        self.lock = Lock()  # Thread-safe access to rate limit storage

        # Get rate limiting settings from configuration
        self.requests_per_minute = getattr(settings, 'requests_per_minute', 60)  # Default: 60 requests per minute
        self.requests_per_hour = getattr(settings, 'requests_per_hour', 1000)   # Default: 1000 requests per hour
        self.default_limit = getattr(settings, 'default_rate_limit', 100)      # Default: 100 requests per window

        self.window_size = 60  # 60 seconds window

    def _get_client_identifier(self, request: Request) -> str:
        """Get a unique identifier for the client"""
        # Use IP address as the primary identifier
        client_ip = request.client.host if request.client else "unknown"

        # You could also include other identifiers like API key, user ID, etc.
        # For now, we'll just use the IP address
        return client_ip

    def _is_rate_limited(self, client_id: str) -> tuple[bool, Optional[int]]:
        """
        Check if a client is rate limited.

        Returns:
            tuple[bool, Optional[int]]: (is_limited, seconds_to_reset)
        """
        current_time = time.time()
        window_start = current_time - self.window_size

        with self.lock:
            client_info = self.storage[client_id]

            # Remove old requests outside the current window
            while client_info.requests and client_info.requests[0] < window_start:
                client_info.requests.popleft()
                client_info.count -= 1

            # Check if the client has exceeded the rate limit
            if client_info.count >= self.default_limit:
                # Calculate seconds until reset
                oldest_request = client_info.requests[0] if client_info.requests else current_time
                seconds_to_reset = int(oldest_request + self.window_size - current_time) + 1
                return True, max(1, seconds_to_reset)

            # Add current request
            client_info.requests.append(current_time)
            client_info.count += 1

            # Calculate time until this request expires from the window
            seconds_to_reset = int(client_info.requests[0] + self.window_size - current_time) + 1
            return False, max(1, seconds_to_reset)

    async def __call__(self, request: Request, call_next: Callable) -> Any:
        """Middleware call implementation"""
        try:
            # Skip rate limiting for health check and other public endpoints
            if request.url.path in ["/health", "/docs", "/redoc", "/openapi.json"]:
                response = await call_next(request)
                return response

            # Get client identifier
            client_id = self._get_client_identifier(request)

            # Check if client is rate limited
            is_limited, seconds_to_reset = self._is_rate_limited(client_id)

            if is_limited:
                self.logger.warning(f"Rate limit exceeded for client {client_id}")

                raise HTTPException(
                    status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                    detail={
                        "error": "Rate limit exceeded",
                        "message": f"Too many requests. Please try again in {seconds_to_reset} seconds.",
                        "retry_after": seconds_to_reset
                    }
                )

            # Add rate limit headers to response
            request.state.rate_limit_remaining = self.default_limit - self.storage[client_id].count
            request.state.rate_limit_reset = seconds_to_reset

            # Continue with the request
            response = await call_next(request)

            # Add rate limit headers to response if it's a Response object
            if hasattr(response, 'headers'):
                response.headers["X-RateLimit-Limit"] = str(self.default_limit)
                response.headers["X-RateLimit-Remaining"] = str(request.state.rate_limit_remaining)
                response.headers["X-RateLimit-Reset"] = str(request.state.rate_limit_reset)

            return response

        except HTTPException:
            # Re-raise HTTP exceptions
            raise
        except Exception as e:
            self.logger.error(f"Unexpected error in rate limit middleware: {str(e)}")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Internal server error"
            )


# Function to add rate limiting middleware to FastAPI app
def add_rate_limiting_middleware(app):
    """
    Add rate limiting middleware to the FastAPI application.

    Args:
        app: FastAPI application instance
    """
    from starlette.middleware.base import BaseHTTPMiddleware

    # Use our custom middleware
    rate_limit_middleware = RateLimitMiddleware()

    # Add the middleware to the app
    app.add_middleware(BaseHTTPMiddleware, dispatch=rate_limit_middleware.__call__)

    logging.getLogger(__name__).info("Rate limiting middleware added to application")


class UserRateLimiter:
    """Class for implementing per-user rate limiting based on the functional requirement"""

    def __init__(self):
        """Initialize user-specific rate limiter"""
        self.logger = logging.getLogger(__name__)
        self.user_limits = defaultdict(lambda: RateLimitInfo(deque(), 0, 0))
        self.lock = Lock()

        # Get configurable rate limits from settings
        self.user_requests_per_minute = getattr(settings, 'user_requests_per_minute', 30)
        self.user_requests_per_hour = getattr(settings, 'user_requests_per_hour', 500)
        self.user_default_limit = getattr(settings, 'user_rate_limit', 100)

    def check_user_limit(self, user_id: str) -> tuple[bool, Optional[int]]:
        """
        Check if a specific user is rate limited.

        Returns:
            tuple[bool, Optional[int]]: (is_limited, seconds_to_reset)
        """
        current_time = time.time()
        window_start = current_time - 3600  # 1-hour window

        with self.lock:
            user_info = self.user_limits[user_id]

            # Remove old requests outside the current window
            while user_info.requests and user_info.requests[0] < window_start:
                user_info.requests.popleft()
                user_info.count -= 1

            # Check if the user has exceeded the rate limit
            if user_info.count >= self.user_default_limit:
                # Calculate seconds until reset
                oldest_request = user_info.requests[0] if user_info.requests else current_time
                seconds_to_reset = int(oldest_request + 3600 - current_time) + 1  # 1 hour window
                return True, max(1, seconds_to_reset)

            # Add current request
            user_info.requests.append(current_time)
            user_info.count += 1

            return False, None


# Global rate limiter instance
rate_limiter = UserRateLimiter()