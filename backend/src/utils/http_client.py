"""
HTTP client utility with rate limiting for the RAG Ingestion Pipeline.
Provides a consistent interface for making HTTP requests with built-in rate limiting.
"""

import asyncio
import time
from typing import Optional, Dict, Any
from urllib.parse import urljoin
import httpx
from src.config.settings import settings
from src.config.constants import (
    DEFAULT_TIMEOUT,
    DEFAULT_MAX_RETRIES,
    DEFAULT_RETRY_DELAY,
    DEFAULT_BACKOFF_FACTOR
)
from src.utils.logging import get_logger


class RateLimiter:
    """Simple rate limiter to control request frequency."""

    def __init__(self, requests_per_second: float = 1.0):
        self.requests_per_second = requests_per_second
        self.min_interval = 1.0 / requests_per_second if requests_per_second > 0 else 0
        self.last_request_time = 0.0

    async def acquire(self):
        """Wait until it's safe to make another request."""
        current_time = time.time()
        elapsed = current_time - self.last_request_time

        if elapsed < self.min_interval:
            sleep_time = self.min_interval - elapsed
            await asyncio.sleep(sleep_time)

        self.last_request_time = time.time()


class HttpClient:
    """HTTP client with rate limiting and retry logic."""

    def __init__(
        self,
        base_url: Optional[str] = None,
        rate_limit: float = settings.rate_limit_delay,  # requests per second
        timeout: int = DEFAULT_TIMEOUT,
        max_retries: int = DEFAULT_MAX_RETRIES
    ):
        self.base_url = base_url
        self.rate_limiter = RateLimiter(1.0 / rate_limit if rate_limit > 0 else float('inf'))
        self.timeout = timeout
        self.max_retries = max_retries
        self.logger = get_logger("http_client")
        self._client: Optional[httpx.AsyncClient] = None

    async def __aenter__(self):
        """Async context manager entry."""
        self._client = httpx.AsyncClient(
            timeout=httpx.Timeout(self.timeout),
            follow_redirects=True
        )
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        if self._client:
            await self._client.aclose()
            self._client = None

    async def _make_request(
        self,
        method: str,
        url: str,
        **kwargs
    ) -> httpx.Response:
        """Make an HTTP request with rate limiting and retries."""
        if not self._client:
            raise RuntimeError("HttpClient not initialized. Use as async context manager.")

        # Join with base URL if provided
        if self.base_url:
            url = urljoin(self.base_url, url)

        # Apply rate limiting
        await self.rate_limiter.acquire()

        last_exception = None

        for attempt in range(self.max_retries + 1):
            try:
                response = await self._client.request(method, url, **kwargs)

                # Log the request
                self.logger.info(
                    f"HTTP {method} {url} - Status: {response.status_code}",
                    method=method,
                    url=url,
                    status_code=response.status_code,
                    attempt=attempt + 1
                )

                # Check for successful response
                response.raise_for_status()
                return response

            except httpx.HTTPStatusError as e:
                status_code = e.response.status_code
                self.logger.error(
                    f"HTTP {method} {url} failed with status {status_code}",
                    method=method,
                    url=url,
                    status_code=status_code,
                    attempt=attempt + 1
                )

                # Don't retry for client errors (4xx) except 429
                if 400 <= status_code < 500 and status_code != 429:
                    raise

                last_exception = e

                # If this was the last attempt, re-raise the exception
                if attempt == self.max_retries:
                    raise

            except httpx.RequestError as e:
                self.logger.error(
                    f"Request error for {method} {url}: {str(e)}",
                    method=method,
                    url=url,
                    error=str(e),
                    attempt=attempt + 1
                )
                last_exception = e

                # If this was the last attempt, re-raise the exception
                if attempt == self.max_retries:
                    raise

            # Wait before retrying (with exponential backoff)
            if attempt < self.max_retries:
                wait_time = DEFAULT_RETRY_DELAY * (DEFAULT_BACKOFF_FACTOR ** attempt)
                self.logger.info(
                    f"Retrying in {wait_time}s (attempt {attempt + 1}/{self.max_retries + 1})",
                    wait_time=wait_time,
                    attempt=attempt + 1
                )
                await asyncio.sleep(wait_time)

        # This should not be reached, but just in case
        if last_exception:
            raise last_exception

        raise RuntimeError("Request failed but no exception was recorded")

    async def get(self, url: str, **kwargs) -> httpx.Response:
        """Make a GET request."""
        return await self._make_request("GET", url, **kwargs)

    async def post(self, url: str, **kwargs) -> httpx.Response:
        """Make a POST request."""
        return await self._make_request("POST", url, **kwargs)

    async def put(self, url: str, **kwargs) -> httpx.Response:
        """Make a PUT request."""
        return await self._make_request("PUT", url, **kwargs)

    async def delete(self, url: str, **kwargs) -> httpx.Response:
        """Make a DELETE request."""
        return await self._make_request("DELETE", url, **kwargs)

    async def head(self, url: str, **kwargs) -> httpx.Response:
        """Make a HEAD request."""
        return await self._make_request("HEAD", url, **kwargs)

    async def patch(self, url: str, **kwargs) -> httpx.Response:
        """Make a PATCH request."""
        return await self._make_request("PATCH", url, **kwargs)


class CachedHttpClient(HttpClient):
    """HTTP client with caching capabilities."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._cache: Dict[str, tuple] = {}  # url -> (response, timestamp)
        self._cache_ttl = kwargs.get('cache_ttl', 300)  # 5 minutes default

    async def get(self, url: str, use_cache: bool = True, **kwargs) -> httpx.Response:
        """Make a GET request with optional caching."""
        if use_cache:
            cache_key = f"GET:{url}:{str(kwargs)}"
            if cache_key in self._cache:
                response, timestamp = self._cache[cache_key]
                if time.time() - timestamp < self._cache_ttl:
                    self.logger.info(f"Cache hit for {url}")
                    # Return a new response object with the same content
                    return httpx.Response(
                        status_code=response.status_code,
                        headers=response.headers,
                        content=response.content,
                        request=httpx.Request("GET", url)
                    )

        # Make the actual request
        response = await super().get(url, **kwargs)

        # Cache the response
        if use_cache:
            cache_key = f"GET:{url}:{str(kwargs)}"
            self._cache[cache_key] = (response, time.time())

        return response


# Convenience function to create a default HTTP client
def create_default_http_client() -> HttpClient:
    """Create a default HTTP client with standard settings."""
    return HttpClient(
        rate_limit=settings.rate_limit_delay,
        timeout=DEFAULT_TIMEOUT,
        max_retries=DEFAULT_MAX_RETRIES
    )


# Convenience function to create a cached HTTP client
def create_cached_http_client(cache_ttl: int = 300) -> CachedHttpClient:
    """Create a cached HTTP client with standard settings."""
    return CachedHttpClient(
        rate_limit=settings.rate_limit_delay,
        timeout=DEFAULT_TIMEOUT,
        max_retries=DEFAULT_MAX_RETRIES,
        cache_ttl=cache_ttl
    )