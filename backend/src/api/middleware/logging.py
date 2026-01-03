"""
Request/response logging middleware for the RAG Ingestion Pipeline API.
Provides structured logging for all API requests and responses.
"""

import time
import json
from typing import Callable, Awaitable
from uuid import uuid4

from fastapi import Request, Response
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.types import ASGIApp

from src.utils.logging import get_logger


class RequestResponseLoggingMiddleware(BaseHTTPMiddleware):
    """
    Middleware to log all incoming requests and outgoing responses.
    Provides structured logging with performance metrics.
    """

    def __init__(self, app: ASGIApp):
        super().__init__(app)
        self.logger = get_logger("api_request_response")

    async def dispatch(self, request: Request, call_next: Callable[[Request], Awaitable[Response]]) -> Response:
        # Generate request ID for tracking
        request_id = str(uuid4())
        request.state.request_id = request_id

        # Log request details
        start_time = time.time()

        # Get client IP
        client_host = request.client.host if request.client else "unknown"
        client_port = request.client.port if request.client else "unknown"

        # Get request headers (excluding sensitive ones)
        request_headers = dict(request.headers)
        sensitive_headers = ["authorization", "x-api-key", "cookie", "x-forwarded-for"]
        for header in sensitive_headers:
            if header in request_headers:
                request_headers[header] = "[REDACTED]"

        # Log the incoming request
        self.logger.info(
            f"API Request: {request.method} {request.url.path}",
            request_id=request_id,
            method=request.method,
            path=request.url.path,
            query_params=dict(request.query_params),
            client_host=client_host,
            client_port=client_port,
            headers=request_headers,
            content_length=request.headers.get("content-length", "0"),
            user_agent=request.headers.get("user-agent", ""),
            referer=request.headers.get("referer", "")
        )

        try:
            # Process the request
            response = await call_next(request)
        except Exception as e:
            # Calculate duration
            duration = time.time() - start_time

            # Log the error
            self.logger.error(
                f"API Request Error: {request.method} {request.url.path}",
                request_id=request_id,
                method=request.method,
                path=request.url.path,
                client_host=client_host,
                client_port=client_port,
                duration_ms=round(duration * 1000, 2),
                error=str(e),
                error_type=type(e).__name__
            )
            raise

        # Calculate duration
        duration = time.time() - start_time

        # Log the response
        self.logger.info(
            f"API Response: {request.method} {request.url.path} {response.status_code}",
            request_id=request_id,
            method=request.method,
            path=request.url.path,
            status_code=response.status_code,
            duration_ms=round(duration * 1000, 2),
            content_length=response.headers.get("content-length", "0"),
            content_type=response.headers.get("content-type", "")
        )

        # Add request ID to response headers for tracing
        response.headers["X-Request-ID"] = request_id
        response.headers["X-Response-Time"] = f"{round(duration * 1000, 2)}ms"

        return response


class DetailedLoggingMiddleware(BaseHTTPMiddleware):
    """
    Enhanced logging middleware with more detailed information.
    """

    def __init__(self, app: ASGIApp, log_request_body: bool = False, log_response_body: bool = False):
        super().__init__(app)
        self.logger = get_logger("detailed_api_logger")
        self.log_request_body = log_request_body
        self.log_response_body = log_response_body

    async def dispatch(self, request: Request, call_next: Callable[[Request], Awaitable[Response]]) -> Response:
        request_id = str(uuid4())
        request.state.request_id = request_id

        start_time = time.time()

        # Get client info
        client_info = {
            "host": request.client.host if request.client else "unknown",
            "port": request.client.port if request.client else "unknown"
        }

        # Prepare request data for logging
        request_data = {
            "request_id": request_id,
            "method": request.method,
            "path": request.url.path,
            "full_url": str(request.url),
            "query_params": dict(request.query_params),
            "client": client_info,
            "headers": self._sanitize_headers(dict(request.headers)),
            "content_length": request.headers.get("content-length", "0"),
            "user_agent": request.headers.get("user-agent", ""),
            "referer": request.headers.get("referer", ""),
            "timestamp": time.time()
        }

        # Log request body if enabled
        if self.log_request_body:
            try:
                body = await self._get_request_body(request)
                request_data["body"] = body
            except Exception:
                request_data["body"] = "[BODY_READ_ERROR]"

        # Log the request
        self.logger.info("API Request Started", **request_data)

        try:
            response = await call_next(request)
        except Exception as e:
            duration = time.time() - start_time
            error_data = {
                "request_id": request_id,
                "method": request.method,
                "path": request.url.path,
                "client": client_info,
                "duration_ms": round(duration * 1000, 2),
                "error": str(e),
                "error_type": type(e).__name__,
                "timestamp": time.time()
            }
            self.logger.error("API Request Failed", **error_data)
            raise

        duration = time.time() - start_time

        # Prepare response data for logging
        response_data = {
            "request_id": request_id,
            "method": request.method,
            "path": request.url.path,
            "status_code": response.status_code,
            "duration_ms": round(duration * 1000, 2),
            "content_length": response.headers.get("content-length", "0"),
            "content_type": response.headers.get("content-type", ""),
            "headers": dict(response.headers),
            "timestamp": time.time()
        }

        # Log response body if enabled
        if self.log_response_body:
            try:
                # Note: We can't easily read the response body without more complex handling
                # For now, we'll just log that it was sent
                response_data["body_logged"] = True
            except Exception:
                response_data["body_logged"] = False

        # Log the response
        self.logger.info("API Request Completed", **response_data)

        # Add tracing headers
        response.headers["X-Request-ID"] = request_id
        response.headers["X-Response-Time"] = f"{round(duration * 1000, 2)}ms"
        response.headers["X-Server"] = "RAG-Ingestion-Pipeline-API"

        return response

    def _sanitize_headers(self, headers: dict) -> dict:
        """
        Remove sensitive information from headers.

        Args:
            headers: Dictionary of headers

        Returns:
            Sanitized headers dictionary
        """
        sensitive_headers = ["authorization", "x-api-key", "cookie", "x-forwarded-for", "x-real-ip"]
        sanitized = headers.copy()

        for header in sensitive_headers:
            if header in sanitized:
                sanitized[header] = "[REDACTED]"

        return sanitized

    async def _get_request_body(self, request: Request):
        """
        Get the request body for logging purposes.

        Args:
            request: The incoming request

        Returns:
            Request body content
        """
        try:
            # Create a copy of the request stream
            body = await request.body()
            if body:
                # Try to decode as JSON
                try:
                    return json.loads(body.decode("utf-8"))
                except json.JSONDecodeError:
                    # If not JSON, return as string
                    return body.decode("utf-8", errors="replace")
            return None
        except Exception:
            return "[BODY_READ_ERROR]"


class PerformanceLoggingMiddleware(BaseHTTPMiddleware):
    """
    Middleware to specifically log performance metrics for API requests.
    """

    def __init__(self, app: ASGIApp):
        super().__init__(app)
        self.logger = get_logger("api_performance")

    async def dispatch(self, request: Request, call_next: Callable[[Request], Awaitable[Response]]) -> Response:
        request_id = str(uuid4())
        request.state.request_id = request_id

        start_time = time.time()
        start_perf_counter = time.perf_counter()

        response = None
        try:
            response = await call_next(request)
            response_status_code = response.status_code
        except Exception as e:
            # If there was an exception, we don't have a response, so use 500
            response_status_code = 500
            raise
        finally:
            # Calculate metrics
            end_time = time.time()
            end_perf_counter = time.perf_counter()

            duration_wall = end_time - start_time
            duration_cpu = end_perf_counter - start_perf_counter

            # Log performance metrics
            self.logger.info(
                f"Performance: {request.method} {request.url.path} -> {response_status_code}",
                request_id=request_id,
                method=request.method,
                path=request.url.path,
                status_code=response_status_code,
                duration_wall_ms=round(duration_wall * 1000, 2),
                duration_cpu_ms=round(duration_cpu * 1000, 2),
                duration_diff_ms=round((duration_wall - duration_cpu) * 1000, 2),
                client_host=request.client.host if request.client else "unknown",
                timestamp=end_time
            )

            # Add performance headers if we have a response
            if response is not None:
                response.headers["X-Response-Time"] = f"{round(duration_wall * 1000, 2)}ms"
                response.headers["X-Processing-Time"] = f"{round(duration_cpu * 1000, 2)}ms"

        return response


def add_request_response_logging(app, detailed: bool = False, log_request_body: bool = False, log_response_body: bool = False):
    """
    Add request/response logging middleware to the FastAPI application.

    Args:
        app: FastAPI application instance
        detailed: Whether to use detailed logging
        log_request_body: Whether to log request body content
        log_response_body: Whether to log response body content
    """
    if detailed:
        app.add_middleware(
            DetailedLoggingMiddleware,
            log_request_body=log_request_body,
            log_response_body=log_response_body
        )
    else:
        app.add_middleware(RequestResponseLoggingMiddleware)


def add_performance_logging(app):
    """
    Add performance logging middleware to the FastAPI application.

    Args:
        app: FastAPI application instance
    """
    app.add_middleware(PerformanceLoggingMiddleware)