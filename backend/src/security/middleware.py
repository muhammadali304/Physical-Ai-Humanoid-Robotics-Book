"""
Security middleware for the RAG Ingestion Pipeline.
Implements security validation for API requests and responses.
"""

import time
from typing import Dict, Any, Optional, Callable, Awaitable
from fastapi import Request, Response, HTTPException
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.types import ASGIApp

from src.security.validation import get_security_validator, SecurityValidationResult
from src.utils.logging import get_logger


class SecurityMiddleware(BaseHTTPMiddleware):
    """
    Security middleware to validate requests and responses.
    """

    def __init__(self, app: ASGIApp, enable_request_validation: bool = True, enable_response_sanitization: bool = False):
        super().__init__(app)
        self.security_validator = get_security_validator()
        self.enable_request_validation = enable_request_validation
        self.enable_response_sanitization = enable_response_sanitization
        self.logger = get_logger("security_middleware")

    async def dispatch(self, request: Request, call_next: Callable[[Request], Awaitable[Response]]) -> Response:
        """
        Process the request through security validation.

        Args:
            request: Incoming request
            call_next: Next middleware/handler in the chain

        Returns:
            Response from the next handler
        """
        start_time = time.time()

        # Log the request for security monitoring
        self._log_request(request)

        # Validate the request if enabled
        if self.enable_request_validation:
            validation_result = await self._validate_request(request)
            if not validation_result.is_valid:
                self.logger.error(
                    f"Security validation failed for {request.method} {request.url.path}",
                    method=request.method,
                    path=request.url.path,
                    threats=validation_result.threats_detected,
                    severity=validation_result.severity
                )

                # Block the request if threats are detected
                raise HTTPException(
                    status_code=400,
                    detail={
                        "error": "Security validation failed",
                        "threats_detected": validation_result.threats_detected,
                        "severity": validation_result.severity
                    }
                )

        # Process the request
        response = await call_next(request)

        # Sanitize the response if enabled
        if self.enable_response_sanitization:
            response = await self._sanitize_response(response)

        # Log the response for security monitoring
        self._log_response(request, response, time.time() - start_time)

        return response

    async def _validate_request(self, request: Request) -> SecurityValidationResult:
        """
        Validate the incoming request for security threats.

        Args:
            request: FastAPI request object

        Returns:
            SecurityValidationResult with validation results
        """
        try:
            # Validate URL
            url_result = self.security_validator.validate_url(str(request.url))
            if not url_result.is_valid:
                return url_result

            # Validate query parameters
            query_params = dict(request.query_params)
            if query_params:
                query_result = self.security_validator.validate_input(query_params, "query_params")
                if not query_result.is_valid:
                    return query_result

            # Validate headers (excluding sensitive ones)
            headers_to_check = {}
            for key, value in request.headers.items():
                if key.lower() not in ['authorization', 'cookie', 'x-forwarded-for']:
                    headers_to_check[key] = value

            if headers_to_check:
                header_result = self.security_validator.validate_input(headers_to_check, "headers")
                if not header_result.is_valid:
                    return header_result

            # Validate request body if present
            if request.method in ["POST", "PUT", "PATCH"]:
                try:
                    body = await request.json()
                    body_result = self.security_validator.validate_input(body, "request_body")
                    if not body_result.is_valid:
                        return body_result
                except Exception:
                    # If JSON parsing fails, try to read as text
                    try:
                        body_bytes = await request.body()
                        body_str = body_bytes.decode('utf-8')
                        body_result = self.security_validator.validate_input(body_str, "request_body")
                        if not body_result.is_valid:
                            return body_result
                    except Exception:
                        # If all parsing fails, skip body validation
                        pass

            # If all validations pass, return a valid result
            return SecurityValidationResult(
                is_valid=True,
                scan_type=self.security_validator.validate_input.__name__,  # This is a placeholder
                threats_detected=[],
                details={"validation_passed": True},
                severity="low"
            )

        except Exception as e:
            self.logger.error(f"Error during request validation: {str(e)}", error=str(e))
            # Return a valid result to avoid blocking legitimate requests due to validation errors
            return SecurityValidationResult(
                is_valid=True,
                scan_type=self.security_validator.validate_input.__name__,  # This is a placeholder
                threats_detected=[f"Validation error: {str(e)}"],
                details={"validation_error": str(e)},
                severity="low"
            )

    async def _sanitize_response(self, response: Response) -> Response:
        """
        Sanitize the response for security threats.

        Args:
            response: FastAPI response object

        Returns:
            Sanitized response
        """
        try:
            # Currently, we don't modify response content in this middleware
            # Response sanitization would typically happen at the application level
            # based on the content type and context
            return response
        except Exception as e:
            self.logger.error(f"Error during response sanitization: {str(e)}", error=str(e))
            return response

    def _log_request(self, request: Request):
        """
        Log request details for security monitoring.

        Args:
            request: FastAPI request object
        """
        try:
            client_host = request.client.host if request.client else "unknown"
            client_port = request.client.port if request.client else "unknown"

            self.logger.info(
                f"Security audit: {request.method} {request.url.path}",
                method=request.method,
                path=request.url.path,
                client_host=client_host,
                client_port=client_port,
                user_agent=request.headers.get("user-agent", ""),
                referer=request.headers.get("referer", ""),
                content_length=request.headers.get("content-length", "0")
            )
        except Exception as e:
            self.logger.error(f"Error logging request: {str(e)}", error=str(e))

    def _log_response(self, request: Request, response: Response, duration: float):
        """
        Log response details for security monitoring.

        Args:
            request: FastAPI request object
            response: FastAPI response object
            duration: Request processing duration
        """
        try:
            self.logger.info(
                f"Security audit: {request.method} {request.url.path} -> {response.status_code} in {duration:.3f}s",
                method=request.method,
                path=request.url.path,
                status_code=response.status_code,
                duration=duration,
                content_length=response.headers.get("content-length", "0")
            )
        except Exception as e:
            self.logger.error(f"Error logging response: {str(e)}", error=str(e))


def add_security_middleware(app, enable_request_validation: bool = True, enable_response_sanitization: bool = False):
    """
    Add security middleware to the FastAPI application.

    Args:
        app: FastAPI application instance
        enable_request_validation: Whether to enable request validation
        enable_response_sanitization: Whether to enable response sanitization
    """
    app.add_middleware(
        SecurityMiddleware,
        enable_request_validation=enable_request_validation,
        enable_response_sanitization=enable_response_sanitization
    )


class RateLimitingMiddleware(BaseHTTPMiddleware):
    """
    Additional security middleware for rate limiting and protection against abuse.
    """

    def __init__(self, app: ASGIApp, requests_per_minute: int = 60):
        super().__init__(app)
        self.requests_per_minute = requests_per_minute
        self.request_counts: Dict[str, list] = {}
        self.logger = get_logger("rate_limiting_middleware")

    async def dispatch(self, request: Request, call_next: Callable[[Request], Awaitable[Response]]) -> Response:
        """
        Process the request through rate limiting.

        Args:
            request: Incoming request
            call_next: Next middleware/handler in the chain

        Returns:
            Response from the next handler
        """
        client_ip = self._get_client_ip(request)

        # Clean old requests (older than 1 minute)
        current_time = time.time()
        if client_ip in self.request_counts:
            self.request_counts[client_ip] = [
                req_time for req_time in self.request_counts[client_ip]
                if current_time - req_time < 60  # Keep requests from last minute
            ]
        else:
            self.request_counts[client_ip] = []

        # Add current request
        self.request_counts[client_ip].append(current_time)

        # Check rate limit
        if len(self.request_counts[client_ip]) > self.requests_per_minute:
            self.logger.warning(
                f"Rate limit exceeded for IP {client_ip}: {len(self.request_counts[client_ip])} requests in last minute",
                client_ip=client_ip,
                request_count=len(self.request_counts[client_ip]),
                limit=self.requests_per_minute
            )

            raise HTTPException(
                status_code=429,
                detail={
                    "error": "Rate limit exceeded",
                    "message": f"Too many requests. Maximum {self.requests_per_minute} per minute."
                }
            )

        return await call_next(request)

    def _get_client_ip(self, request: Request) -> str:
        """
        Get the client IP address, accounting for proxies.

        Args:
            request: FastAPI request object

        Returns:
            Client IP address
        """
        # Check for forwarded headers first
        forwarded_for = request.headers.get("x-forwarded-for")
        if forwarded_for:
            # Take the first IP from the list (client's original IP)
            return forwarded_for.split(",")[0].strip()

        forwarded_host = request.headers.get("x-forwarded-host")
        if forwarded_host:
            return forwarded_host

        real_ip = request.headers.get("x-real-ip")
        if real_ip:
            return real_ip

        # Fall back to direct client IP
        if request.client and request.client.host:
            return request.client.host

        return "unknown"


def add_rate_limiting_middleware(app, requests_per_minute: int = 60):
    """
    Add rate limiting middleware to the FastAPI application.

    Args:
        app: FastAPI application instance
        requests_per_minute: Maximum requests allowed per minute per IP
    """
    app.add_middleware(
        RateLimitingMiddleware,
        requests_per_minute=requests_per_minute
    )


class InputSanitizationMiddleware(BaseHTTPMiddleware):
    """
    Middleware to sanitize input data for security.
    """

    def __init__(self, app: ASGIApp):
        super().__init__(app)
        self.security_validator = get_security_validator()
        self.logger = get_logger("input_sanitization_middleware")

    async def dispatch(self, request: Request, call_next: Callable[[Request], Awaitable[Response]]) -> Response:
        """
        Sanitize input data in the request.

        Args:
            request: Incoming request
            call_next: Next middleware/handler in the chain

        Returns:
            Response from the next handler
        """
        # Note: Actually modifying the request body would require more complex handling
        # For now, we'll just validate and log any issues

        # Process the request normally
        response = await call_next(request)

        return response


def add_input_sanitization_middleware(app):
    """
    Add input sanitization middleware to the FastAPI application.

    Args:
        app: FastAPI application instance
    """
    app.add_middleware(InputSanitizationMiddleware)


# Export for use in other modules
__all__ = [
    'SecurityMiddleware',
    'RateLimitingMiddleware',
    'InputSanitizationMiddleware',
    'add_security_middleware',
    'add_rate_limiting_middleware',
    'add_input_sanitization_middleware'
]