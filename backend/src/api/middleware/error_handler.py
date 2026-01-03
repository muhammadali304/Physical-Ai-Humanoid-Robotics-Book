"""
Error handling middleware for the RAG Ingestion Pipeline API.
Provides comprehensive error handling and consistent error responses.
"""

import traceback
from typing import Callable, Awaitable
from uuid import uuid4

from fastapi import Request, Response, HTTPException
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.middleware.errors import ServerErrorMiddleware
from pydantic import ValidationError

from src.utils.logging import get_logger


class ErrorDetails:
    """
    Container for error details.
    """
    def __init__(self, status_code: int, message: str, error_id: str = None, details: dict = None):
        self.status_code = status_code
        self.message = message
        self.error_id = error_id or str(uuid4())
        self.details = details or {}


class APIErrorHandlerMiddleware(BaseHTTPMiddleware):
    """
    Custom error handling middleware for the API.
    Provides consistent error responses and logging.
    """

    def __init__(self, app):
        super().__init__(app)
        self.logger = get_logger("api_error_handler")

    async def dispatch(self, request: Request, call_next: Callable[[Request], Awaitable[Response]]) -> Response:
        try:
            # Add request ID for tracing
            request_id = str(uuid4())
            request.state.request_id = request_id

            # Add to logger context
            response = await call_next(request)

            return response

        except HTTPException as e:
            error_details = ErrorDetails(
                status_code=e.status_code,
                message=e.detail,
                details={
                    "type": "HTTPException",
                    "headers": dict(e.headers) if e.headers else None
                }
            )
            return self._create_error_response(request, error_details)

        except ValidationError as e:
            error_details = ErrorDetails(
                status_code=422,
                message="Validation error in request body",
                details={
                    "type": "ValidationError",
                    "errors": e.errors()
                }
            )
            return self._create_error_response(request, error_details)

        except Exception as e:
            # Log the full exception with traceback
            self.logger.error(
                f"Unhandled exception in request {getattr(request.state, 'request_id', 'unknown')}: {str(e)}",
                request_id=getattr(request.state, 'request_id', 'unknown'),
                path=request.url.path,
                method=request.method,
                error=str(e),
                traceback=traceback.format_exc()
            )

            error_details = ErrorDetails(
                status_code=500,
                message="Internal server error",
                details={
                    "type": "InternalServerError",
                    "error_class": type(e).__name__,
                    "error_message": str(e)
                }
            )
            return self._create_error_response(request, error_details)

    def _create_error_response(self, request: Request, error_details: ErrorDetails) -> Response:
        """
        Create a standardized error response.

        Args:
            request: The incoming request
            error_details: Error details to include in response

        Returns:
            JSONResponse with standardized error format
        """
        # Log the error
        self.logger.error(
            f"API Error {error_details.error_id}: {error_details.message}",
            request_id=getattr(request.state, 'request_id', 'unknown'),
            path=request.url.path,
            method=request.method,
            status_code=error_details.status_code,
            error_id=error_details.error_id,
            error_message=error_details.message,
            error_details=error_details.details
        )

        error_response = {
            "error": {
                "id": error_details.error_id,
                "message": error_details.message,
                "status_code": error_details.status_code,
                "timestamp": __import__('datetime').datetime.utcnow().isoformat(),
                "path": request.url.path,
                "method": request.method
            }
        }

        # Add error details if present
        if error_details.details:
            error_response["error"]["details"] = error_details.details

        return JSONResponse(
            status_code=error_details.status_code,
            content=error_response
        )


class DetailedHTTPException(HTTPException):
    """
    Extended HTTPException with additional details for better error reporting.
    """

    def __init__(
        self,
        status_code: int,
        detail: str = None,
        headers: dict = None,
        error_code: str = None,
        additional_info: dict = None
    ):
        super().__init__(status_code=status_code, detail=detail, headers=headers)
        self.error_code = error_code
        self.additional_info = additional_info or {}


def add_error_handling_middleware(app):
    """
    Add error handling middleware to the FastAPI application.

    Args:
        app: FastAPI application instance
    """
    # Add our custom error handler middleware
    app.add_middleware(APIErrorHandlerMiddleware)

    # Also keep the default server error middleware but with custom handler
    app.add_middleware(
        ServerErrorMiddleware,
        handler=None  # Use our custom handler instead
    )


# Error handler functions for specific error types
async def handle_validation_error(request: Request, exc: ValidationError):
    """
    Handle validation errors specifically.

    Args:
        request: The incoming request
        exc: The validation exception

    Returns:
        JSONResponse with validation error details
    """
    logger = get_logger("validation_error_handler")

    error_id = str(uuid4())
    logger.error(
        f"Validation error {error_id}",
        request_id=getattr(request.state, 'request_id', 'unknown'),
        path=request.url.path,
        method=request.method,
        errors=exc.errors()
    )

    return JSONResponse(
        status_code=422,
        content={
            "error": {
                "id": error_id,
                "message": "Request validation failed",
                "status_code": 422,
                "timestamp": __import__('datetime').datetime.utcnow().isoformat(),
                "path": request.url.path,
                "method": request.method,
                "details": {
                    "errors": exc.errors(),
                    "field_count": len(exc.errors())
                }
            }
        }
    )


async def handle_http_error(request: Request, exc: HTTPException):
    """
    Handle HTTP exceptions specifically.

    Args:
        request: The incoming request
        exc: The HTTP exception

    Returns:
        JSONResponse with HTTP error details
    """
    logger = get_logger("http_error_handler")

    error_id = str(uuid4())
    logger.error(
        f"HTTP error {error_id}: {exc.status_code}",
        request_id=getattr(request.state, 'request_id', 'unknown'),
        path=request.url.path,
        method=request.method,
        status_code=exc.status_code,
        detail=exc.detail
    )

    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": {
                "id": error_id,
                "message": str(exc.detail),
                "status_code": exc.status_code,
                "timestamp": __import__('datetime').datetime.utcnow().isoformat(),
                "path": request.url.path,
                "method": request.method
            }
        }
    )


# Standard error response models
class ErrorResponse:
    """
    Standard error response model.
    """

    @staticmethod
    def create_error_response(
        message: str,
        status_code: int,
        error_code: str = None,
        details: dict = None,
        request_id: str = None
    ) -> dict:
        """
        Create a standardized error response dictionary.

        Args:
            message: Error message
            status_code: HTTP status code
            error_code: Application-specific error code
            details: Additional error details
            request_id: Request ID for tracing

        Returns:
            Dictionary with standardized error format
        """
        response = {
            "error": {
                "message": message,
                "status_code": status_code,
                "timestamp": __import__('datetime').datetime.utcnow().isoformat(),
                "details": details or {}
            }
        }

        if error_code:
            response["error"]["error_code"] = error_code

        if request_id:
            response["error"]["request_id"] = request_id

        return response


# Common error types and their handlers
class ErrorTypes:
    """
    Predefined error types for common API errors.
    """
    VALIDATION_ERROR = "VALIDATION_ERROR"
    AUTHENTICATION_ERROR = "AUTHENTICATION_ERROR"
    AUTHORIZATION_ERROR = "AUTHORIZATION_ERROR"
    NOT_FOUND_ERROR = "NOT_FOUND_ERROR"
    INTERNAL_ERROR = "INTERNAL_ERROR"
    RATE_LIMIT_ERROR = "RATE_LIMIT_ERROR"
    TIMEOUT_ERROR = "TIMEOUT_ERROR"
    CONFLICT_ERROR = "CONFLICT_ERROR"


def create_error_response(error_type: str, message: str, **kwargs) -> dict:
    """
    Create a standard error response for common error types.

    Args:
        error_type: Type of error from ErrorTypes enum
        message: Error message
        **kwargs: Additional fields to include in response

    Returns:
        Dictionary with error response
    """
    status_codes = {
        ErrorTypes.VALIDATION_ERROR: 422,
        ErrorTypes.AUTHENTICATION_ERROR: 401,
        ErrorTypes.AUTHORIZATION_ERROR: 403,
        ErrorTypes.NOT_FOUND_ERROR: 404,
        ErrorTypes.INTERNAL_ERROR: 500,
        ErrorTypes.RATE_LIMIT_ERROR: 429,
        ErrorTypes.TIMEOUT_ERROR: 408,
        ErrorTypes.CONFLICT_ERROR: 409
    }

    status_code = status_codes.get(error_type, 400)

    response = {
        "error": {
            "type": error_type,
            "message": message,
            "status_code": status_code,
            "timestamp": __import__('datetime').datetime.utcnow().isoformat()
        }
    }

    response["error"].update(kwargs)
    return response