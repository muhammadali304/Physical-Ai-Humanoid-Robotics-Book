"""
Error response models for the RAG Ingestion Pipeline API.
Defines standardized error response formats.
"""

from typing import Optional, Dict, Any
from datetime import datetime
from uuid import UUID, uuid4

from pydantic import BaseModel, Field


class APIError(BaseModel):
    """
    Standard API error model.
    """
    id: str = Field(default_factory=lambda: str(uuid4()), description="Unique error identifier")
    message: str = Field(..., description="Human-readable error message")
    status_code: int = Field(..., description="HTTP status code")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Error timestamp")
    path: Optional[str] = Field(None, description="Request path where error occurred")
    method: Optional[str] = Field(None, description="HTTP method that caused the error")
    details: Optional[Dict[str, Any]] = Field(None, description="Additional error details")
    error_code: Optional[str] = Field(None, description="Application-specific error code")
    request_id: Optional[str] = Field(None, description="Request identifier for tracing")


class ErrorResponse(BaseModel):
    """
    Standard error response wrapper.
    """
    error: APIError = Field(..., description="Error details")


class ValidationErrorItem(BaseModel):
    """
    Individual validation error item.
    """
    loc: list = Field(..., description="Location of the validation error")
    msg: str = Field(..., description="Error message")
    type: str = Field(..., description="Error type")


class ValidationErrorResponse(BaseModel):
    """
    Validation error response model.
    """
    error: APIError = Field(..., description="Error details")
    validation_errors: Optional[list] = Field(None, description="List of validation errors")


class RateLimitError(BaseModel):
    """
    Rate limit error model.
    """
    id: str = Field(default_factory=lambda: str(uuid4()), description="Unique error identifier")
    message: str = Field(default="Rate limit exceeded", description="Error message")
    status_code: int = Field(429, description="HTTP status code")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Error timestamp")
    retry_after: Optional[int] = Field(None, description="Seconds to wait before retrying")
    limit: Optional[int] = Field(None, description="Rate limit value")
    window: Optional[int] = Field(None, description="Rate limit window in seconds")


class RateLimitErrorResponse(BaseModel):
    """
    Rate limit error response wrapper.
    """
    error: RateLimitError = Field(..., description="Rate limit error details")


class NotFoundError(BaseModel):
    """
    Not found error model.
    """
    id: str = Field(default_factory=lambda: str(uuid4()), description="Unique error identifier")
    message: str = Field(default="Resource not found", description="Error message")
    status_code: int = Field(404, description="HTTP status code")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Error timestamp")
    resource_type: Optional[str] = Field(None, description="Type of resource that was not found")
    resource_id: Optional[str] = Field(None, description="ID of the resource that was not found")


class NotFoundErrorResponse(BaseModel):
    """
    Not found error response wrapper.
    """
    error: NotFoundError = Field(..., description="Not found error details")


class ConflictError(BaseModel):
    """
    Conflict error model.
    """
    id: str = Field(default_factory=lambda: str(uuid4()), description="Unique error identifier")
    message: str = Field(default="Resource conflict", description="Error message")
    status_code: int = Field(409, description="HTTP status code")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Error timestamp")
    conflict_type: Optional[str] = Field(None, description="Type of conflict")


class ConflictErrorResponse(BaseModel):
    """
    Conflict error response wrapper.
    """
    error: ConflictError = Field(..., description="Conflict error details")


class UnauthorizedError(BaseModel):
    """
    Unauthorized error model.
    """
    id: str = Field(default_factory=lambda: str(uuid4()), description="Unique error identifier")
    message: str = Field(default="Authentication required", description="Error message")
    status_code: int = Field(401, description="HTTP status code")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Error timestamp")
    auth_method: Optional[str] = Field(None, description="Authentication method required")


class UnauthorizedErrorResponse(BaseModel):
    """
    Unauthorized error response wrapper.
    """
    error: UnauthorizedError = Field(..., description="Unauthorized error details")


class ForbiddenError(BaseModel):
    """
    Forbidden error model.
    """
    id: str = Field(default_factory=lambda: str(uuid4()), description="Unique error identifier")
    message: str = Field(default="Access forbidden", description="Error message")
    status_code: int = Field(403, description="HTTP status code")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Error timestamp")
    required_permission: Optional[str] = Field(None, description="Permission required")


class ForbiddenErrorResponse(BaseModel):
    """
    Forbidden error response wrapper.
    """
    error: ForbiddenError = Field(..., description="Forbidden error details")


class ServerError(BaseModel):
    """
    Server error model.
    """
    id: str = Field(default_factory=lambda: str(uuid4()), description="Unique error identifier")
    message: str = Field(default="Internal server error", description="Error message")
    status_code: int = Field(500, description="HTTP status code")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Error timestamp")
    error_type: Optional[str] = Field(None, description="Type of server error")


class ServerErrorResponse(BaseModel):
    """
    Server error response wrapper.
    """
    error: ServerError = Field(..., description="Server error details")


# Common error response instances
class ErrorResponses:
    """
    Predefined common error responses for easy reuse.
    """

    @staticmethod
    def bad_request(message: str = "Bad request", details: Optional[Dict[str, Any]] = None) -> ErrorResponse:
        """Create a bad request error response."""
        return ErrorResponse(
            error=APIError(
                message=message,
                status_code=400,
                details=details
            )
        )

    @staticmethod
    def unauthorized(message: str = "Unauthorized", details: Optional[Dict[str, Any]] = None) -> UnauthorizedErrorResponse:
        """Create an unauthorized error response."""
        return UnauthorizedErrorResponse(
            error=UnauthorizedError(
                message=message,
                details=details
            )
        )

    @staticmethod
    def forbidden(message: str = "Forbidden", details: Optional[Dict[str, Any]] = None) -> ForbiddenErrorResponse:
        """Create a forbidden error response."""
        return ForbiddenErrorResponse(
            error=ForbiddenError(
                message=message,
                details=details
            )
        )

    @staticmethod
    def not_found(resource_type: str = None, resource_id: str = None, details: Optional[Dict[str, Any]] = None) -> NotFoundErrorResponse:
        """Create a not found error response."""
        return NotFoundErrorResponse(
            error=NotFoundError(
                message=f"{resource_type} with ID {resource_id} not found" if resource_type and resource_id else "Resource not found",
                resource_type=resource_type,
                resource_id=resource_id,
                details=details
            )
        )

    @staticmethod
    def conflict(message: str = "Conflict", details: Optional[Dict[str, Any]] = None) -> ConflictErrorResponse:
        """Create a conflict error response."""
        return ConflictErrorResponse(
            error=ConflictError(
                message=message,
                details=details
            )
        )

    @staticmethod
    def validation_error(errors: list, details: Optional[Dict[str, Any]] = None) -> ValidationErrorResponse:
        """Create a validation error response."""
        return ValidationErrorResponse(
            error=APIError(
                message="Validation failed",
                status_code=422,
                details=details
            ),
            validation_errors=errors
        )

    @staticmethod
    def rate_limit(retry_after: int = None, limit: int = None, details: Optional[Dict[str, Any]] = None) -> RateLimitErrorResponse:
        """Create a rate limit error response."""
        return RateLimitErrorResponse(
            error=RateLimitError(
                message="Rate limit exceeded",
                retry_after=retry_after,
                limit=limit,
                details=details
            )
        )

    @staticmethod
    def server_error(message: str = "Internal server error", details: Optional[Dict[str, Any]] = None) -> ServerErrorResponse:
        """Create a server error response."""
        return ServerErrorResponse(
            error=ServerError(
                message=message,
                details=details
            )
        )


# Exception to error response mapping
class ExceptionErrorMapper:
    """
    Maps exceptions to appropriate error responses.
    """

    @staticmethod
    def map_exception_to_response(exception: Exception, request_path: str = None, request_method: str = None) -> ErrorResponse:
        """
        Map an exception to an appropriate error response.

        Args:
            exception: The exception to map
            request_path: The request path where the exception occurred
            request_method: The request method that caused the exception

        Returns:
            Appropriate error response for the exception
        """
        from fastapi import HTTPException

        if isinstance(exception, HTTPException):
            return ErrorResponse(
                error=APIError(
                    message=str(exception.detail),
                    status_code=exception.status_code,
                    path=request_path,
                    method=request_method,
                    details=getattr(exception, 'headers', None)
                )
            )

        # For other exceptions, return a server error
        return ErrorResponses.server_error(
            message="Internal server error",
            details={
                "exception_type": type(exception).__name__,
                "exception_message": str(exception)
            }
        )