"""
API error handling utilities for the RAG Agent Backend.

This module provides error handling and response formatting utilities
following the implementation plan requirements.
"""

from typing import Optional, Dict, Any
from fastapi import HTTPException, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel
import logging
from enum import Enum


class ErrorCode(str, Enum):
    """Enumeration of error codes for consistent error handling"""
    # General errors
    INTERNAL_ERROR = "INTERNAL_ERROR"
    VALIDATION_ERROR = "VALIDATION_ERROR"
    NOT_FOUND = "NOT_FOUND"
    UNAUTHORIZED = "UNAUTHORIZED"
    FORBIDDEN = "FORBIDDEN"
    TOO_MANY_REQUESTS = "TOO_MANY_REQUESTS"

    # RAG-specific errors
    QUERY_PROCESSING_ERROR = "QUERY_PROCESSING_ERROR"
    RETRIEVAL_ERROR = "RETRIEVAL_ERROR"
    GENERATION_ERROR = "GENERATION_ERROR"
    EMBEDDING_ERROR = "EMBEDDING_ERROR"
    DOCUMENT_PROCESSING_ERROR = "DOCUMENT_PROCESSING_ERROR"
    QDRANT_CONNECTION_ERROR = "QDRANT_CONNECTION_ERROR"
    GEMINI_API_ERROR = "GEMINI_API_ERROR"


class APIError(BaseModel):
    """API Error response model"""
    error: str
    error_code: str
    details: Optional[Dict[str, Any]] = None


class RAGError(Exception):
    """Base exception class for RAG-related errors"""

    def __init__(self, message: str, error_code: ErrorCode, details: Optional[Dict[str, Any]] = None):
        self.message = message
        self.error_code = error_code
        self.details = details or {}
        super().__init__(self.message)

    def to_api_error(self) -> APIError:
        """Convert to API error response"""
        return APIError(
            error=self.message,
            error_code=self.error_code,
            details=self.details
        )


class QueryProcessingError(RAGError):
    """Exception raised when query processing fails"""
    def __init__(self, message: str = "Error processing query", details: Optional[Dict[str, Any]] = None):
        super().__init__(message, ErrorCode.QUERY_PROCESSING_ERROR, details)


class RetrievalError(RAGError):
    """Exception raised when document retrieval fails"""
    def __init__(self, message: str = "Error retrieving documents", details: Optional[Dict[str, Any]] = None):
        super().__init__(message, ErrorCode.RETRIEVAL_ERROR, details)


class GenerationError(RAGError):
    """Exception raised when response generation fails"""
    def __init__(self, message: str = "Error generating response", details: Optional[Dict[str, Any]] = None):
        super().__init__(message, ErrorCode.GENERATION_ERROR, details)


class EmbeddingError(RAGError):
    """Exception raised when embedding generation fails"""
    def __init__(self, message: str = "Error generating embedding", details: Optional[Dict[str, Any]] = None):
        super().__init__(message, ErrorCode.EMBEDDING_ERROR, details)


class DocumentProcessingError(RAGError):
    """Exception raised when document processing fails"""
    def __init__(self, message: str = "Error processing document", details: Optional[Dict[str, Any]] = None):
        super().__init__(message, ErrorCode.DOCUMENT_PROCESSING_ERROR, details)


def handle_validation_error(exc: Exception) -> JSONResponse:
    """Handle validation errors"""
    logging.getLogger(__name__).error(f"Validation error: {str(exc)}")
    return JSONResponse(
        status_code=422,
        content=APIError(
            error="Validation error",
            error_code=ErrorCode.VALIDATION_ERROR,
            details={"message": str(exc)}
        ).dict()
    )


def handle_not_found_error(request: Request, exc: Exception) -> JSONResponse:
    """Handle not found errors"""
    logging.getLogger(__name__).warning(f"Not found: {str(exc)}")
    return JSONResponse(
        status_code=404,
        content=APIError(
            error="Resource not found",
            error_code=ErrorCode.NOT_FOUND,
            details={"message": str(exc)}
        ).dict()
    )


def handle_unauthorized_error(request: Request, exc: Exception) -> JSONResponse:
    """Handle unauthorized errors"""
    logging.getLogger(__name__).warning(f"Unauthorized access: {str(exc)}")
    return JSONResponse(
        status_code=401,
        content=APIError(
            error="Unauthorized",
            error_code=ErrorCode.UNAUTHORIZED,
            details={"message": str(exc)}
        ).dict()
    )


def handle_forbidden_error(request: Request, exc: Exception) -> JSONResponse:
    """Handle forbidden errors"""
    logging.getLogger(__name__).warning(f"Forbidden access: {str(exc)}")
    return JSONResponse(
        status_code=403,
        content=APIError(
            error="Forbidden",
            error_code=ErrorCode.FORBIDDEN,
            details={"message": str(exc)}
        ).dict()
    )


def handle_rate_limit_error(request: Request, exc: Exception) -> JSONResponse:
    """Handle rate limit errors"""
    logging.getLogger(__name__).warning(f"Rate limit exceeded: {str(exc)}")
    return JSONResponse(
        status_code=429,
        content=APIError(
            error="Rate limit exceeded",
            error_code=ErrorCode.TOO_MANY_REQUESTS,
            details={"message": str(exc)}
        ).dict()
    )


def handle_rag_error(request: Request, exc: RAGError) -> JSONResponse:
    """Handle RAG-specific errors"""
    logger = logging.getLogger(__name__)
    logger.error(f"RAG error: {exc.error_code} - {exc.message}")

    return JSONResponse(
        status_code=500,
        content=exc.to_api_error().dict()
    )


def handle_internal_error(request: Request, exc: Exception) -> JSONResponse:
    """Handle internal server errors"""
    logger = logging.getLogger(__name__)
    logger.error(f"Internal server error: {str(exc)}", exc_info=True)

    return JSONResponse(
        status_code=500,
        content=APIError(
            error="Internal server error",
            error_code=ErrorCode.INTERNAL_ERROR,
            details={"message": "An unexpected error occurred"}
        ).dict()
    )


def add_error_handling_middleware(app):
    """
    Add error handling middleware to the FastAPI application.

    Args:
        app: FastAPI application instance
    """
    # Add exception handlers for different types of errors
    app.add_exception_handler(HTTPException, handle_http_exception)
    app.add_exception_handler(RAGError, handle_rag_error)
    app.add_exception_handler(Exception, handle_internal_error)

    logging.getLogger(__name__).info("Error handling middleware added to application")


async def handle_http_exception(request: Request, exc: HTTPException) -> JSONResponse:
    """Handle HTTP exceptions"""
    logger = logging.getLogger(__name__)
    logger.warning(f"HTTP exception: {exc.status_code} - {exc.detail}")

    # Map HTTP status codes to error codes
    error_code_map = {
        400: ErrorCode.VALIDATION_ERROR,
        401: ErrorCode.UNAUTHORIZED,
        403: ErrorCode.FORBIDDEN,
        404: ErrorCode.NOT_FOUND,
        422: ErrorCode.VALIDATION_ERROR,
        429: ErrorCode.TOO_MANY_REQUESTS,
        500: ErrorCode.INTERNAL_ERROR
    }

    error_code = error_code_map.get(exc.status_code, ErrorCode.INTERNAL_ERROR)

    return JSONResponse(
        status_code=exc.status_code,
        content=APIError(
            error="HTTP Error",
            error_code=error_code,
            details={"message": str(exc.detail)}
        ).dict()
    )


def create_error_response(error_code: ErrorCode, message: str, details: Optional[Dict[str, Any]] = None) -> JSONResponse:
    """
    Create a standardized error response.

    Args:
        error_code: The error code
        message: Error message
        details: Optional additional details

    Returns:
        JSONResponse with standardized error format
    """
    return JSONResponse(
        status_code=500,  # Default to 500, caller should adjust if needed
        content=APIError(
            error=message,
            error_code=error_code,
            details=details
        ).dict()
    )


def log_error(logger: logging.Logger, error: Exception, context: Optional[Dict[str, Any]] = None) -> None:
    """
    Log an error with context.

    Args:
        logger: Logger instance
        error: Exception to log
        context: Optional context information
    """
    context_str = f" (Context: {context})" if context else ""
    logger.error(f"Error occurred: {str(error)}{context_str}", exc_info=True)


def handle_gemini_api_error(exc: Exception) -> RAGError:
    """Handle errors specific to Gemini API calls"""
    logger = logging.getLogger(__name__)
    logger.error(f"Gemini API error: {str(exc)}", exc_info=True)

    return RAGError(
        message="Error calling Gemini API",
        error_code=ErrorCode.GEMINI_API_ERROR,
        details={"original_error": str(exc)}
    )


def handle_qdrant_error(exc: Exception) -> RAGError:
    """Handle errors specific to Qdrant operations"""
    logger = logging.getLogger(__name__)
    logger.error(f"Qdrant error: {str(exc)}", exc_info=True)

    return RAGError(
        message="Error with Qdrant vector database",
        error_code=ErrorCode.QDRANT_CONNECTION_ERROR,
        details={"original_error": str(exc)}
    )