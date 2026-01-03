"""
Response serialization utilities for the RAG Ingestion Pipeline API.
Provides consistent serialization of API responses.
"""

from typing import Any, Dict, List, Union, Optional
from datetime import datetime
from uuid import UUID
import json

from pydantic import BaseModel
from fastapi import Response
from fastapi.responses import JSONResponse

from src.utils.logging import get_logger


class ResponseSerializer:
    """
    Utility class for serializing API responses consistently.
    """

    def __init__(self):
        self.logger = get_logger("response_serializer")

    def serialize(self, data: Any) -> Any:
        """
        Serialize data for API response.

        Args:
            data: Data to serialize

        Returns:
            Serialized data
        """
        try:
            if isinstance(data, BaseModel):
                # Pydantic models can be directly converted to dict
                return data.dict()
            elif isinstance(data, (list, tuple)):
                # Recursively serialize list items
                return [self.serialize(item) for item in data]
            elif isinstance(data, dict):
                # Recursively serialize dict values
                return {key: self.serialize(value) for key, value in data.items()}
            elif isinstance(data, UUID):
                # Convert UUID to string
                return str(data)
            elif isinstance(data, datetime):
                # Convert datetime to ISO format string
                return data.isoformat()
            elif isinstance(data, (str, int, float, bool)) or data is None:
                # Primitive types are already serializable
                return data
            else:
                # For other types, try to convert to dict or str
                if hasattr(data, '__dict__'):
                    return self.serialize(data.__dict__)
                else:
                    return str(data)
        except Exception as e:
            self.logger.error(f"Error serializing data: {str(e)}", error=str(e))
            return str(data)

    def create_response(self, data: Any, status_code: int = 200, headers: Optional[Dict[str, str]] = None) -> Response:
        """
        Create a consistent API response.

        Args:
            data: Response data
            status_code: HTTP status code
            headers: Additional headers

        Returns:
            FastAPI Response object
        """
        serialized_data = self.serialize(data)
        return JSONResponse(
            content=serialized_data,
            status_code=status_code,
            headers=headers or {}
        )

    def create_success_response(self, data: Any = None, message: str = "Success", status_code: int = 200) -> Response:
        """
        Create a success response with consistent format.

        Args:
            data: Response data
            message: Success message
            status_code: HTTP status code

        Returns:
            FastAPI Response object
        """
        response_data = {
            "success": True,
            "message": message,
            "data": self.serialize(data),
            "timestamp": datetime.utcnow().isoformat()
        }
        return self.create_response(response_data, status_code)

    def create_error_response(self, message: str, status_code: int = 400, error_code: Optional[str] = None) -> Response:
        """
        Create an error response with consistent format.

        Args:
            message: Error message
            status_code: HTTP status code
            error_code: Application-specific error code

        Returns:
            FastAPI Response object
        """
        response_data = {
            "success": False,
            "message": message,
            "error_code": error_code,
            "timestamp": datetime.utcnow().isoformat()
        }
        return self.create_response(response_data, status_code)

    def create_paginated_response(
        self,
        items: List[Any],
        total: int,
        page: int = 1,
        limit: int = 10,
        message: str = "Success"
    ) -> Response:
        """
        Create a paginated response with consistent format.

        Args:
            items: List of items
            total: Total number of items
            page: Current page number
            limit: Number of items per page
            message: Success message

        Returns:
            FastAPI Response object
        """
        # Calculate pagination details
        pages = (total + limit - 1) // limit  # Ceiling division
        has_next = page < pages
        has_prev = page > 1

        response_data = {
            "success": True,
            "message": message,
            "data": {
                "items": self.serialize(items),
                "pagination": {
                    "total": total,
                    "pages": pages,
                    "current_page": page,
                    "per_page": limit,
                    "has_next": has_next,
                    "has_prev": has_prev,
                    "next_page": page + 1 if has_next else None,
                    "prev_page": page - 1 if has_prev else None
                }
            },
            "timestamp": datetime.utcnow().isoformat()
        }

        return self.create_response(response_data, 200)


class CustomJSONEncoder(json.JSONEncoder):
    """
    Custom JSON encoder for handling special data types.
    """

    def default(self, obj):
        """
        Encode special objects to JSON-compatible format.

        Args:
            obj: Object to encode

        Returns:
            JSON-compatible representation of the object
        """
        if isinstance(obj, UUID):
            return str(obj)
        elif isinstance(obj, datetime):
            return obj.isoformat()
        elif isinstance(obj, BaseModel):
            return obj.dict()
        elif isinstance(obj, set):
            return list(obj)  # Convert sets to lists
        elif hasattr(obj, '__dict__'):
            return obj.__dict__

        # Let the base class handle the default
        return super().default(obj)


def serialize_response(data: Any) -> Any:
    """
    Serialize response data using the default serializer.

    Args:
        data: Data to serialize

    Returns:
        Serialized data
    """
    serializer = ResponseSerializer()
    return serializer.serialize(data)


def create_success_response(data: Any = None, message: str = "Success", status_code: int = 200) -> Response:
    """
    Create a success response using the default serializer.

    Args:
        data: Response data
        message: Success message
        status_code: HTTP status code

    Returns:
        FastAPI Response object
    """
    serializer = ResponseSerializer()
    return serializer.create_success_response(data, message, status_code)


def create_error_response(message: str, status_code: int = 400, error_code: Optional[str] = None) -> Response:
    """
    Create an error response using the default serializer.

    Args:
        message: Error message
        status_code: HTTP status code
        error_code: Application-specific error code

    Returns:
        FastAPI Response object
    """
    serializer = ResponseSerializer()
    return serializer.create_error_response(message, status_code, error_code)


def create_paginated_response(
    items: List[Any],
    total: int,
    page: int = 1,
    limit: int = 10,
    message: str = "Success"
) -> Response:
    """
    Create a paginated response using the default serializer.

    Args:
        items: List of items
        total: Total number of items
        page: Current page number
        limit: Number of items per page
        message: Success message

    Returns:
        FastAPI Response object
    """
    serializer = ResponseSerializer()
    return serializer.create_paginated_response(items, total, page, limit, message)


# Common response formats
class APIResponse:
    """
    Standard API response formats.
    """

    @staticmethod
    def success(data: Any = None, message: str = "Success", meta: Optional[Dict] = None) -> Dict[str, Any]:
        """
        Create a standard success response format.

        Args:
            data: Response data
            message: Success message
            meta: Additional metadata

        Returns:
            Dictionary with standard success format
        """
        response = {
            "success": True,
            "message": message,
            "data": serialize_response(data),
            "timestamp": datetime.utcnow().isoformat()
        }

        if meta:
            response["meta"] = meta

        return response

    @staticmethod
    def error(message: str, error_code: Optional[str] = None, details: Optional[Dict] = None) -> Dict[str, Any]:
        """
        Create a standard error response format.

        Args:
            message: Error message
            error_code: Application-specific error code
            details: Additional error details

        Returns:
            Dictionary with standard error format
        """
        response = {
            "success": False,
            "message": message,
            "timestamp": datetime.utcnow().isoformat()
        }

        if error_code:
            response["error_code"] = error_code

        if details:
            response["details"] = details

        return response

    @staticmethod
    def paginated(
        items: List[Any],
        total: int,
        page: int = 1,
        limit: int = 10,
        message: str = "Success"
    ) -> Dict[str, Any]:
        """
        Create a standard paginated response format.

        Args:
            items: List of items
            total: Total number of items
            page: Current page number
            limit: Number of items per page
            message: Success message

        Returns:
            Dictionary with standard paginated format
        """
        pages = (total + limit - 1) // limit  # Ceiling division
        has_next = page < pages
        has_prev = page > 1

        return {
            "success": True,
            "message": message,
            "data": {
                "items": serialize_response(items),
                "pagination": {
                    "total": total,
                    "pages": pages,
                    "current_page": page,
                    "per_page": limit,
                    "has_next": has_next,
                    "has_prev": has_prev,
                    "next_page": page + 1 if has_next else None,
                    "prev_page": page - 1 if has_prev else None
                }
            },
            "timestamp": datetime.utcnow().isoformat()
        }


# Decorator for automatic response serialization
def serialize_response_decorator(func):
    """
    Decorator to automatically serialize function return values.

    Args:
        func: Function to decorate

    Returns:
        Decorated function
    """
    async def wrapper(*args, **kwargs):
        result = await func(*args, **kwargs)
        return serialize_response(result)
    return wrapper