"""
Authentication middleware for the RAG Agent Backend.

This module provides authentication functionality for protecting API endpoints
following the implementation plan requirements.
"""

import asyncio
from typing import Optional, Callable, Any
from fastapi import Request, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from fastapi.security.api_key import APIKeyHeader
import logging
from src.config.settings import settings


class AuthMiddleware:
    """Middleware class for handling authentication"""

    def __init__(self):
        """Initialize the authentication middleware"""
        self.logger = logging.getLogger(__name__)
        self.required_auth = settings.require_auth if hasattr(settings, 'require_auth') else True
        self.api_key_header = APIKeyHeader(name="Authorization", auto_error=False)

    async def authenticate_request(self, request: Request) -> bool:
        """Authenticate an incoming request"""
        try:
            # For this implementation, we'll implement a simple API key check
            # In a real implementation, this would integrate with a proper auth system

            auth_header = request.headers.get("Authorization")
            if not auth_header:
                if self.required_auth:
                    self.logger.warning("Authentication required but no Authorization header provided")
                    raise HTTPException(
                        status_code=status.HTTP_401_UNAUTHORIZED,
                        detail="Authorization header required"
                    )
                else:
                    # If auth is not required, allow the request
                    return True

            # Check if the API key is valid
            # Format: "Bearer <token>" or just "<token>" depending on the auth scheme
            if auth_header.startswith("Bearer "):
                token = auth_header[7:]  # Remove "Bearer " prefix
            else:
                token = auth_header

            # Validate the token against the expected value
            expected_token = getattr(settings, 'api_key', None) or getattr(settings, 'gemini_api_key', None) or getattr(settings, 'cohere_api_key', None)

            if not expected_token:
                self.logger.warning("No API key configured in settings")
                # If no expected token is configured, we might allow access depending on environment
                if settings.debug:
                    self.logger.info("Debug mode: allowing access without valid token")
                    return True
                else:
                    raise HTTPException(
                        status_code=status.HTTP_401_UNAUTHORIZED,
                        detail="API key not configured"
                    )

            if token != expected_token:
                self.logger.warning(f"Invalid API key provided: {token[:8]}...")
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="Invalid API key"
                )

            self.logger.info("Request authenticated successfully")
            return True

        except HTTPException:
            # Re-raise HTTP exceptions to be handled by FastAPI
            raise
        except Exception as e:
            self.logger.error(f"Error during authentication: {str(e)}")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Authentication error"
            )

    async def __call__(self, request: Request, call_next: Callable) -> Any:
        """Middleware call implementation"""
        try:
            # Skip authentication for health check and other public endpoints
            if request.url.path in ["/health", "/docs", "/redoc", "/openapi.json"]:
                response = await call_next(request)
                return response

            # Authenticate the request
            is_authenticated = await self.authenticate_request(request)
            if not is_authenticated:
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="Authentication failed"
                )

            # Add authentication context to request
            request.state.authenticated = True

            # Continue with the request
            response = await call_next(request)
            return response

        except HTTPException:
            # Re-raise HTTP exceptions
            raise
        except Exception as e:
            self.logger.error(f"Unexpected error in auth middleware: {str(e)}")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Internal server error"
            )


# Function to add authentication middleware to FastAPI app
def add_authentication_middleware(app, protected_routes=None):
    """
    Add authentication middleware to the FastAPI application.

    Args:
        app: FastAPI application instance
        protected_routes: List of routes that require authentication (if None, all routes except public ones are protected)
    """
    from starlette.middleware.base import BaseHTTPMiddleware

    # Use our custom middleware
    auth_middleware = AuthMiddleware()

    # Add the middleware to the app
    app.add_middleware(BaseHTTPMiddleware, dispatch=auth_middleware.__call__)

    logging.getLogger(__name__).info("Authentication middleware added to application")