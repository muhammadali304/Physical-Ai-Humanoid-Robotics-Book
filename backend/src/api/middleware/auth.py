"""
Authentication middleware for the RAG Ingestion Pipeline API.
Provides API key authentication for protected endpoints.
"""

from typing import Optional, Callable, Awaitable
from fastapi import Request, HTTPException, status
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response
from starlette.types import ASGIApp

from src.api.auth.api_key_auth import api_key_auth
from src.utils.logging import get_logger


class AuthenticationMiddleware(BaseHTTPMiddleware):
    """
    Authentication middleware that validates API keys for protected routes.
    """

    def __init__(
        self,
        app: ASGIApp,
        protected_routes: Optional[list] = None,
        excluded_routes: Optional[list] = None,
        require_auth_by_default: bool = False
    ):
        super().__init__(app)
        self.protected_routes = protected_routes or []
        self.excluded_routes = excluded_routes or []
        self.require_auth_by_default = require_auth_by_default
        self.logger = get_logger("auth_middleware")

    async def dispatch(self, request: Request, call_next: Callable[[Request], Awaitable[Response]]) -> Response:
        """
        Process the request and apply authentication if required.

        Args:
            request: The incoming request
            call_next: Function to call the next middleware/route handler

        Returns:
            Response from the next handler or authentication response
        """
        path = request.url.path
        method = request.method

        # Determine if authentication is required for this route
        requires_auth = self._requires_authentication(path, method)

        if requires_auth:
            # Perform authentication
            try:
                api_key = await api_key_auth.authenticate(request)
                if not api_key:
                    raise HTTPException(
                        status_code=status.HTTP_401_UNAUTHORIZED,
                        detail="Authentication required"
                    )
            except HTTPException:
                # Re-raise HTTP exceptions (like 401 Unauthorized)
                raise
            except Exception as e:
                self.logger.error(
                    f"Authentication error for {path}",
                    path=path,
                    method=method,
                    error=str(e)
                )
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="Authentication failed"
                )

        # Continue with the request
        response = await call_next(request)
        return response

    def _requires_authentication(self, path: str, method: str) -> bool:
        """
        Determine if a route requires authentication.

        Args:
            path: Request path
            method: Request method

        Returns:
            True if authentication is required, False otherwise
        """
        # Check if path is explicitly excluded from authentication
        for excluded_route in self.excluded_routes:
            if path.startswith(excluded_route):
                return False

        # If require_auth_by_default is True, all routes except explicitly excluded require auth
        if self.require_auth_by_default:
            # Check if path is in protected routes (explicitly protected)
            for protected_route in self.protected_routes:
                if path.startswith(protected_route):
                    return True
            # If not in protected routes, it might be in excluded routes
            default_excluded = ["/", "/health", "/docs", "/redoc", "/openapi.json"]
            return not any(path.startswith(route) for route in default_excluded + self.excluded_routes)

        # Otherwise, only explicitly protected routes require auth
        for protected_route in self.protected_routes:
            if path.startswith(protected_route):
                return True

        return False


def add_authentication_middleware(
    app,
    protected_routes: Optional[list] = None,
    excluded_routes: Optional[list] = None,
    require_auth_by_default: bool = False
):
    """
    Add authentication middleware to the FastAPI application.

    Args:
        app: FastAPI application instance
        protected_routes: List of routes that require authentication
        excluded_routes: List of routes that should be excluded from authentication
        require_auth_by_default: Whether to require auth by default
    """
    app.add_middleware(
        AuthenticationMiddleware,
        protected_routes=protected_routes or ["/api/v1/"],
        excluded_routes=excluded_routes or [],
        require_auth_by_default=require_auth_by_default
    )