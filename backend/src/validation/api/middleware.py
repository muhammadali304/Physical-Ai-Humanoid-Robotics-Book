from fastapi import Request, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from typing import Optional
import time
from ..config import settings


class AuthenticationMiddleware:
    def __init__(self):
        self.security = HTTPBearer()
        self.api_key = settings.validation_api_key
        # Simple in-memory rate limiting store
        self.request_store = {}

    async def authenticate(self, request: Request) -> bool:
        """
        Authenticate request using API key
        """
        auth_header = request.headers.get("Authorization")
        if not auth_header or not auth_header.startswith("Bearer "):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Missing or invalid Authorization header"
            )

        token = auth_header[7:]  # Remove "Bearer " prefix
        if token != self.api_key:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid API key"
            )

        return True

    async def rate_limit(self, request: Request) -> bool:
        """
        Implement rate limiting for validation endpoints
        """
        client_ip = request.client.host
        current_time = time.time()

        # Parse rate limit settings (format: "requests/period" like "100/minute")
        rate_limit_parts = settings.validation_rate_limit.split("/")
        requests_limit = int(rate_limit_parts[0])
        period = rate_limit_parts[1]  # minute, hour, etc.

        # Convert period to seconds
        if "minute" in period:
            period_seconds = 60
        elif "hour" in period:
            period_seconds = 3600
        else:
            period_seconds = 60  # default to minute

        # Initialize client record if not exists
        if client_ip not in self.request_store:
            self.request_store[client_ip] = []

        # Remove requests older than the rate limit window
        self.request_store[client_ip] = [
            req_time for req_time in self.request_store[client_ip]
            if current_time - req_time <= period_seconds
        ]

        # Check if client has exceeded rate limit
        if len(self.request_store[client_ip]) >= requests_limit:
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail="Rate limit exceeded"
            )

        # Add current request to store
        self.request_store[client_ip].append(current_time)

        return True


# Global instance
auth_middleware = AuthenticationMiddleware()