"""
API key authentication for the RAG Ingestion Pipeline API.
Implements secure API key-based authentication for API endpoints.
"""

import hashlib
import secrets
import time
from typing import Optional, List, Dict, Any
from datetime import datetime, timedelta
from dataclasses import dataclass

from fastapi import HTTPException, Request, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel

from src.utils.logging import get_logger


@dataclass
class APIKey:
    """
    API key model with metadata.
    """
    key: str
    key_hash: str
    created_at: datetime
    expires_at: Optional[datetime] = None
    scopes: List[str] = None
    user_id: Optional[str] = None
    name: Optional[str] = None
    rate_limit: Optional[Dict[str, int]] = None  # {window: requests}


class APIKeyStore:
    """
    In-memory store for API keys.
    In production, this would use a database or other persistent storage.
    """

    def __init__(self):
        self.api_keys: Dict[str, APIKey] = {}
        self.logger = get_logger("api_key_store")

    def create_api_key(
        self,
        name: str,
        user_id: Optional[str] = None,
        scopes: List[str] = None,
        expires_in_days: Optional[int] = None,
        rate_limit: Optional[Dict[str, int]] = None
    ) -> APIKey:
        """
        Create a new API key.

        Args:
            name: Name for the API key
            user_id: Associated user ID (optional)
            scopes: List of permissions/scopes
            expires_in_days: Number of days until expiration (optional)
            rate_limit: Rate limiting configuration (optional)

        Returns:
            Created APIKey object
        """
        # Generate a secure API key
        raw_key = f"rag_{secrets.token_urlsafe(32)}"
        key_hash = hashlib.sha256(raw_key.encode()).hexdigest()

        # Calculate expiration
        expires_at = None
        if expires_in_days:
            expires_at = datetime.utcnow() + timedelta(days=expires_in_days)

        api_key = APIKey(
            key=raw_key,
            key_hash=key_hash,
            created_at=datetime.utcnow(),
            expires_at=expires_at,
            scopes=scopes or [],
            user_id=user_id,
            name=name,
            rate_limit=rate_limit
        )

        self.api_keys[key_hash] = api_key

        self.logger.info(
            f"Created new API key: {name}",
            key_name=name,
            user_id=user_id,
            scopes=scopes,
            expires_at=expires_at.isoformat() if expires_at else None
        )

        return api_key

    def get_api_key(self, key_hash: str) -> Optional[APIKey]:
        """
        Get an API key by its hash.

        Args:
            key_hash: Hash of the API key

        Returns:
            APIKey object if found and valid, None otherwise
        """
        if key_hash not in self.api_keys:
            return None

        api_key = self.api_keys[key_hash]

        # Check if key is expired
        if api_key.expires_at and api_key.expires_at < datetime.utcnow():
            self.logger.warning(
                f"API key {api_key.name} has expired",
                key_name=api_key.name,
                user_id=api_key.user_id,
                expired_at=api_key.expires_at.isoformat()
            )
            # Remove expired key
            del self.api_keys[key_hash]
            return None

        return api_key

    def validate_api_key(self, raw_key: str) -> Optional[APIKey]:
        """
        Validate a raw API key.

        Args:
            raw_key: Raw API key string

        Returns:
            APIKey object if valid, None otherwise
        """
        key_hash = hashlib.sha256(raw_key.encode()).hexdigest()
        return self.get_api_key(key_hash)

    def revoke_api_key(self, key_hash: str) -> bool:
        """
        Revoke (delete) an API key.

        Args:
            key_hash: Hash of the API key to revoke

        Returns:
            True if revoked, False if not found
        """
        if key_hash in self.api_keys:
            api_key = self.api_keys[key_hash]
            del self.api_keys[key_hash]
            self.logger.info(
                f"Revoked API key: {api_key.name}",
                key_name=api_key.name,
                user_id=api_key.user_id
            )
            return True
        return False

    def list_api_keys(self, user_id: Optional[str] = None) -> List[APIKey]:
        """
        List all API keys, optionally filtered by user ID.

        Args:
            user_id: Optional user ID to filter by

        Returns:
            List of APIKey objects
        """
        keys = list(self.api_keys.values())

        if user_id:
            keys = [key for key in keys if key.user_id == user_id]

        return keys


class APIKeyAuth:
    """
    API key authentication handler.
    """

    def __init__(self):
        self.security = HTTPBearer(auto_error=False)  # Don't auto-error, we'll handle it
        self.store = APIKeyStore()
        self.logger = get_logger("api_key_auth")

    async def authenticate(self, request: Request) -> Optional[APIKey]:
        """
        Authenticate a request using API key.

        Args:
            request: The incoming request

        Returns:
            APIKey object if authenticated, None otherwise
        """
        # Try to get API key from header
        credentials: HTTPAuthorizationCredentials = await self.security(request)

        if credentials:
            raw_key = credentials.credentials
        else:
            # Try to get API key from query parameter as fallback
            raw_key = request.query_params.get("api_key")

        if not raw_key:
            self.logger.debug("No API key provided in request", path=request.url.path)
            return None

        # Validate the API key
        api_key = self.store.validate_api_key(raw_key)

        if not api_key:
            self.logger.warning(
                "Invalid or expired API key provided",
                client_host=request.client.host if request.client else "unknown"
            )
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid or expired API key"
            )

        # Add API key info to request state for later use
        request.state.api_key = api_key

        self.logger.debug(
            f"Successfully authenticated API key: {api_key.name}",
            key_name=api_key.name,
            user_id=api_key.user_id
        )

        return api_key

    def require_api_key(self, scopes: List[str] = None):
        """
        Decorator/factory to create an authentication dependency with required scopes.

        Args:
            scopes: List of required scopes (optional)

        Returns:
            Dependency function
        """
        async def auth_dependency(request: Request) -> APIKey:
            api_key = await self.authenticate(request)

            if not api_key:
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="Authentication required"
                )

            # Check if required scopes are granted
            if scopes:
                if not api_key.scopes or not all(scope in api_key.scopes for scope in scopes):
                    self.logger.warning(
                        f"API key {api_key.name} missing required scopes",
                        key_name=api_key.name,
                        required_scopes=scopes,
                        granted_scopes=api_key.scopes
                    )
                    raise HTTPException(
                        status_code=status.HTTP_403_FORBIDDEN,
                        detail="Insufficient permissions"
                    )

            return api_key

        return auth_dependency

    def create_api_key(
        self,
        name: str,
        user_id: Optional[str] = None,
        scopes: List[str] = None,
        expires_in_days: Optional[int] = None,
        rate_limit: Optional[Dict[str, int]] = None
    ) -> str:
        """
        Create a new API key (returns the raw key, not the hash).

        Args:
            name: Name for the API key
            user_id: Associated user ID (optional)
            scopes: List of permissions/scopes
            expires_in_days: Number of days until expiration (optional)
            rate_limit: Rate limiting configuration (optional)

        Returns:
            Raw API key string (this is the only time it's returned)
        """
        api_key = self.store.create_api_key(
            name=name,
            user_id=user_id,
            scopes=scopes,
            expires_in_days=expires_in_days,
            rate_limit=rate_limit
        )
        return api_key.key  # Return the raw key (not the hash) - only time this is exposed


class APIKeyResponse(BaseModel):
    """
    Response model for API key operations.
    """
    key: str
    name: str
    created_at: datetime
    expires_at: Optional[datetime] = None
    scopes: List[str] = []
    user_id: Optional[str] = None


class APIKeyListResponse(BaseModel):
    """
    Response model for listing API keys.
    """
    api_keys: List[APIKeyResponse]


# Global instance for use throughout the application
api_key_auth = APIKeyAuth()


def get_api_key_auth() -> APIKeyAuth:
    """
    Get the global API key authentication instance.

    Returns:
        APIKeyAuth instance
    """
    return api_key_auth


# Convenience functions
def create_api_key(
    name: str,
    user_id: Optional[str] = None,
    scopes: List[str] = None,
    expires_in_days: Optional[int] = None,
    rate_limit: Optional[Dict[str, int]] = None
) -> str:
    """
    Create a new API key.

    Args:
        name: Name for the API key
        user_id: Associated user ID (optional)
        scopes: List of permissions/scopes
        expires_in_days: Number of days until expiration (optional)
        rate_limit: Rate limiting configuration (optional)

    Returns:
        Raw API key string
    """
    return api_key_auth.create_api_key(
        name=name,
        user_id=user_id,
        scopes=scopes,
        expires_in_days=expires_in_days,
        rate_limit=rate_limit
    )


def authenticate_request(request: Request) -> Optional[APIKey]:
    """
    Authenticate a request using API key.

    Args:
        request: The incoming request

    Returns:
        APIKey object if authenticated, None otherwise
    """
    return api_key_auth.authenticate(request)