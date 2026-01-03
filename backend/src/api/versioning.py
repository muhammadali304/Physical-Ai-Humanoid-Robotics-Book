"""
API versioning system for the RAG Ingestion Pipeline.
Implements API versioning through URL paths and headers.
"""

from enum import Enum
from typing import Dict, List, Optional, Any
from fastapi import FastAPI, Request, HTTPException, status
from fastapi.routing import APIRoute
from pydantic import BaseModel

from src.utils.logging import get_logger


class APIVersion(str, Enum):
    """
    Supported API versions.
    """
    V1 = "v1"
    V2 = "v2"


class VersionConfig(BaseModel):
    """
    Configuration for an API version.
    """
    version: APIVersion
    deprecated: bool = False
    deprecation_date: Optional[str] = None
    migration_guide: Optional[str] = None
    supported_until: Optional[str] = None


class APIVersionManager:
    """
    Manager for API versioning functionality.
    """

    def __init__(self):
        self.logger = get_logger("api_versioning")
        self.version_configs: Dict[APIVersion, VersionConfig] = {}

        # Initialize with default version configuration
        self.add_version_config(VersionConfig(version=APIVersion.V1))

    def add_version_config(self, config: VersionConfig):
        """
        Add a version configuration.

        Args:
            config: Version configuration to add
        """
        self.version_configs[config.version] = config
        self.logger.info(
            f"Added API version configuration: {config.version}",
            version=config.version,
            deprecated=config.deprecated
        )

    def get_version_config(self, version: APIVersion) -> Optional[VersionConfig]:
        """
        Get configuration for a specific version.

        Args:
            version: API version to get config for

        Returns:
            Version configuration if found, None otherwise
        """
        return self.version_configs.get(version)

    def is_version_supported(self, version: APIVersion) -> bool:
        """
        Check if an API version is supported.

        Args:
            version: API version to check

        Returns:
            True if supported, False otherwise
        """
        config = self.get_version_config(version)
        return config is not None and not config.deprecated

    def get_latest_version(self) -> Optional[APIVersion]:
        """
        Get the latest supported API version.

        Returns:
            Latest supported version or None if none are supported
        """
        for version in reversed(list(APIVersion)):
            if self.is_version_supported(version):
                return version
        return None

    def extract_version_from_path(self, path: str) -> Optional[APIVersion]:
        """
        Extract API version from a path.

        Args:
            path: Request path

        Returns:
            Extracted version or None if not found
        """
        # Look for /api/v1/, /api/v2/, etc. patterns
        path_parts = path.strip('/').split('/')
        if len(path_parts) >= 2 and path_parts[0] == 'api':
            version_part = path_parts[1].lower()
            # Remove 'v' prefix if present
            version_clean = version_part[1:] if version_part.startswith('v') else version_part
            try:
                return APIVersion(version_clean)
            except ValueError:
                return None
        return None

    def validate_version(self, version: APIVersion) -> bool:
        """
        Validate if a version is supported and not deprecated.

        Args:
            version: Version to validate

        Returns:
            True if valid, raises exception if not
        """
        if not self.is_version_supported(version):
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"API version {version} is not supported"
            )

        config = self.get_version_config(version)
        if config and config.deprecated:
            self.logger.warning(
                f"Accessing deprecated API version: {version}",
                version=version,
                deprecation_date=config.deprecation_date
            )

        return True

    def get_version_routes(self, app: FastAPI, version: APIVersion) -> List[APIRoute]:
        """
        Get all routes for a specific version.

        Args:
            app: FastAPI application
            version: API version

        Returns:
            List of APIRoute objects for the version
        """
        version_prefix = f"/api/{version}"
        version_routes = []

        for route in app.routes:
            if isinstance(route, APIRoute) and route.path.startswith(version_prefix):
                version_routes.append(route)

        return version_routes


def get_api_version_from_request(request: Request) -> Optional[APIVersion]:
    """
    Extract API version from the request path.

    Args:
        request: FastAPI request object

    Returns:
        API version if found in path, None otherwise
    """
    version_manager = get_version_manager()
    return version_manager.extract_version_from_path(request.url.path)


def get_version_manager() -> APIVersionManager:
    """
    Get the default version manager instance.

    Returns:
        APIVersionManager instance
    """
    if not hasattr(get_version_manager, '_instance'):
        get_version_manager._instance = APIVersionManager()
    return get_version_manager._instance


def add_version_headers_middleware(app: FastAPI):
    """
    Add middleware to include version information in response headers.

    Args:
        app: FastAPI application instance
    """
    @app.middleware("http")
    async def version_headers_middleware(request: Request, call_next):
        response = await call_next(request)

        # Extract version from path
        version_manager = get_version_manager()
        version = version_manager.extract_version_from_path(request.url.path)

        if version:
            response.headers["X-API-Version"] = version.value

        # Add server info header
        response.headers["X-Server"] = "RAG-Ingestion-Pipeline-API"
        response.headers["X-Server-Version"] = "0.1.0"

        return response


# Common versioning utilities
class APIVersionUtils:
    """
    Utility functions for API versioning.
    """

    @staticmethod
    def get_version_from_request_path(path: str) -> Optional[APIVersion]:
        """
        Extract API version from request path.

        Args:
            path: Request path

        Returns:
            Extracted version or None
        """
        path_parts = path.strip('/').split('/')
        if len(path_parts) >= 2 and path_parts[0] == 'api':
            version_part = path_parts[1].lower()
            version_clean = version_part[1:] if version_part.startswith('v') else version_part
            try:
                return APIVersion(version_clean)
            except ValueError:
                return None
        return None

    @staticmethod
    def create_versioned_response(data: Any, version: APIVersion) -> Dict[str, Any]:
        """
        Create a response with version information.

        Args:
            data: Response data
            version: API version

        Returns:
            Versioned response dictionary
        """
        return {
            "data": data,
            "version": version,
            "timestamp": __import__('datetime').datetime.utcnow().isoformat()
        }

    @staticmethod
    def validate_api_version(version: str) -> Optional[APIVersion]:
        """
        Validate an API version string.

        Args:
            version: Version string to validate

        Returns:
            APIVersion enum if valid, None otherwise
        """
        try:
            return APIVersion(version)
        except ValueError:
            return None


def setup_versioning(app: FastAPI):
    """
    Set up versioning for the FastAPI application.

    Args:
        app: FastAPI application instance
    """
    # Add version headers middleware
    add_version_headers_middleware(app)

    # The versioning is primarily handled by the route prefixes (e.g., /api/v1/)
    # The middleware will automatically add version information to responses