"""
Environment validation for the RAG Ingestion Pipeline.
Validates that all required environment variables and settings are properly configured.
"""

import os
import sys
from typing import List, Tuple
from urllib.parse import urlparse

from src.config.settings import settings
from src.utils.logging import get_logger


class EnvironmentValidator:
    """Validates environment configuration and required settings."""

    def __init__(self):
        self.logger = get_logger("environment_validator")
        self.errors: List[str] = []
        self.warnings: List[str] = []

    def validate_cohere_api_key(self) -> bool:
        """Validate that Cohere API key is properly set."""
        api_key = settings.cohere_api_key
        if not api_key:
            self.errors.append("COHERE_API_KEY environment variable is not set")
            return False

        if len(api_key) < 10:
            self.errors.append("COHERE_API_KEY appears to be invalid (too short)")
            return False

        # Basic format check (Cohere keys typically start with "co-")
        if not api_key.startswith("co-"):
            self.warnings.append("COHERE_API_KEY format may be incorrect (should start with 'co-')")

        return True

    def validate_qdrant_config(self) -> bool:
        """Validate Qdrant configuration."""
        url = settings.qdrant_url
        api_key = settings.qdrant_api_key

        is_valid = True

        if not url:
            self.errors.append("QDRANT_URL environment variable is not set")
            is_valid = False
        else:
            try:
                parsed = urlparse(url)
                if not parsed.scheme or not parsed.netloc:
                    self.errors.append(f"QDRANT_URL is not a valid URL: {url}")
                    is_valid = False
            except Exception as e:
                self.errors.append(f"Error parsing QDRANT_URL: {str(e)}")
                is_valid = False

        if not api_key:
            self.errors.append("QDRANT_API_KEY environment variable is not set")
            is_valid = False
        elif len(api_key) < 10:
            self.errors.append("QDRANT_API_KEY appears to be invalid (too short)")
            is_valid = False

        return is_valid

    def validate_redis_config(self) -> bool:
        """Validate Redis configuration."""
        # For Redis, we just validate the settings are reasonable
        if settings.redis_port < 1 or settings.redis_port > 65535:
            self.errors.append(f"Invalid Redis port: {settings.redis_port}")
            return False

        return True

    def validate_paths_and_directories(self) -> bool:
        """Validate that required paths and directories are accessible."""
        # Check if the backend directory exists (where we're running from)
        import pathlib
        current_dir = pathlib.Path(__file__).parent.parent.parent
        if not current_dir.exists():
            self.errors.append(f"Backend directory does not exist: {current_dir}")
            return False

        return True

    def validate_settings_consistency(self) -> bool:
        """Validate consistency between related settings."""
        is_valid = True

        # Check that max_chunk_size is within valid range
        if settings.max_chunk_size < 50 or settings.max_chunk_size > 2000:
            self.errors.append(f"max_chunk_size ({settings.max_chunk_size}) is outside valid range (50-2000)")
            is_valid = False

        # Check that chunk_overlap is between 0 and 1
        if settings.chunk_overlap < 0 or settings.chunk_overlap > 1:
            self.errors.append(f"chunk_overlap ({settings.chunk_overlap}) must be between 0 and 1")
            is_valid = False

        # Check that rate_limit_delay is positive
        if settings.rate_limit_delay <= 0:
            self.errors.append(f"rate_limit_delay ({settings.rate_limit_delay}) must be positive")
            is_valid = False

        # Check that max_workers is reasonable
        if settings.max_workers < 1 or settings.max_workers > 50:
            self.warnings.append(f"max_workers ({settings.max_workers}) may be outside reasonable range (1-50)")

        return is_valid

    def validate_all(self) -> Tuple[bool, List[str], List[str]]:
        """Run all validations and return (is_valid, errors, warnings)."""
        self.logger.info("Starting environment validation")

        validations = [
            self.validate_cohere_api_key(),
            self.validate_qdrant_config(),
            self.validate_redis_config(),
            self.validate_paths_and_directories(),
            self.validate_settings_consistency(),
        ]

        is_valid = all(validations)

        self.logger.info(
            f"Environment validation completed: {'PASS' if is_valid else 'FAIL'}",
            error_count=len(self.errors),
            warning_count=len(self.warnings)
        )

        if self.errors:
            for error in self.errors:
                self.logger.error(error)

        if self.warnings:
            for warning in self.warnings:
                self.logger.warning(warning)

        return is_valid, self.errors, self.warnings

    def assert_valid(self, exit_on_error: bool = True) -> bool:
        """Assert that environment is valid, optionally exiting on error."""
        is_valid, errors, warnings = self.validate_all()

        if not is_valid and exit_on_error:
            self.logger.error("Environment validation failed, exiting")
            sys.exit(1)

        return is_valid


def validate_environment(exit_on_error: bool = True) -> bool:
    """Convenience function to validate environment."""
    validator = EnvironmentValidator()
    return validator.assert_valid(exit_on_error)


def get_validation_report() -> Tuple[bool, List[str], List[str]]:
    """Get validation report without exiting."""
    validator = EnvironmentValidator()
    return validator.validate_all()


# Run validation if this module is executed directly
if __name__ == "__main__":
    is_valid, errors, warnings = get_validation_report()

    print(f"Environment validation: {'PASS' if is_valid else 'FAIL'}")

    if errors:
        print("\nErrors:")
        for error in errors:
            print(f"  - {error}")

    if warnings:
        print("\nWarnings:")
        for warning in warnings:
            print(f"  - {warning}")

    if is_valid:
        print("\nAll validations passed!")
    else:
        sys.exit(1 if errors else 0)