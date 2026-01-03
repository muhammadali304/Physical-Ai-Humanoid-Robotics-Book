"""
Configuration for pytest.
"""

import pytest
import asyncio
from unittest.mock import AsyncMock, MagicMock, patch
from typing import AsyncGenerator

from src.config.settings import settings


@pytest.fixture(scope="session")
def event_loop():
    """Create an instance of the default event loop for test session."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest.fixture
def mock_settings():
    """Mock settings for testing."""
    with patch('src.config.settings.settings') as mock_settings:
        mock_settings.cohere_api_key = "test-cohere-key"
        mock_settings.qdrant_url = "http://localhost:6333"
        mock_settings.qdrant_api_key = "test-qdrant-key"
        mock_settings.qdrant_collection_name = "test_collection"
        mock_settings.rate_limit_delay = 0.1  # Fast for tests
        mock_settings.max_retries = 1  # No retries in tests
        yield mock_settings


@pytest.fixture
def mock_http_client():
    """Mock HTTP client for testing."""
    with patch('src.utils.http_client.HttpClient') as mock:
        mock_instance = AsyncMock()
        mock.return_value = mock_instance
        yield mock_instance


@pytest.fixture
def mock_cohere_client():
    """Mock Cohere client for testing."""
    with patch('src.services.embedding_service.cohere.AsyncClient') as mock:
        mock_client = AsyncMock()
        mock.return_value = mock_client

        # Mock the embed method
        mock_client.embed.return_value = AsyncMock()
        mock_client.embed.return_value.embeddings = [[0.1] * 1024]  # Mock 1024-dim embedding

        yield mock_client


@pytest.fixture
def mock_qdrant_client():
    """Mock Qdrant client for testing."""
    with patch('src.services.qdrant_client.QdrantClient') as mock:
        mock_client = MagicMock()
        mock.return_value = mock_client

        # Mock collection methods
        mock_client.get_collections.return_value = MagicMock()
        mock_client.get_collections.return_value.collections = []

        # Mock upsert method
        mock_client.upsert = MagicMock()

        # Mock retrieve method
        mock_client.retrieve = MagicMock()
        mock_client.retrieve.return_value = []

        # Mock search method
        mock_client.search = MagicMock()
        mock_client.search.return_value = []

        yield mock_client