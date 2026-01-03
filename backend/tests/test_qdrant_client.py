"""
Unit tests for the Qdrant client service.
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from typing import List
from uuid import UUID

from src.services.qdrant_client import QdrantClientService
from src.models.embedding import EmbeddingVector


@pytest.fixture
async def qdrant_client():
    """Create a Qdrant client service instance for testing."""
    service = QdrantClientService()
    # Override the actual Qdrant client with a mock
    service.client = MagicMock()
    return service


@pytest.mark.asyncio
async def test_setup_collection_success(qdrant_client):
    """Test successful collection setup."""
    # Mock collection methods
    mock_collection = MagicMock()
    mock_collection.config.params.size = 1024
    mock_collection.config.params.distance = "cosine"
    mock_collection.points_count = 0

    mock_collections = MagicMock()
    mock_collections.collections = []

    qdrant_client.client.get_collections.return_value = mock_collections
    qdrant_client.client.create_collection = MagicMock()

    result = await qdrant_client.setup_collection()

    assert result is True
    qdrant_client.client.create_collection.assert_called_once()


@pytest.mark.asyncio
async def test_setup_collection_already_exists(qdrant_client):
    """Test collection setup when collection already exists."""
    # Mock existing collection
    mock_existing_collection = MagicMock()
    mock_existing_collection.config.params.size = 1024
    mock_existing_collection.config.params.distance = "cosine"
    mock_existing_collection.points_count = 0

    mock_collection_list = MagicMock()
    mock_collection_list.collections = [MagicMock(name=qdrant_client.collection_name)]

    qdrant_client.client.get_collections.return_value = mock_collection_list
    qdrant_client.client.get_collection.return_value = mock_existing_collection

    result = await qdrant_client.setup_collection()

    assert result is True
    # Should not call create_collection since it already exists
    qdrant_client.client.create_collection.assert_not_called()


@pytest.mark.asyncio
async def test_setup_collection_wrong_config(qdrant_client):
    """Test collection setup when existing collection has wrong config."""
    # Mock existing collection with wrong config
    mock_existing_collection = MagicMock()
    mock_existing_collection.config.params.size = 512  # Wrong size
    mock_existing_collection.config.params.distance = "cosine"
    mock_existing_collection.points_count = 0

    mock_collection_list = MagicMock()
    mock_collection_list.collections = [MagicMock(name=qdrant_client.collection_name)]

    qdrant_client.client.get_collections.return_value = mock_collection_list
    qdrant_client.client.get_collection.return_value = mock_existing_collection

    # Should raise an error for wrong configuration
    with pytest.raises(ValueError):
        await qdrant_client.setup_collection()


@pytest.mark.asyncio
async def test_store_embedding_success(qdrant_client):
    """Test successful embedding storage."""
    # Create a test embedding
    embedding = EmbeddingVector(
        vector_data=[0.1] * 1024,
        model_used="cohere/test-model",
        model_version="v1"
    )

    # Mock successful storage
    qdrant_client.client.upsert = MagicMock()

    result = await qdrant_client.store_embedding(embedding)

    assert result is True
    qdrant_client.client.upsert.assert_called_once()


@pytest.mark.asyncio
async def test_store_embedding_batch_success(qdrant_client):
    """Test successful batch embedding storage."""
    # Create test embeddings
    embeddings = [
        EmbeddingVector(
            vector_data=[0.1] * 1024,
            model_used="cohere/test-model",
            model_version="v1"
        ),
        EmbeddingVector(
            vector_data=[0.2] * 1024,
            model_used="cohere/test-model",
            model_version="v1"
        )
    ]

    # Mock successful batch storage
    qdrant_client.client.upsert = MagicMock()

    result = await qdrant_client.store_embeddings_batch(embeddings)

    assert result is True
    qdrant_client.client.upsert.assert_called_once()


@pytest.mark.asyncio
async def test_store_embedding_batch_empty(qdrant_client):
    """Test batch embedding storage with empty list."""
    result = await qdrant_client.store_embeddings_batch([])

    # Should return True for empty list
    assert result is True
    # Should not call upsert for empty list
    qdrant_client.client.upsert.assert_not_called()


@pytest.mark.asyncio
async def test_retrieve_embedding_success(qdrant_client):
    """Test successful embedding retrieval."""
    # Mock the response
    mock_point = MagicMock()
    mock_point.id = "test-id"
    mock_point.vector = [0.1] * 1024
    mock_point.payload = {
        "content_chunk_id": "chunk-123",
        "model_used": "cohere/test-model",
        "model_version": "v1",
        "created_at": "2023-01-01T00:00:00Z"
    }

    qdrant_client.client.retrieve.return_value = [mock_point]

    result = await qdrant_client.retrieve_embedding("test-id")

    assert result is not None
    assert result.id == UUID("test-id")
    assert len(result.vector_data) == 1024
    assert result.model_used == "cohere/test-model"


@pytest.mark.asyncio
async def test_retrieve_embedding_not_found(qdrant_client):
    """Test embedding retrieval when not found."""
    # Mock empty response
    qdrant_client.client.retrieve.return_value = []

    result = await qdrant_client.retrieve_embedding("nonexistent-id")

    # Should return None when not found
    assert result is None


@pytest.mark.asyncio
async def test_search_similar_success(qdrant_client):
    """Test successful similar embeddings search."""
    # Mock search results
    mock_result = MagicMock()
    mock_result.id = "similar-id-1"
    mock_result.score = 0.9
    mock_result.payload = {"content_chunk_id": "chunk-123"}
    mock_result.vector = [0.1] * 1024

    qdrant_client.client.search.return_value = [mock_result]

    query_vector = [0.1] * 1024
    results = await qdrant_client.search_similar(query_vector, top_k=5)

    assert len(results) == 1
    assert results[0]["id"] == "similar-id-1"
    assert results[0]["score"] == 0.9


@pytest.mark.asyncio
async def test_search_similar_wrong_dimensions(qdrant_client):
    """Test similar search with wrong query dimensions."""
    query_vector = [0.1] * 1023  # Wrong dimension

    with pytest.raises(ValueError):
        await qdrant_client.search_similar(query_vector, top_k=5)


@pytest.mark.asyncio
async def test_search_by_content_chunk_success(qdrant_client):
    """Test searching by content chunk ID."""
    # Mock search results
    mock_result = MagicMock()
    mock_result.id = "embedding-id-1"
    mock_result.score = 0.8
    mock_result.payload = {"content_chunk_id": "target-chunk-id"}
    mock_result.vector = [0.1] * 1024

    qdrant_client.client.search.return_value = [mock_result]

    chunk_id = UUID("12345678-1234-5678-1234-567812345678")
    result = await qdrant_client.search_by_content_chunk(chunk_id)

    assert result is not None
    assert result.id == UUID("embedding-id-1")


@pytest.mark.asyncio
async def test_search_by_content_chunk_not_found(qdrant_client):
    """Test searching by content chunk ID when not found."""
    # Mock empty search results
    qdrant_client.client.search.return_value = []

    chunk_id = UUID("12345678-1234-5678-1234-567812345678")
    result = await qdrant_client.search_by_content_chunk(chunk_id)

    # Should return None when not found
    assert result is None


@pytest.mark.asyncio
async def test_delete_embedding_success(qdrant_client):
    """Test successful embedding deletion."""
    qdrant_client.client.delete = MagicMock()

    result = await qdrant_client.delete_embedding("test-id")

    assert result is True
    qdrant_client.client.delete.assert_called_once()


@pytest.mark.asyncio
async def test_get_collection_info_success(qdrant_client):
    """Test successful collection info retrieval."""
    # Mock collection info
    mock_collection = MagicMock()
    mock_collection.config.params.size = 1024
    mock_collection.config.params.distance = "cosine"
    mock_collection.points_count = 100
    mock_collection.indexed_vectors_count = 100

    qdrant_client.client.get_collection.return_value = mock_collection

    info = await qdrant_client.get_collection_info()

    assert info["collection_name"] == qdrant_client.collection_name
    assert info["vector_size"] == 1024
    assert info["distance"] == "cosine"
    assert info["point_count"] == 100


@pytest.mark.asyncio
async def test_health_check_success(qdrant_client):
    """Test successful health check."""
    # Mock successful connection
    mock_collections = MagicMock()
    mock_collections.collections = []
    qdrant_client.client.get_collections.return_value = mock_collections

    result = await qdrant_client.health_check()

    assert result is True


@pytest.mark.asyncio
async def test_health_check_failure(qdrant_client):
    """Test health check failure."""
    # Mock connection failure
    qdrant_client.client.get_collections.side_effect = Exception("Connection failed")

    result = await qdrant_client.health_check()

    assert result is False


@pytest.mark.asyncio
async def test_store_embedding_with_fallback_success(qdrant_client):
    """Test embedding storage with fallback strategy."""
    # Create a test embedding
    embedding = EmbeddingVector(
        vector_data=[0.1] * 1024,
        model_used="cohere/test-model",
        model_version="v1"
    )

    # Mock successful primary storage
    qdrant_client.store_embedding = AsyncMock(return_value=True)

    result = await qdrant_client.store_embedding_with_fallback(embedding)

    assert result is True
    qdrant_client.store_embedding.assert_called_once_with(embedding, None)


@pytest.mark.asyncio
async def test_store_embedding_with_fallback_primary_failure(qdrant_client):
    """Test embedding storage with fallback when primary fails."""
    # Create a test embedding
    embedding = EmbeddingVector(
        vector_data=[0.1] * 1024,
        model_used="cohere/test-model",
        model_version="v1"
    )

    # Mock primary storage failure but fallback success
    qdrant_client.store_embedding = AsyncMock(return_value=False)
    qdrant_client._store_in_fallback = AsyncMock(return_value=True)

    result = await qdrant_client.store_embedding_with_fallback(embedding)

    assert result is True
    qdrant_client.store_embedding.assert_called_once_with(embedding, None)
    qdrant_client._store_in_fallback.assert_called_once()


@pytest.mark.asyncio
async def test_retrieve_embedding_with_fallback_success(qdrant_client):
    """Test embedding retrieval with fallback strategy."""
    # Mock successful primary retrieval
    mock_embedding = MagicMock()
    qdrant_client.retrieve_embedding = AsyncMock(return_value=mock_embedding)

    result = await qdrant_client.retrieve_embedding_with_fallback("test-id")

    assert result is mock_embedding
    qdrant_client.retrieve_embedding.assert_called_once_with("test-id")


@pytest.mark.asyncio
async def test_retrieve_embedding_with_fallback_primary_failure(qdrant_client):
    """Test embedding retrieval with fallback when primary fails."""
    # Mock primary retrieval failure but fallback success
    qdrant_client.retrieve_embedding = AsyncMock(return_value=None)
    mock_fallback_embedding = MagicMock()
    qdrant_client._retrieve_from_fallback = AsyncMock(return_value=mock_fallback_embedding)

    result = await qdrant_client.retrieve_embedding_with_fallback("test-id")

    assert result is mock_fallback_embedding
    qdrant_client.retrieve_embedding.assert_called_once_with("test-id")
    qdrant_client._retrieve_from_fallback.assert_called_once_with("test-id")