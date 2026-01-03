"""
Unit tests for the Cohere embedding service.
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from typing import List
import numpy as np

from src.services.embedding_service import CohereEmbeddingService
from src.models.embedding import EmbeddingVector
from src.models.chunk import ContentChunk
from src.config.settings import settings


@pytest.fixture
async def embedding_service():
    """Create an embedding service instance for testing."""
    service = CohereEmbeddingService()
    # Override the Cohere client with a mock
    service.client = AsyncMock()
    service.qdrant_client = AsyncMock()
    return service


@pytest.mark.asyncio
async def test_generate_embedding_success(embedding_service):
    """Test successful embedding generation."""
    # Mock the Cohere API response
    mock_response = AsyncMock()
    mock_response.embeddings = [[0.1, 0.2, 0.3] + [0.0] * 1021]  # 1024-dimensional vector
    embedding_service.client.embed.return_value = mock_response

    # Mock Qdrant storage success
    embedding_service.qdrant_client.store_embedding.return_value = True

    text = "Test text for embedding"
    result = await embedding_service.generate_embedding(text)

    # Verify the result
    assert isinstance(result, EmbeddingVector)
    assert len(result.vector_data) == 1024  # Expected dimension
    assert result.model_used == "cohere/embed-multilingual-v3.0"
    assert result.model_version == "v3"

    # Verify API call
    embedding_service.client.embed.assert_called_once()
    call_args = embedding_service.client.embed.call_args
    assert call_args[1]["texts"] == [text]


@pytest.mark.asyncio
async def test_generate_embedding_batch_success(embedding_service):
    """Test successful batch embedding generation."""
    # Mock the Cohere API response for batch
    mock_response = AsyncMock()
    mock_response.embeddings = [
        [0.1, 0.2, 0.3] + [0.0] * 1021,  # First embedding
        [0.4, 0.5, 0.6] + [0.0] * 1021,  # Second embedding
    ]
    embedding_service.client.embed.return_value = mock_response

    texts = ["Text 1", "Text 2"]
    result = await embedding_service._generate_embedding_batch(texts)

    # Verify the results
    assert len(result) == 2
    for embedding in result:
        assert isinstance(embedding, EmbeddingVector)
        assert len(embedding.vector_data) == 1024

    # Verify API call
    embedding_service.client.embed.assert_called_once()
    call_args = embedding_service.client.embed.call_args
    assert call_args[1]["texts"] == texts


@pytest.mark.asyncio
async def test_generate_embedding_cohere_error_fallback(embedding_service):
    """Test fallback when Cohere API fails."""
    # Mock Cohere API to raise an exception
    embedding_service.client.embed.side_effect = Exception("API Error")

    text = "Test text for embedding"
    result = await embedding_service.generate_embedding(text)

    # Should return a fallback embedding
    assert isinstance(result, EmbeddingVector)
    assert len(result.vector_data) == 1024
    assert "fallback" in result.model_used


@pytest.mark.asyncio
async def test_generate_embedding_from_chunk(embedding_service):
    """Test generating embedding from a content chunk."""
    # Mock the Cohere API response
    mock_response = AsyncMock()
    mock_response.embeddings = [[0.1, 0.2, 0.3] + [0.0] * 1021]  # 1024-dimensional vector
    embedding_service.client.embed.return_value = mock_response

    # Create a test chunk
    chunk = ContentChunk(
        id="test-id",
        source_url="https://example.com",
        page_title="Test Page",
        section_heading="Test Section",
        chunk_index=1,
        content="This is test content for embedding generation.",
        token_count=10
    )

    result = await embedding_service.generate_embedding_from_chunk(chunk)

    # Verify the result
    assert isinstance(result, EmbeddingVector)
    assert result.content_chunk_id == chunk.id

    # Verify API call with combined text
    embedding_service.client.embed.assert_called_once()
    call_args = embedding_service.client.embed.call_args
    called_text = call_args[1]["texts"][0]
    assert "Test Page" in called_text
    assert "Test Section" in called_text
    assert "This is test content" in called_text


@pytest.mark.asyncio
async def test_validate_embedding_success(embedding_service):
    """Test successful embedding validation."""
    # Create a valid embedding
    valid_embedding = EmbeddingVector(
        vector_data=[0.1] * 1024,
        model_used="cohere/embed-multilingual-v3.0",
        model_version="v3"
    )

    result = await embedding_service.validate_embedding(valid_embedding)
    assert result is True


@pytest.mark.asyncio
async def test_validate_embedding_invalid_dimensions(embedding_service):
    """Test embedding validation with invalid dimensions."""
    # Create an invalid embedding with wrong dimensions
    invalid_embedding = EmbeddingVector(
        vector_data=[0.1] * 1023,  # Wrong dimension
        model_used="cohere/embed-multilingual-v3.0",
        model_version="v3"
    )

    result = await embedding_service.validate_embedding(invalid_embedding)
    assert result is False


@pytest.mark.asyncio
async def test_store_embedding_success(embedding_service):
    """Test successful embedding storage."""
    # Create a test embedding
    embedding = EmbeddingVector(
        vector_data=[0.1] * 1024,
        model_used="cohere/embed-multilingual-v3.0",
        model_version="v3"
    )

    # Mock successful storage
    embedding_service.qdrant_client.store_embedding.return_value = True

    result = await embedding_service.store_embedding(embedding)

    assert result is True
    embedding_service.qdrant_client.store_embedding.assert_called_once_with(
        embedding, None, None
    )


@pytest.mark.asyncio
async def test_store_embedding_failure(embedding_service):
    """Test embedding storage failure."""
    # Create a test embedding
    embedding = EmbeddingVector(
        vector_data=[0.1] * 1024,
        model_used="cohere/embed-multilingual-v3.0",
        model_version="v3"
    )

    # Mock storage failure
    embedding_service.qdrant_client.store_embedding.return_value = False

    result = await embedding_service.store_embedding(embedding)

    assert result is False


@pytest.mark.asyncio
async def test_search_similar_embeddings(embedding_service):
    """Test similar embeddings search."""
    # Mock the search response
    mock_results = [
        {
            "id": "test-id-1",
            "score": 0.9,
            "payload": {"content_chunk_id": "chunk-1"},
            "vector": [0.1] * 1024
        },
        {
            "id": "test-id-2",
            "score": 0.8,
            "payload": {"content_chunk_id": "chunk-2"},
            "vector": [0.2] * 1024
        }
    ]
    embedding_service.qdrant_client.search_similar.return_value = mock_results

    query_text = "test query"
    # Mock the embedding generation for the query
    mock_query_embedding = AsyncMock()
    mock_query_embedding.vector_data = [0.1] * 1024
    embedding_service.generate_embedding = AsyncMock(return_value=mock_query_embedding)

    results = await embedding_service.search_similar_embeddings(query_text, top_k=2)

    assert len(results) == 2
    assert results[0]["score"] == 0.9
    assert results[1]["score"] == 0.8


@pytest.mark.asyncio
async def test_generate_and_store_embedding_success(embedding_service):
    """Test generating and storing an embedding in one operation."""
    # Mock the Cohere API response
    mock_response = AsyncMock()
    mock_response.embeddings = [[0.1, 0.2, 0.3] + [0.0] * 1021]  # 1024-dimensional vector
    embedding_service.client.embed.return_value = mock_response

    # Mock successful validation and storage
    embedding_service.validate_embedding = AsyncMock(return_value=True)
    embedding_service.qdrant_client.store_embedding.return_value = True

    text = "Test text for embedding"
    result = await embedding_service.generate_and_store_embedding(text)

    # Verify the result
    assert result is not None
    assert isinstance(result, EmbeddingVector)


@pytest.mark.asyncio
async def test_generate_and_store_embedding_validation_failure(embedding_service):
    """Test generating and storing when validation fails."""
    # Mock the Cohere API response
    mock_response = AsyncMock()
    mock_response.embeddings = [[0.1] * 1023]  # Invalid dimension
    embedding_service.client.embed.return_value = mock_response

    # Mock validation failure
    embedding_service.validate_embedding = AsyncMock(return_value=False)

    text = "Test text for embedding"
    result = await embedding_service.generate_and_store_embedding(text)

    # Should return None due to validation failure
    assert result is None


@pytest.mark.asyncio
async def test_get_embedding_info(embedding_service):
    """Test getting embedding service information."""
    info = await embedding_service.get_embedding_info()

    assert "model" in info
    assert "dimensions" in info
    assert info["dimensions"] == 1024
    assert info["model"] == "embed-multilingual-v3.0"
    assert info["api_provider"] == "cohere"