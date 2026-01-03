import pytest
import asyncio
from unittest.mock import Mock, patch, AsyncMock
from backend.src.validation.services.embedding_service import EmbeddingService


class TestEmbeddingService:
    """
    Unit tests for EmbeddingService
    """

    @pytest.fixture
    def embedding_service(self):
        with patch('backend.src.validation.services.embedding_service.cohere.Client') as mock_client:
            service = EmbeddingService()
            service.client = mock_client
            return service

    @pytest.mark.asyncio
    async def test_generate_embedding(self, embedding_service):
        """
        Test generating embedding for a single text
        """
        # Mock the Cohere client response
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3, 0.4]]
        embedding_service.client.embed.return_value = mock_response

        result = await embedding_service.generate_embedding("test query")

        assert result == [0.1, 0.2, 0.3, 0.4]
        embedding_service.client.embed.assert_called_once()

    @pytest.mark.asyncio
    async def test_generate_embeddings_multiple_texts(self, embedding_service):
        """
        Test generating embeddings for multiple texts
        """
        # Mock the Cohere client response
        mock_response = Mock()
        mock_response.embeddings = [
            [0.1, 0.2, 0.3, 0.4],
            [0.5, 0.6, 0.7, 0.8]
        ]
        embedding_service.client.embed.return_value = mock_response

        texts = ["test query 1", "test query 2"]
        result = await embedding_service.generate_embeddings(texts)

        assert len(result) == 2
        assert result[0] == [0.1, 0.2, 0.3, 0.4]
        assert result[1] == [0.5, 0.6, 0.7, 0.8]
        embedding_service.client.embed.assert_called_once()

    @pytest.mark.asyncio
    async def test_generate_embeddings_with_caching(self, embedding_service):
        """
        Test embedding caching functionality
        """
        # Mock the Cohere client response
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3, 0.4]]
        embedding_service.client.embed.return_value = mock_response

        # Generate embedding for the first time
        result1 = await embedding_service.generate_embedding("test query")
        assert embedding_service.client.embed.call_count == 1

        # Generate embedding for the same query (should use cache)
        result2 = await embedding_service.generate_embedding("test query")
        assert embedding_service.client.embed.call_count == 1  # Still 1, used cache

        assert result1 == result2

    @pytest.mark.asyncio
    async def test_clear_cache(self, embedding_service):
        """
        Test clearing the embedding cache
        """
        # Add something to cache
        embedding_service._cache["test:mock"] = [0.1, 0.2, 0.3, 0.4]
        assert len(embedding_service._cache) == 1

        embedding_service.clear_cache()
        assert len(embedding_service._cache) == 0

    @pytest.mark.asyncio
    async def test_generate_embedding_error_handling(self, embedding_service):
        """
        Test error handling when Cohere API fails
        """
        embedding_service.client.embed.side_effect = Exception("API Error")

        with pytest.raises(Exception, match="API Error"):
            await embedding_service.generate_embedding("test query")