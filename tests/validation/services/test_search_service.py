import pytest
from unittest.mock import Mock, patch, AsyncMock
from backend.src.validation.services.search_service import SearchService


class TestSearchService:
    """
    Unit tests for SearchService
    """

    @pytest.fixture
    def search_service(self):
        with patch('backend.src.validation.services.search_service.EmbeddingService'), \
             patch('backend.src.validation.services.search_service.QdrantService'):
            service = SearchService()
            service.embedding_service = Mock()
            service.qdrant_service = Mock()
            return service

    @pytest.mark.asyncio
    async def test_search_validation(self, search_service):
        """
        Test basic search validation functionality
        """
        # Mock embedding service
        search_service.embedding_service.generate_embedding = AsyncMock(return_value=[0.1, 0.2, 0.3])

        # Mock qdrant service
        mock_result = {
            "id": "test_id",
            "score": 0.95,
            "content": "test content",
            "metadata": Mock(),
            "query_id": "test_query_id"
        }
        search_service.qdrant_service.search = AsyncMock(return_value=[mock_result])

        results = await search_service.search_validation(
            query_text="test query",
            top_k=5,
            min_score=0.5
        )

        assert len(results) == 1
        assert results[0].id == "test_id"
        assert results[0].score == 0.95
        assert results[0].content == "test content"

    @pytest.mark.asyncio
    async def test_validate_url_filtering(self, search_service):
        """
        Test URL filtering validation
        """
        # Mock embedding service
        search_service.embedding_service.generate_embedding = AsyncMock(return_value=[0.1, 0.2, 0.3])

        # Mock qdrant service
        mock_result = {
            "id": "test_id",
            "score": 0.85,
            "content": "filtered content",
            "metadata": Mock(),
            "query_id": "test_query_id"
        }
        search_service.qdrant_service.search = AsyncMock(return_value=[mock_result])

        results = await search_service.validate_url_filtering(
            query_text="test query",
            source_url="test.com",
            top_k=5
        )

        assert len(results) == 1
        # Verify that search was called with filters
        search_service.qdrant_service.search.assert_called_once()
        args, kwargs = search_service.qdrant_service.search.call_args
        assert kwargs["filters"] == {"source_url": "test.com"}

    @pytest.mark.asyncio
    async def test_validate_section_filtering(self, search_service):
        """
        Test section filtering validation
        """
        # Mock embedding service
        search_service.embedding_service.generate_embedding = AsyncMock(return_value=[0.1, 0.2, 0.3])

        # Mock qdrant service
        mock_result = {
            "id": "test_id",
            "score": 0.75,
            "content": "section content",
            "metadata": Mock(),
            "query_id": "test_query_id"
        }
        search_service.qdrant_service.search = AsyncMock(return_value=[mock_result])

        results = await search_service.validate_section_filtering(
            query_text="test query",
            section="section1",
            top_k=5
        )

        assert len(results) == 1
        # Verify that search was called with filters
        search_service.qdrant_service.search.assert_called_once()
        args, kwargs = search_service.qdrant_service.search.call_args
        assert kwargs["filters"] == {"section": "section1"}