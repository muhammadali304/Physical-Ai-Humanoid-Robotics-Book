import pytest
from unittest.mock import Mock, patch
from backend.src.validation.services.qdrant_service import QdrantService


class TestQdrantService:
    """
    Unit tests for QdrantService
    """

    @pytest.fixture
    def qdrant_service(self):
        with patch('backend.src.validation.services.qdrant_service.QdrantClient') as mock_client:
            service = QdrantService()
            service.client = mock_client
            return service

    def test_search_basic(self, qdrant_service):
        """
        Test basic search functionality
        """
        # Mock search results
        mock_hit = Mock()
        mock_hit.id = "test_id"
        mock_hit.score = 0.95
        mock_hit.payload = {
            "content": "test content",
            "metadata": {"url": "test.com", "section": "section1", "chunk_index": 1}
        }
        qdrant_service.client.search.return_value = [mock_hit]

        result = qdrant_service.search([0.1, 0.2, 0.3], top_k=5, min_score=0.5)

        assert len(result) == 1
        assert result[0]["id"] == "test_id"
        assert result[0]["score"] == 0.95
        assert result[0]["content"] == "test content"
        assert result[0]["metadata"]["url"] == "test.com"

    def test_search_with_filters(self, qdrant_service):
        """
        Test search with filters
        """
        # Mock search results
        mock_hit = Mock()
        mock_hit.id = "test_id"
        mock_hit.score = 0.85
        mock_hit.payload = {
            "content": "filtered content",
            "metadata": {"url": "filtered.com", "section": "section2", "chunk_index": 2}
        }
        qdrant_service.client.search.return_value = [mock_hit]

        filters = {"source_url": "filtered.com"}
        result = qdrant_service.search([0.1, 0.2, 0.3], top_k=5, min_score=0.5, filters=filters)

        # Verify that the search method was called with appropriate filters
        assert len(result) == 1
        assert result[0]["content"] == "filtered content"

    def test_get_point(self, qdrant_service):
        """
        Test retrieving a specific point
        """
        # Mock retrieve results
        mock_record = Mock()
        mock_record.id = "test_point_id"
        mock_record.payload = {
            "content": "specific content",
            "metadata": {"url": "specific.com", "section": "section3", "chunk_index": 3}
        }
        qdrant_service.client.retrieve.return_value = [mock_record]

        result = qdrant_service.get_point("test_point_id")

        assert result["id"] == "test_point_id"
        assert result["content"] == "specific content"
        assert result["metadata"]["url"] == "specific.com"

    def test_get_point_not_found(self, qdrant_service):
        """
        Test retrieving a point that doesn't exist
        """
        qdrant_service.client.retrieve.return_value = []

        result = qdrant_service.get_point("nonexistent_id")

        assert result is None

    def test_validate_collection_exists(self, qdrant_service):
        """
        Test validating collection exists
        """
        mock_collection_info = Mock()
        qdrant_service.client.get_collection.return_value = mock_collection_info

        result = qdrant_service.validate_collection_exists()

        assert result is True
        qdrant_service.client.get_collection.assert_called_once()

    def test_validate_collection_exists_error(self, qdrant_service):
        """
        Test validating collection when it doesn't exist
        """
        qdrant_service.client.get_collection.side_effect = Exception("Collection not found")

        result = qdrant_service.validate_collection_exists()

        assert result is False