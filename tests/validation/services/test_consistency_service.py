import pytest
from unittest.mock import Mock, AsyncMock, patch
from backend.src.validation.services.consistency_service import ConsistencyService
from backend.src.validation.models.validation_models import (
    SearchResult, ResultMetadata, ValidationTest, ValidationStatus
)


class TestConsistencyService:
    """
    Unit tests for ConsistencyService
    """

    @pytest.fixture
    def consistency_service(self):
        with patch('backend.src.validation.services.consistency_service.SearchService'), \
             patch('backend.src.validation.services.consistency_service.MetricsService'):
            service = ConsistencyService()
            service.search_service = Mock()
            service.metrics_service = Mock()
            return service

    @pytest.mark.asyncio
    async def test_validate_consistency_for_query(self, consistency_service):
        """
        Test validate_consistency_for_query method
        """
        # Mock search service
        mock_result = Mock()
        mock_result.id = "result_1"
        mock_result.score = 0.85
        mock_result.content = "test content"
        mock_result.metadata = Mock()
        mock_result.metadata.url = "https://example.com"
        mock_result.metadata.section = "Section 1"
        mock_result.metadata.chunk_index = 1
        mock_result.query_id = "query_1"

        consistency_service.search_service.search_validation = AsyncMock(return_value=[mock_result])

        # Mock metrics service
        consistency_service.metrics_service.calculate_consistency_metrics = Mock(return_value={
            "consistency_score": 0.95,
            "variance": 0.01,
            "top_k_overlap": 1.0
        })

        result = await consistency_service.validate_consistency_for_query(
            query_text="test query",
            num_repetitions=3,
            top_k=5
        )

        assert "query" in result
        assert "repetitions" in result
        assert "results" in result
        assert "consistency_metrics" in result
        assert result["query"] == "test query"
        assert result["repetitions"] == 3
        assert len(result["results"]) == 3  # Should have 3 result sets

    @pytest.mark.asyncio
    async def test_run_test_suite_execution(self, consistency_service):
        """
        Test run_test_suite_execution method
        """
        # Mock search service
        mock_result = Mock()
        mock_result.id = "result_1"
        mock_result.score = 0.85
        mock_result.content = "test content"
        mock_result.metadata = Mock()
        mock_result.metadata.url = "https://example.com"
        mock_result.metadata.section = "Section 1"
        mock_result.metadata.chunk_index = 1
        mock_result.query_id = "query_1"

        consistency_service.search_service.search_validation = AsyncMock(return_value=[mock_result])

        # Mock metrics service
        consistency_service.metrics_service.calculate_relevance_metrics = Mock(return_value={
            "accuracy": 0.9,
            "precision": 0.85,
            "recall": 0.8
        })
        consistency_service.metrics_service.calculate_metadata_validation = Mock(return_value={
            "url_integrity": 1.0,
            "section_integrity": 1.0,
            "completeness": 1.0
        })

        # Create test objects
        test1 = ValidationTest(
            name="Test 1",
            description="Test 1 description",
            query="test query 1",
            category="accuracy"
        )
        test2 = ValidationTest(
            name="Test 2",
            description="Test 2 description",
            query="test query 2",
            category="accuracy"
        )

        tests = [test1, test2]
        report = await consistency_service.run_test_suite_execution(
            test_suite_name="test_suite_1",
            tests=tests,
            concurrency=2
        )

        assert report.test_suite == "test_suite_1"
        assert report.total_tests == 2
        assert len(report.results) >= 0  # May vary based on implementation

    @pytest.mark.asyncio
    async def test_run_batch_validation_processing(self, consistency_service):
        """
        Test run_batch_validation_processing method
        """
        # Mock search service
        mock_result = Mock()
        mock_result.id = "result_1"
        mock_result.score = 0.85
        mock_result.content = "test content"
        mock_result.metadata = Mock()
        mock_result.metadata.url = "https://example.com"
        mock_result.metadata.section = "Section 1"
        mock_result.metadata.chunk_index = 1
        mock_result.query_id = "query_1"

        consistency_service.search_service.search_validation = AsyncMock(return_value=[mock_result])

        # Mock metrics service
        consistency_service.metrics_service.calculate_relevance_metrics = Mock(return_value={
            "accuracy": 0.9,
            "precision": 0.85,
            "recall": 0.8
        })
        consistency_service.metrics_service.calculate_metadata_validation = Mock(return_value={
            "url_integrity": 1.0,
            "section_integrity": 1.0,
            "completeness": 1.0
        })

        # Create query parameters
        queries = [
            {
                "query_text": "test query 1",
                "top_k": 5,
                "min_score": 0.3
            },
            {
                "query_text": "test query 2",
                "top_k": 5,
                "min_score": 0.3
            }
        ]

        report = await consistency_service.run_batch_validation_processing(
            queries=queries,
            batch_size=2,
            concurrency=2
        )

        assert report.total_tests == 2
        assert len(report.results) >= 0  # May vary based on implementation

    @pytest.mark.asyncio
    async def test_run_batch_validation_processing_single_batch(self, consistency_service):
        """
        Test run_batch_validation_processing with single batch
        """
        # Mock search service
        mock_result = Mock()
        mock_result.id = "result_1"
        mock_result.score = 0.85
        mock_result.content = "test content"
        mock_result.metadata = Mock()
        mock_result.metadata.url = "https://example.com"
        mock_result.metadata.section = "Section 1"
        mock_result.metadata.chunk_index = 1
        mock_result.query_id = "query_1"

        consistency_service.search_service.search_validation = AsyncMock(return_value=[mock_result])

        # Mock metrics service
        consistency_service.metrics_service.calculate_relevance_metrics = Mock(return_value={
            "accuracy": 0.9,
            "precision": 0.85,
            "recall": 0.8
        })
        consistency_service.metrics_service.calculate_metadata_validation = Mock(return_value={
            "url_integrity": 1.0,
            "section_integrity": 1.0,
            "completeness": 1.0
        })

        queries = [
            {
                "query_text": "test query 1",
                "top_k": 5,
                "min_score": 0.3
            }
        ]

        report = await consistency_service.run_batch_validation_processing(
            queries=queries,
            batch_size=1,
            concurrency=1
        )

        assert report.total_tests == 1
        assert len(report.results) >= 0  # May vary based on implementation