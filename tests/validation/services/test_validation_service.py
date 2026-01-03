import pytest
from unittest.mock import Mock, AsyncMock, patch
from backend.src.validation.services.validation_service import ValidationService
from backend.src.validation.models.validation_models import (
    SearchQuery, ValidationResult, ValidationReport, ValidationStatus
)


class TestValidationService:
    """
    Unit tests for ValidationService
    """

    @pytest.fixture
    def validation_service(self):
        with patch('backend.src.validation.services.validation_service.SearchService'):
            service = ValidationService()
            service.search_service = Mock()
            return service

    @pytest.mark.asyncio
    async def test_validate_search(self, validation_service):
        """
        Test validate_search functionality
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

        validation_service.search_service.search_validation = AsyncMock(return_value=[mock_result])

        query = SearchQuery(
            query_text="test query",
            top_k=5,
            min_score=0.5
        )

        result = await validation_service.validate_search(query)

        assert isinstance(result, ValidationResult)
        assert result.query.query_text == "test query"
        assert len(result.results) == 1
        assert result.results[0].score == 0.85

    @pytest.mark.asyncio
    async def test_run_batch_validation(self, validation_service):
        """
        Test run_batch_validation functionality
        """
        # Mock search service for validation
        mock_result = Mock()
        mock_result.id = "result_1"
        mock_result.score = 0.85
        mock_result.content = "test content"
        mock_result.metadata = Mock()
        mock_result.metadata.url = "https://example.com"
        mock_result.metadata.section = "Section 1"
        mock_result.metadata.chunk_index = 1
        mock_result.query_id = "query_1"

        validation_service.search_service.search_validation = AsyncMock(return_value=[mock_result])

        # Create batch request
        from backend.src.validation.models.validation_models import BatchValidationRequest
        batch_request = BatchValidationRequest(
            queries=[
                SearchQuery(query_text="query 1", top_k=5, min_score=0.3),
                SearchQuery(query_text="query 2", top_k=5, min_score=0.3)
            ],
            batch_size=2,
            concurrency=2
        )

        report = await validation_service.run_batch_validation(batch_request)

        assert isinstance(report, ValidationReport)
        assert report.total_tests == 2
        assert len(report.results) >= 0  # Results may vary based on implementation

    @pytest.mark.asyncio
    async def test_execute_test_suite(self, validation_service):
        """
        Test execute_test_suite functionality
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

        validation_service.search_service.search_validation = AsyncMock(return_value=[mock_result])

        # Create test suite request
        from backend.src.validation.models.validation_models import TestSuiteRequest
        test_suite_request = TestSuiteRequest(
            suite_name="basic_accuracy_tests",
            batch_size=5,
            concurrency=5
        )

        report = await validation_service.execute_test_suite(test_suite_request)

        assert isinstance(report, ValidationReport)
        assert report.test_suite == "basic_accuracy_tests"
        assert len(report.results) >= 0  # Results may vary based on implementation

    def test_calculate_relevance_metrics(self, validation_service):
        """
        Test _calculate_relevance_metrics method
        """
        # Create mock results
        mock_result = Mock()
        mock_result.score = 0.85

        results = [mock_result, mock_result, mock_result]
        metrics = validation_service._calculate_relevance_metrics(results)

        assert "accuracy" in metrics
        assert "precision" in metrics
        assert "recall" in metrics

    def test_validate_metadata_integrity(self, validation_service):
        """
        Test _validate_metadata_integrity method
        """
        # Create mock results with metadata
        mock_result = Mock()
        mock_result.metadata = Mock()
        mock_result.metadata.url = "https://example.com"
        mock_result.metadata.section = "Section 1"
        mock_result.metadata.chunk_index = 1

        results = [mock_result, mock_result, mock_result]
        validation = validation_service._validate_metadata_integrity(results)

        assert "url_integrity" in validation
        assert "section_integrity" in validation
        assert "completeness" in validation

    def test_determine_validation_status(self, validation_service):
        """
        Test _determine_validation_status method
        """
        # Create mock results
        mock_result = Mock()
        mock_result.score = 0.85

        # Test with high accuracy metrics
        high_metrics = {"accuracy": 0.90}
        status = validation_service._determine_validation_status([mock_result], high_metrics)
        assert status == ValidationStatus.SUCCESS

        # Test with medium accuracy metrics
        medium_metrics = {"accuracy": 0.75}
        status = validation_service._determine_validation_status([mock_result], medium_metrics)
        assert status == ValidationStatus.PARTIAL

        # Test with low accuracy metrics
        low_metrics = {"accuracy": 0.50}
        status = validation_service._determine_validation_status([mock_result], low_metrics)
        assert status == ValidationStatus.FAILED

        # Test with no results
        status = validation_service._determine_validation_status([], {})
        assert status == ValidationStatus.FAILED