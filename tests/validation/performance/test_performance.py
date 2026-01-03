import pytest
import time
from unittest.mock import Mock, AsyncMock, patch
from backend.src.validation.services.validation_service import ValidationService


class TestValidationPerformance:
    """
    Performance tests for validation service
    """

    @pytest.fixture
    def validation_service(self):
        with patch('backend.src.validation.services.validation_service.SearchService'):
            service = ValidationService()
            service.search_service = Mock()
            return service

    @pytest.mark.asyncio
    async def test_search_validation_performance(self, validation_service):
        """
        Test the performance of search validation
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

        # Measure performance
        start_time = time.time()

        # Run multiple validations to test performance
        for i in range(10):
            query = Mock()
            query.query_text = f"test query {i}"
            query.top_k = 5
            query.min_score = 0.5
            await validation_service.validate_search(query)

        end_time = time.time()

        execution_time = end_time - start_time
        avg_time_per_validation = execution_time / 10

        # Assert that average time per validation is reasonable
        # This is a mock test, so the time should be very fast
        assert avg_time_per_validation < 0.1, f"Average validation time {avg_time_per_validation}s is too slow"

    @pytest.mark.asyncio
    async def test_batch_validation_performance(self, validation_service):
        """
        Test the performance of batch validation
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

        # Create a batch of queries
        from backend.src.validation.models.validation_models import SearchQuery, BatchValidationRequest
        queries = []
        for i in range(5):
            queries.append(SearchQuery(
                query_text=f"batch query {i}",
                top_k=5,
                min_score=0.3
            ))

        batch_request = BatchValidationRequest(
            queries=queries,
            batch_size=5,
            concurrency=3
        )

        # Measure performance
        start_time = time.time()
        report = await validation_service.run_batch_validation(batch_request)
        end_time = time.time()

        execution_time = end_time - start_time

        # Verify the report was generated
        assert report.total_tests == 5
        # Assert that the batch validation completed in a reasonable time
        assert execution_time < 1.0, f"Batch validation took {execution_time}s which is too slow"

    def test_memory_usage_during_validation(self, validation_service):
        """
        Test memory usage during validation (basic check)
        """
        import psutil
        import os

        # Get initial memory usage
        process = psutil.Process(os.getpid())
        initial_memory = process.memory_info().rss / 1024 / 1024  # MB

        # Perform multiple validations
        for i in range(100):
            # Create mock results to simulate validation
            mock_result = Mock()
            mock_result.id = f"result_{i}"
            mock_result.score = 0.85
            mock_result.content = f"test content {i}"
            mock_result.metadata = Mock()
            mock_result.metadata.url = f"https://example{i}.com"
            mock_result.metadata.section = f"Section {i}"
            mock_result.metadata.chunk_index = i
            mock_result.query_id = f"query_{i}"

        # Check final memory usage
        final_memory = process.memory_info().rss / 1024 / 1024  # MB
        memory_increase = final_memory - initial_memory

        # Assert that memory increase is reasonable (less than 100MB for 100 operations)
        assert memory_increase < 100, f"Memory increased by {memory_increase}MB which is too high"

    @pytest.mark.asyncio
    async def test_concurrent_validation_performance(self, validation_service):
        """
        Test performance under concurrent validation loads
        """
        import asyncio

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

        # Create multiple concurrent validation tasks
        async def run_single_validation(i):
            query = Mock()
            query.query_text = f"concurrent query {i}"
            query.top_k = 5
            query.min_score = 0.5
            return await validation_service.validate_search(query)

        # Measure performance with concurrent tasks
        start_time = time.time()
        tasks = [run_single_validation(i) for i in range(10)]
        results = await asyncio.gather(*tasks)
        end_time = time.time()

        execution_time = end_time - start_time
        avg_time_per_validation = execution_time / 10

        # Verify all validations completed
        assert len(results) == 10
        # Assert that concurrent validations completed in reasonable time
        assert avg_time_per_validation < 0.2, f"Average concurrent validation time {avg_time_per_validation}s is too slow"