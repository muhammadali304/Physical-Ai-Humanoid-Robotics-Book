import pytest
import asyncio
import time
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import Mock, AsyncMock, patch
from backend.src.validation.main import create_app
from fastapi.testclient import TestClient


class TestValidationLoad:
    """
    Load tests for validation API
    """

    @pytest.fixture
    def client(self):
        app = create_app()
        with TestClient(app) as test_client:
            yield test_client

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.middleware.auth_middleware')
    @patch('backend.src.validation.services.validation_service.ValidationService')
    def test_high_volume_single_endpoint_load(self, mock_validation_service, mock_auth, client):
        """
        Test the system's ability to handle high volume requests to a single endpoint
        """
        # Mock authentication to pass
        mock_auth.authenticate = AsyncMock(return_value=True)
        mock_auth.rate_limit = AsyncMock(return_value=True)

        # Mock validation service
        mock_result = Mock()
        mock_result.dict = Mock(return_value={
            "id": "vr_1",
            "query": {"query_text": "test query", "top_k": 5, "min_score": 0.5},
            "results": [],
            "relevance_metrics": {"accuracy": 0.9, "precision": 0.85, "recall": 0.8},
            "metadata_validation": {"url_integrity": 1.0, "section_integrity": 1.0},
            "execution_time": 0.1,
            "status": "success",
            "created_at": "2023-01-01T00:00:00"
        })

        mock_service_instance = Mock()
        mock_service_instance.validate_search = AsyncMock(return_value=mock_result)
        mock_validation_service.return_value = mock_service_instance

        # Test high volume requests to /validation/search
        request_count = 50
        start_time = time.time()

        responses = []
        for i in range(request_count):
            response = client.post(
                "/validation/search",
                json={
                    "query_text": f"load test query {i}",
                    "top_k": 5,
                    "min_score": 0.3
                },
                headers={"Authorization": "Bearer test_api_key"}
            )
            responses.append(response)

        end_time = time.time()
        total_time = end_time - start_time
        avg_response_time = total_time / request_count

        # Verify all requests succeeded
        success_count = sum(1 for r in responses if r.status_code == 200)
        assert success_count == request_count, f"Expected {request_count} successful responses, got {success_count}"

        # Verify response time is reasonable
        assert avg_response_time < 0.5, f"Average response time {avg_response_time}s is too high"

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.middleware.auth_middleware')
    @patch('backend.src.validation.services.validation_service.ValidationService')
    def test_concurrent_load_on_multiple_endpoints(self, mock_validation_service, mock_auth, client):
        """
        Test concurrent load on multiple API endpoints
        """
        # Mock authentication to pass
        mock_auth.authenticate = AsyncMock(return_value=True)
        mock_auth.rate_limit = AsyncMock(return_value=True)

        # Mock validation service for different endpoints
        mock_search_result = Mock()
        mock_search_result.dict = Mock(return_value={
            "id": "vr_search",
            "query": {"query_text": "search query", "top_k": 5, "min_score": 0.5},
            "results": [],
            "execution_time": 0.1,
            "status": "success",
            "created_at": "2023-01-01T00:00:00"
        })

        mock_batch_result = Mock()
        mock_batch_result.dict = Mock(return_value={
            "id": "report_batch",
            "test_suite": "batch_test",
            "total_tests": 2,
            "passed_tests": 2,
            "failed_tests": 0,
            "results": [],
            "summary_metrics": {"avg_accuracy": 0.9},
            "status": "pass",
            "duration": 0.5
        })

        mock_service_instance = Mock()
        mock_service_instance.validate_search = AsyncMock(return_value=mock_search_result)
        mock_service_instance.run_batch_validation = AsyncMock(return_value=mock_batch_result)
        mock_validation_service.return_value = mock_service_instance

        # Function to simulate requests to different endpoints
        def make_search_request(i):
            return client.post(
                "/validation/search",
                json={
                    "query_text": f"concurrent search {i}",
                    "top_k": 5,
                    "min_score": 0.3
                },
                headers={"Authorization": "Bearer test_api_key"}
            )

        def make_batch_request(i):
            return client.post(
                "/validation/batch",
                json={
                    "queries": [
                        {
                            "query_text": f"batch query {i}",
                            "top_k": 5,
                            "min_score": 0.3
                        }
                    ],
                    "batch_size": 10,
                    "concurrency": 5
                },
                headers={"Authorization": "Bearer test_api_key"}
            )

        # Run concurrent requests
        start_time = time.time()

        with ThreadPoolExecutor(max_workers=10) as executor:
            search_futures = [executor.submit(make_search_request, i) for i in range(20)]
            batch_futures = [executor.submit(make_batch_request, i) for i in range(10)]

            search_results = [f.result() for f in search_futures]
            batch_results = [f.result() for f in batch_futures]

        end_time = time.time()
        total_time = end_time - start_time

        # Verify responses
        search_successes = sum(1 for r in search_results if r.status_code == 200)
        batch_successes = sum(1 for r in batch_results if r.status_code == 200)

        assert search_successes == 20, f"Expected 20 search successes, got {search_successes}"
        assert batch_successes == 10, f"Expected 10 batch successes, got {batch_successes}"

        # Verify total response time is reasonable
        assert total_time < 5.0, f"Total time {total_time}s is too high for 30 requests"

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.middleware.auth_middleware')
    @patch('backend.src.validation.services.validation_service.ValidationService')
    def test_batch_validation_load(self, mock_validation_service, mock_auth, client):
        """
        Test load on batch validation endpoint with large batches
        """
        # Mock authentication to pass
        mock_auth.authenticate = AsyncMock(return_value=True)
        mock_auth.rate_limit = AsyncMock(return_value=True)

        # Mock validation service
        mock_report = Mock()
        mock_report.dict = Mock(return_value={
            "id": "large_batch_report",
            "test_suite": "large_batch",
            "total_tests": 100,
            "passed_tests": 95,
            "failed_tests": 5,
            "results": [],
            "summary_metrics": {"avg_accuracy": 0.88},
            "status": "warning",
            "duration": 1.0
        })

        mock_service_instance = Mock()
        mock_service_instance.run_batch_validation = AsyncMock(return_value=mock_report)
        mock_validation_service.return_value = mock_service_instance

        # Create a large batch request
        large_batch_queries = []
        for i in range(100):
            large_batch_queries.append({
                "query_text": f"large batch query {i}",
                "top_k": 5,
                "min_score": 0.3
            })

        start_time = time.time()
        response = client.post(
            "/validation/batch",
            json={
                "queries": large_batch_queries,
                "batch_size": 50,
                "concurrency": 10
            },
            headers={"Authorization": "Bearer test_api_key"}
        )
        end_time = time.time()

        assert response.status_code == 200
        assert response.json()["total_tests"] == 100
        assert end_time - start_time < 10.0  # Should complete in under 10 seconds

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.middleware.auth_middleware')
    @patch('backend.src.validation.services.validation_service.ValidationService')
    def test_multiple_concurrent_users(self, mock_validation_service, mock_auth, client):
        """
        Test the system with multiple simulated concurrent users
        """
        # Mock authentication to pass
        mock_auth.authenticate = AsyncMock(return_value=True)
        mock_auth.rate_limit = AsyncMock(return_value=True)

        # Mock validation service
        mock_result = Mock()
        mock_result.dict = Mock(return_value={
            "id": "multi_user_result",
            "query": {"query_text": "multi-user query", "top_k": 5, "min_score": 0.5},
            "results": [],
            "execution_time": 0.1,
            "status": "success",
            "created_at": "2023-01-01T00:00:00"
        })

        mock_service_instance = Mock()
        mock_service_instance.validate_search = AsyncMock(return_value=mock_result)
        mock_validation_service.return_value = mock_service_instance

        # Simulate multiple users making requests concurrently
        async def simulate_user_requests(user_id, num_requests):
            responses = []
            for i in range(num_requests):
                response = client.post(
                    "/validation/search",
                    json={
                        "query_text": f"user {user_id} query {i}",
                        "top_k": 5,
                        "min_score": 0.3
                    },
                    headers={"Authorization": f"Bearer user_{user_id}_token"}
                )
                responses.append(response.status_code)
            return responses

        # Run multiple users concurrently
        start_time = time.time()

        # Simulate 5 users each making 10 requests
        user_tasks = [simulate_user_requests(user_id, 10) for user_id in range(5)]
        all_responses = await asyncio.gather(*user_tasks)

        end_time = time.time()
        total_time = end_time - start_time

        # Flatten responses
        all_status_codes = [status for user_responses in all_responses for status in user_responses]

        # Verify results
        total_requests = 5 * 10  # 5 users * 10 requests
        success_count = sum(1 for status in all_status_codes if status == 200)

        assert len(all_status_codes) == total_requests
        assert success_count == total_requests, f"Expected {total_requests} successes, got {success_count}"
        assert total_time < 10.0, f"Total time {total_time}s is too high for {total_requests} requests"

    def test_health_endpoint_under_load(self, client):
        """
        Test that health endpoint remains responsive under load
        """
        # Make multiple health check requests
        start_time = time.time()
        for i in range(100):
            response = client.get("/health")
            assert response.status_code == 200
            assert response.json()["status"] == "healthy"
        end_time = time.time()

        total_time = end_time - start_time
        avg_time = total_time / 100

        # Health endpoint should be very fast
        assert avg_time < 0.05, f"Average health check time {avg_time}s is too slow"