import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, AsyncMock, patch
from backend.src.validation.main import create_app


class TestAPIIntegration:
    """
    Integration tests for validation API
    """

    @pytest.fixture
    def client(self):
        app = create_app()
        with TestClient(app) as test_client:
            yield test_client

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.middleware.auth_middleware')
    @patch('backend.src.validation.services.validation_service.ValidationService')
    def test_full_search_validation_flow(self, mock_validation_service, mock_auth, client):
        """
        Test the complete flow from API request to service response
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

        # Test the search validation endpoint
        response = client.post(
            "/validation/search",
            json={
                "query_text": "test query",
                "top_k": 5,
                "min_score": 0.5
            },
            headers={"Authorization": "Bearer test_api_key"}
        )

        assert response.status_code == 200
        response_data = response.json()
        assert response_data["id"] == "vr_1"
        assert response_data["query"]["query_text"] == "test query"
        assert response_data["status"] == "success"

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.middleware.auth_middleware')
    @patch('backend.src.validation.services.validation_service.ValidationService')
    def test_full_batch_validation_flow(self, mock_validation_service, mock_auth, client):
        """
        Test the complete batch validation flow
        """
        # Mock authentication to pass
        mock_auth.authenticate = AsyncMock(return_value=True)
        mock_auth.rate_limit = AsyncMock(return_value=True)

        # Mock validation service
        mock_report = Mock()
        mock_report.dict = Mock(return_value={
            "id": "report_1",
            "test_suite": "batch_validation",
            "total_tests": 2,
            "passed_tests": 2,
            "failed_tests": 0,
            "results": [],
            "summary_metrics": {"avg_accuracy": 0.88, "avg_precision": 0.85},
            "status": "pass",
            "created_at": "2023-01-01T00:00:00",
            "duration": 0.5
        })

        mock_service_instance = Mock()
        mock_service_instance.run_batch_validation = AsyncMock(return_value=mock_report)
        mock_validation_service.return_value = mock_service_instance

        # Test the batch validation endpoint
        response = client.post(
            "/validation/batch",
            json={
                "queries": [
                    {
                        "query_text": "test query 1",
                        "top_k": 5,
                        "min_score": 0.5
                    },
                    {
                        "query_text": "test query 2",
                        "top_k": 5,
                        "min_score": 0.5
                    }
                ],
                "batch_size": 10,
                "concurrency": 5
            },
            headers={"Authorization": "Bearer test_api_key"}
        )

        assert response.status_code == 200
        response_data = response.json()
        assert response_data["id"] == "report_1"
        assert response_data["total_tests"] == 2
        assert response_data["status"] == "pass"

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.middleware.auth_middleware')
    @patch('backend.src.validation.services.validation_service.ValidationService')
    def test_full_test_suite_flow(self, mock_validation_service, mock_auth, client):
        """
        Test the complete test suite execution flow
        """
        # Mock authentication to pass
        mock_auth.authenticate = AsyncMock(return_value=True)
        mock_auth.rate_limit = AsyncMock(return_value=True)

        # Mock validation service
        mock_report = Mock()
        mock_report.dict = Mock(return_value={
            "id": "report_1",
            "test_suite": "accuracy_tests",
            "total_tests": 1,
            "passed_tests": 1,
            "failed_tests": 0,
            "results": [],
            "summary_metrics": {"avg_accuracy": 0.9, "avg_precision": 0.85},
            "status": "pass",
            "created_at": "2023-01-01T00:00:00",
            "duration": 0.2
        })

        mock_service_instance = Mock()
        mock_service_instance.execute_test_suite = AsyncMock(return_value=mock_report)
        mock_validation_service.return_value = mock_service_instance

        # Test the test suite endpoint
        response = client.post(
            "/validation/test-suite",
            json={
                "suite_name": "accuracy_tests",
                "batch_size": 10,
                "concurrency": 5
            },
            headers={"Authorization": "Bearer test_api_key"}
        )

        assert response.status_code == 200
        response_data = response.json()
        assert response_data["test_suite"] == "accuracy_tests"
        assert response_data["status"] == "pass"

    def test_health_endpoint_integration(self, client):
        """
        Test the health endpoint integration
        """
        response = client.get("/health")
        assert response.status_code == 200
        response_data = response.json()
        assert response_data["status"] == "healthy"
        assert response_data["service"] == "rag-validation"

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.middleware.auth_middleware')
    @patch('backend.src.validation.services.validation_service.ValidationService')
    def test_error_handling_in_api(self, mock_validation_service, mock_auth, client):
        """
        Test error handling in the API
        """
        # Mock authentication to pass
        mock_auth.authenticate = AsyncMock(return_value=True)
        mock_auth.rate_limit = AsyncMock(return_value=True)

        # Mock validation service to raise an exception
        mock_service_instance = Mock()
        mock_service_instance.validate_search = AsyncMock(side_effect=Exception("API Error"))
        mock_validation_service.return_value = mock_service_instance

        # Test the search validation endpoint with error
        response = client.post(
            "/validation/search",
            json={
                "query_text": "test query",
                "top_k": 5,
                "min_score": 0.5
            },
            headers={"Authorization": "Bearer test_api_key"}
        )

        assert response.status_code == 500
        assert "Search validation failed" in response.json()["detail"]