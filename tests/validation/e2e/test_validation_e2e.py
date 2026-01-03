import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, AsyncMock, patch
from backend.src.validation.main import create_app


class TestValidationE2E:
    """
    End-to-end tests for the validation service
    """

    @pytest.fixture
    def client(self):
        app = create_app()
        with TestClient(app) as test_client:
            yield test_client

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.middleware.auth_middleware')
    @patch('backend.src.validation.services.search_service.SearchService')
    @patch('backend.src.validation.services.validation_service.ValidationService')
    def test_complete_validation_workflow(self, mock_validation_service, mock_search_service, mock_auth, client):
        """
        Test the complete validation workflow from API to services and back
        """
        # Mock authentication to pass
        mock_auth.authenticate = AsyncMock(return_value=True)
        mock_auth.rate_limit = AsyncMock(return_value=True)

        # Mock the validation service to return a complete validation result
        mock_result = Mock()
        mock_result.id = "vr_1"
        mock_result.query = {"query_text": "test query", "top_k": 5, "min_score": 0.5}
        mock_result.results = []
        mock_result.relevance_metrics = {"accuracy": 0.9, "precision": 0.85, "recall": 0.8}
        mock_result.metadata_validation = {"url_integrity": 1.0, "section_integrity": 1.0, "completeness": 1.0}
        mock_result.execution_time = 0.1
        mock_result.status = "success"
        mock_result.created_at = "2023-01-01T00:00:00"
        mock_result.dict = Mock(return_value={
            "id": "vr_1",
            "query": {"query_text": "test query", "top_k": 5, "min_score": 0.5},
            "results": [],
            "relevance_metrics": {"accuracy": 0.9, "precision": 0.85, "recall": 0.8},
            "metadata_validation": {"url_integrity": 1.0, "section_integrity": 1.0, "completeness": 1.0},
            "execution_time": 0.1,
            "status": "success",
            "created_at": "2023-01-01T00:00:00"
        })

        mock_service_instance = Mock()
        mock_service_instance.validate_search = AsyncMock(return_value=mock_result)
        mock_validation_service.return_value = mock_service_instance

        # Test the complete semantic search validation flow
        response = client.post(
            "/validation/search",
            json={
                "query_text": "What are the key features of this system?",
                "top_k": 5,
                "min_score": 0.3,
                "filters": {
                    "source_url": "https://example.com/docs/",
                    "section": "Features"
                }
            },
            headers={"Authorization": "Bearer test_api_key"}
        )

        assert response.status_code == 200
        response_data = response.json()
        assert response_data["id"] == "vr_1"
        assert response_data["query"]["query_text"] == "What are the key features of this system?"
        assert response_data["status"] == "success"
        assert "relevance_metrics" in response_data
        assert "metadata_validation" in response_data

        # Verify that the validation service was called with correct parameters
        mock_service_instance.validate_search.assert_called_once()

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.middleware.auth_middleware')
    @patch('backend.src.validation.services.validation_service.ValidationService')
    def test_batch_validation_e2e(self, mock_validation_service, mock_auth, client):
        """
        Test the complete batch validation workflow
        """
        # Mock authentication to pass
        mock_auth.authenticate = AsyncMock(return_value=True)
        mock_auth.rate_limit = AsyncMock(return_value=True)

        # Mock the validation service to return a complete batch validation report
        mock_report = Mock()
        mock_report.id = "report_1"
        mock_report.test_suite = "batch_validation"
        mock_report.total_tests = 2
        mock_report.passed_tests = 2
        mock_report.failed_tests = 0
        mock_report.results = []
        mock_report.summary_metrics = {"avg_accuracy": 0.88, "avg_precision": 0.85, "avg_recall": 0.82}
        mock_report.status = "pass"
        mock_report.created_at = "2023-01-01T00:00:00"
        mock_report.duration = 0.5
        mock_report.dict = Mock(return_value={
            "id": "report_1",
            "test_suite": "batch_validation",
            "total_tests": 2,
            "passed_tests": 2,
            "failed_tests": 0,
            "results": [],
            "summary_metrics": {"avg_accuracy": 0.88, "avg_precision": 0.85, "avg_recall": 0.82},
            "status": "pass",
            "created_at": "2023-01-01T00:00:00",
            "duration": 0.5
        })

        mock_service_instance = Mock()
        mock_service_instance.run_batch_validation = AsyncMock(return_value=mock_report)
        mock_validation_service.return_value = mock_service_instance

        # Test the complete batch validation workflow
        response = client.post(
            "/validation/batch",
            json={
                "queries": [
                    {
                        "query_text": "What is the main feature?",
                        "top_k": 5,
                        "min_score": 0.3
                    },
                    {
                        "query_text": "How does the system work?",
                        "top_k": 5,
                        "min_score": 0.3
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
        assert "summary_metrics" in response_data

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.middleware.auth_middleware')
    @patch('backend.src.validation.services.validation_service.ValidationService')
    def test_test_suite_execution_e2e(self, mock_validation_service, mock_auth, client):
        """
        Test the complete test suite execution workflow
        """
        # Mock authentication to pass
        mock_auth.authenticate = AsyncMock(return_value=True)
        mock_auth.rate_limit = AsyncMock(return_value=True)

        # Mock the validation service to return a complete test suite report
        mock_report = Mock()
        mock_report.id = "ts_report_1"
        mock_report.test_suite = "comprehensive_tests"
        mock_report.total_tests = 3
        mock_report.passed_tests = 3
        mock_report.failed_tests = 0
        mock_report.results = []
        mock_report.summary_metrics = {"avg_accuracy": 0.91, "avg_precision": 0.88, "avg_recall": 0.85}
        mock_report.status = "pass"
        mock_report.created_at = "2023-01-01T00:00:00"
        mock_report.duration = 1.2
        mock_report.dict = Mock(return_value={
            "id": "ts_report_1",
            "test_suite": "comprehensive_tests",
            "total_tests": 3,
            "passed_tests": 3,
            "failed_tests": 0,
            "results": [],
            "summary_metrics": {"avg_accuracy": 0.91, "avg_precision": 0.88, "avg_recall": 0.85},
            "status": "pass",
            "created_at": "2023-01-01T00:00:00",
            "duration": 1.2
        })

        mock_service_instance = Mock()
        mock_service_instance.execute_test_suite = AsyncMock(return_value=mock_report)
        mock_validation_service.return_value = mock_service_instance

        # Test the complete test suite execution workflow
        response = client.post(
            "/validation/test-suite",
            json={
                "suite_name": "comprehensive_tests",
                "batch_size": 10,
                "concurrency": 5
            },
            headers={"Authorization": "Bearer test_api_key"}
        )

        assert response.status_code == 200
        response_data = response.json()
        assert response_data["test_suite"] == "comprehensive_tests"
        assert response_data["status"] == "pass"
        assert response_data["total_tests"] == 3

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.middleware.auth_middleware')
    @patch('backend.src.validation.services.validation_service.ValidationService')
    def test_report_retrieval_e2e(self, mock_validation_service, mock_auth, client):
        """
        Test the complete report retrieval workflow
        """
        # Mock authentication to pass
        mock_auth.authenticate = AsyncMock(return_value=True)
        mock_auth.rate_limit = AsyncMock(return_value=True)

        # Mock the validation service to return a report
        mock_report = Mock()
        mock_report.id = "existing_report_1"
        mock_report.test_suite = "past_test_suite"
        mock_report.total_tests = 1
        mock_report.passed_tests = 1
        mock_report.failed_tests = 0
        mock_report.results = []
        mock_report.summary_metrics = {"avg_accuracy": 0.89}
        mock_report.status = "pass"
        mock_report.created_at = "2023-01-01T00:00:00"
        mock_report.duration = 0.8
        mock_report.dict = Mock(return_value={
            "id": "existing_report_1",
            "test_suite": "past_test_suite",
            "total_tests": 1,
            "passed_tests": 1,
            "failed_tests": 0,
            "results": [],
            "summary_metrics": {"avg_accuracy": 0.89},
            "status": "pass",
            "created_at": "2023-01-01T00:00:00",
            "duration": 0.8
        })

        mock_service_instance = Mock()
        mock_service_instance.get_report = AsyncMock(return_value=mock_report)
        mock_validation_service.return_value = mock_service_instance

        # Test the complete report retrieval workflow
        response = client.get(
            "/validation/reports/existing_report_1",
            headers={"Authorization": "Bearer test_api_key"}
        )

        assert response.status_code == 200
        response_data = response.json()
        assert response_data["id"] == "existing_report_1"
        assert response_data["status"] == "pass"

    def test_health_check_endpoint(self, client):
        """
        Test the health check endpoint
        """
        response = client.get("/health")
        assert response.status_code == 200
        response_data = response.json()
        assert response_data["status"] == "healthy"
        assert response_data["service"] == "rag-validation"

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.middleware.auth_middleware')
    @patch('backend.src.validation.services.validation_service.ValidationService')
    def test_complete_error_handling_flow(self, mock_validation_service, mock_auth, client):
        """
        Test the complete error handling flow
        """
        # Mock authentication to pass
        mock_auth.authenticate = AsyncMock(return_value=True)
        mock_auth.rate_limit = AsyncMock(return_value=True)

        # Mock validation service to raise an exception
        mock_service_instance = Mock()
        mock_service_instance.validate_search = AsyncMock(side_effect=Exception("External API failure"))
        mock_validation_service.return_value = mock_service_instance

        # Test that errors are properly propagated to the API
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