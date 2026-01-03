import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, AsyncMock, patch
from backend.src.validation.main import create_app
from backend.src.validation.models.validation_models import SearchQuery


class TestValidationRoutes:
    """
    Unit tests for validation API routes
    """

    @pytest.fixture
    def client(self):
        app = create_app()
        with TestClient(app) as test_client:
            yield test_client

    @pytest.fixture
    def mock_validation_service(self):
        with patch('backend.src.validation.api.routes.validation.ValidationService') as mock_service:
            yield mock_service

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.routes.validation.auth_middleware')
    def test_validate_search_endpoint(self, mock_auth, client, mock_validation_service):
        """
        Test the /validation/search endpoint
        """
        # Mock authentication to pass
        mock_auth.authenticate = AsyncMock(return_value=True)
        mock_auth.rate_limit = AsyncMock(return_value=True)

        # Mock validation service
        mock_result = Mock()
        mock_result.dict = Mock(return_value={
            "id": "result_1",
            "query": {"query_text": "test query", "top_k": 5, "min_score": 0.5},
            "results": [],
            "relevance_metrics": {},
            "metadata_validation": {},
            "execution_time": 0.1,
            "status": "success",
            "created_at": "2023-01-01T00:00:00"
        })

        mock_service_instance = Mock()
        mock_service_instance.validate_search = AsyncMock(return_value=mock_result)
        mock_validation_service.return_value = mock_service_instance

        # Test the endpoint
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
        assert "id" in response.json()

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.routes.validation.auth_middleware')
    def test_run_batch_validation_endpoint(self, mock_auth, client, mock_validation_service):
        """
        Test the /validation/batch endpoint
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
            "summary_metrics": {},
            "status": "pass",
            "created_at": "2023-01-01T00:00:00",
            "duration": 0.5
        })

        mock_service_instance = Mock()
        mock_service_instance.run_batch_validation = AsyncMock(return_value=mock_report)
        mock_validation_service.return_value = mock_service_instance

        # Test the endpoint
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
        assert response.json()["id"] == "report_1"

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.routes.validation.auth_middleware')
    def test_execute_test_suite_endpoint(self, mock_auth, client, mock_validation_service):
        """
        Test the /validation/test-suite endpoint
        """
        # Mock authentication to pass
        mock_auth.authenticate = AsyncMock(return_value=True)
        mock_auth.rate_limit = AsyncMock(return_value=True)

        # Mock validation service
        mock_report = Mock()
        mock_report.dict = Mock(return_value={
            "id": "report_1",
            "test_suite": "test_suite_1",
            "total_tests": 1,
            "passed_tests": 1,
            "failed_tests": 0,
            "results": [],
            "summary_metrics": {},
            "status": "pass",
            "created_at": "2023-01-01T00:00:00",
            "duration": 0.5
        })

        mock_service_instance = Mock()
        mock_service_instance.execute_test_suite = AsyncMock(return_value=mock_report)
        mock_validation_service.return_value = mock_service_instance

        # Test the endpoint
        response = client.post(
            "/validation/test-suite",
            json={
                "suite_name": "basic_accuracy_tests",
                "batch_size": 10,
                "concurrency": 5
            },
            headers={"Authorization": "Bearer test_api_key"}
        )

        assert response.status_code == 200
        assert response.json()["test_suite"] == "test_suite_1"

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.routes.validation.auth_middleware')
    def test_list_validation_reports_endpoint(self, mock_auth, client, mock_validation_service):
        """
        Test the /validation/reports endpoint
        """
        # Mock authentication to pass
        mock_auth.authenticate = AsyncMock(return_value=True)
        mock_auth.rate_limit = AsyncMock(return_value=True)

        # Mock validation service
        mock_service_instance = Mock()
        mock_service_instance.list_reports = AsyncMock(return_value=[])
        mock_validation_service.return_value = mock_service_instance

        # Test the endpoint
        response = client.get(
            "/validation/reports",
            headers={"Authorization": "Bearer test_api_key"}
        )

        assert response.status_code == 200
        assert "reports" in response.json()

    @pytest.mark.asyncio
    @patch('backend.src.validation.api.routes.validation.auth_middleware')
    def test_get_validation_report_endpoint(self, mock_auth, client, mock_validation_service):
        """
        Test the /validation/reports/{report_id} endpoint
        """
        # Mock authentication to pass
        mock_auth.authenticate = AsyncMock(return_value=True)
        mock_auth.rate_limit = AsyncMock(return_value=True)

        # Mock validation service
        mock_report = Mock()
        mock_report.dict = Mock(return_value={
            "id": "report_1",
            "test_suite": "test_suite_1",
            "total_tests": 1,
            "passed_tests": 1,
            "failed_tests": 0,
            "results": [],
            "summary_metrics": {},
            "status": "pass",
            "created_at": "2023-01-01T00:00:00",
            "duration": 0.5
        })

        mock_service_instance = Mock()
        mock_service_instance.get_report = AsyncMock(return_value=mock_report)
        mock_validation_service.return_value = mock_service_instance

        # Test the endpoint
        response = client.get(
            "/validation/reports/report_1",
            headers={"Authorization": "Bearer test_api_key"}
        )

        assert response.status_code == 200
        assert response.json()["id"] == "report_1"

    def test_health_check_endpoint(self, client):
        """
        Test the /health endpoint
        """
        response = client.get("/health")
        assert response.status_code == 200
        assert response.json() == {"status": "healthy", "service": "rag-validation"}