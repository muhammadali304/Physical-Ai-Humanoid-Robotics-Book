import pytest
from backend.src.validation.services.metrics_service import MetricsService
from backend.src.validation.models.validation_models import SearchResult, ResultMetadata, ValidationResult


class TestMetricsService:
    """
    Unit tests for MetricsService
    """

    @pytest.fixture
    def metrics_service(self):
        return MetricsService()

    def test_calculate_relevance_metrics(self, metrics_service):
        """
        Test calculate_relevance_metrics method
        """
        # Create mock results
        metadata = ResultMetadata(
            url="https://example.com",
            section="Section 1",
            chunk_index=1
        )
        result1 = SearchResult(
            id="result_1",
            score=0.85,
            content="content 1",
            metadata=metadata,
            query_id="query_1"
        )
        result2 = SearchResult(
            id="result_2",
            score=0.75,
            content="content 2",
            metadata=metadata,
            query_id="query_1"
        )

        results = [result1, result2]
        metrics = metrics_service.calculate_relevance_metrics(results)

        assert "accuracy" in metrics
        assert "precision" in metrics
        assert "recall" in metrics
        assert "mean_score" in metrics
        assert metrics["mean_score"] == 0.8  # (0.85 + 0.75) / 2

    def test_calculate_relevance_metrics_empty(self, metrics_service):
        """
        Test calculate_relevance_metrics with empty results
        """
        metrics = metrics_service.calculate_relevance_metrics([])
        expected = {
            "accuracy": 0.0,
            "precision": 0.0,
            "recall": 0.0,
            "mean_score": 0.0
        }
        assert metrics == expected

    def test_calculate_metadata_validation(self, metrics_service):
        """
        Test calculate_metadata_validation method
        """
        # Create mock results
        metadata1 = ResultMetadata(
            url="https://example.com",
            section="Section 1",
            chunk_index=1
        )
        metadata2 = ResultMetadata(
            url="https://example2.com",
            section="Section 2",
            chunk_index=2
        )
        result1 = SearchResult(
            id="result_1",
            score=0.85,
            content="content 1",
            metadata=metadata1,
            query_id="query_1"
        )
        result2 = SearchResult(
            id="result_2",
            score=0.75,
            content="content 2",
            metadata=metadata2,
            query_id="query_1"
        )

        results = [result1, result2]
        validation = metrics_service.calculate_metadata_validation(results)

        assert "url_integrity" in validation
        assert "section_integrity" in validation
        assert "completeness" in validation
        assert validation["url_integrity"] == 1.0  # Both have URLs
        assert validation["section_integrity"] == 1.0  # Both have sections
        assert validation["completeness"] == 1.0  # Both have chunk indices

    def test_calculate_metadata_validation_empty(self, metrics_service):
        """
        Test calculate_metadata_validation with empty results
        """
        validation = metrics_service.calculate_metadata_validation([])
        expected = {
            "url_integrity": 0.0,
            "section_integrity": 0.0,
            "completeness": 0.0
        }
        assert validation == expected

    def test_calculate_validation_summary_metrics(self, metrics_service):
        """
        Test calculate_validation_summary_metrics method
        """
        # Create mock validation results
        query = {"query_text": "test query", "top_k": 5, "min_score": 0.5}
        result1 = ValidationResult(
            id="vr_1",
            query=query,
            results=[],
            relevance_metrics={"accuracy": 0.9, "precision": 0.85, "recall": 0.8},
            metadata_validation={"url_integrity": 1.0},
            execution_time=0.1,
            status="success"
        )
        result2 = ValidationResult(
            id="vr_2",
            query=query,
            results=[],
            relevance_metrics={"accuracy": 0.85, "precision": 0.8, "recall": 0.75},
            metadata_validation={"url_integrity": 1.0},
            execution_time=0.15,
            status="success"
        )

        results = [result1, result2]
        summary = metrics_service.calculate_validation_summary_metrics(results)

        assert "avg_accuracy" in summary
        assert "avg_precision" in summary
        assert "avg_recall" in summary
        assert "avg_response_time" in summary
        assert "success_rate" in summary

        # Check calculated averages
        assert summary["avg_accuracy"] == 0.875  # (0.9 + 0.85) / 2
        assert summary["avg_precision"] == 0.825  # (0.85 + 0.8) / 2
        assert summary["avg_response_time"] == 0.125  # (0.1 + 0.15) / 2

    def test_calculate_validation_summary_metrics_empty(self, metrics_service):
        """
        Test calculate_validation_summary_metrics with empty results
        """
        summary = metrics_service.calculate_validation_summary_metrics([])
        expected = {
            "avg_accuracy": 0.0,
            "avg_precision": 0.0,
            "avg_recall": 0.0,
            "avg_response_time": 0.0,
            "success_rate": 0.0
        }
        assert summary == expected

    def test_calculate_consistency_metrics(self, metrics_service):
        """
        Test calculate_consistency_metrics method
        """
        # Create mock result sets for repeated queries
        metadata = ResultMetadata(
            url="https://example.com",
            section="Section 1",
            chunk_index=1
        )
        result1 = SearchResult(
            id="result_1",
            score=0.85,
            content="content 1",
            metadata=metadata,
            query_id="query_1"
        )
        result2 = SearchResult(
            id="result_2",
            score=0.80,
            content="content 2",
            metadata=metadata,
            query_id="query_1"
        )

        repeated_results = [
            [result1, result2],
            [result1, result2],  # Same results for consistency
            [result1, result2]
        ]

        consistency = metrics_service.calculate_consistency_metrics("test query", repeated_results)

        assert "consistency_score" in consistency
        assert "variance" in consistency
        assert "top_k_overlap" in consistency

        # With identical results, consistency should be high
        assert consistency["consistency_score"] >= 0.0
        assert consistency["top_k_overlap"] == 1.0  # All results are identical

    def test_calculate_consistency_metrics_single_result(self, metrics_service):
        """
        Test calculate_consistency_metrics with single result set
        """
        metadata = ResultMetadata(
            url="https://example.com",
            section="Section 1",
            chunk_index=1
        )
        result = SearchResult(
            id="result_1",
            score=0.85,
            content="content 1",
            metadata=metadata,
            query_id="query_1"
        )

        repeated_results = [[result]]
        consistency = metrics_service.calculate_consistency_metrics("test query", repeated_results)

        assert consistency["consistency_score"] == 1.0
        assert consistency["variance"] == 0.0
        assert consistency["top_k_overlap"] == 1.0