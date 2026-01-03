import pytest
from datetime import datetime
from backend.src.validation.models.validation_models import (
    SearchQuery, SearchResult, ValidationResult, ValidationReport,
    ValidationTest, ValidationConfig, ResultMetadata
)


class TestValidationModels:
    """
    Unit tests for validation models
    """

    def test_search_query_model(self):
        """
        Test SearchQuery model creation and validation
        """
        query = SearchQuery(
            query_text="test query",
            top_k=5,
            min_score=0.5,
            filters={"source_url": "test.com"}
        )

        assert query.query_text == "test query"
        assert query.top_k == 5
        assert query.min_score == 0.5
        assert query.filters == {"source_url": "test.com"}

    def test_search_query_validation(self):
        """
        Test SearchQuery model validation rules
        """
        # Test query text validation
        with pytest.raises(ValueError):
            SearchQuery(query_text="")

        # Test top_k validation
        with pytest.raises(ValueError):
            SearchQuery(query_text="test", top_k=0)

        with pytest.raises(ValueError):
            SearchQuery(query_text="test", top_k=101)

        # Test min_score validation
        with pytest.raises(ValueError):
            SearchQuery(query_text="test", min_score=-0.1)

        with pytest.raises(ValueError):
            SearchQuery(query_text="test", min_score=1.1)

    def test_result_metadata_model(self):
        """
        Test ResultMetadata model
        """
        metadata = ResultMetadata(
            url="https://example.com",
            section="Section 1",
            chunk_index=1,
            source_title="Example Title"
        )

        assert metadata.url == "https://example.com"
        assert metadata.section == "Section 1"
        assert metadata.chunk_index == 1
        assert metadata.source_title == "Example Title"

    def test_search_result_model(self):
        """
        Test SearchResult model creation and validation
        """
        metadata = ResultMetadata(
            url="https://example.com",
            section="Section 1",
            chunk_index=1
        )

        result = SearchResult(
            id="result_1",
            score=0.85,
            content="test content",
            metadata=metadata,
            query_id="query_1"
        )

        assert result.id == "result_1"
        assert result.score == 0.85
        assert result.content == "test content"
        assert result.query_id == "query_1"

    def test_search_result_validation(self):
        """
        Test SearchResult model validation rules
        """
        metadata = ResultMetadata(
            url="https://example.com",
            section="Section 1",
            chunk_index=1
        )

        # Test score validation
        with pytest.raises(ValueError):
            SearchResult(
                id="result_1",
                score=-0.1,
                content="test",
                metadata=metadata,
                query_id="query_1"
            )

        with pytest.raises(ValueError):
            SearchResult(
                id="result_1",
                score=1.1,
                content="test",
                metadata=metadata,
                query_id="query_1"
            )

        # Test content validation
        with pytest.raises(ValueError):
            SearchResult(
                id="result_1",
                score=0.5,
                content="",
                metadata=metadata,
                query_id="query_1"
            )

    def test_validation_result_model(self):
        """
        Test ValidationResult model creation and validation
        """
        query = SearchQuery(query_text="test query")
        metadata = ResultMetadata(url="https://example.com", section="Section 1", chunk_index=1)
        result = SearchResult(
            id="result_1",
            score=0.85,
            content="test content",
            metadata=metadata,
            query_id="query_1"
        )

        validation_result = ValidationResult(
            id="vr_1",
            query=query,
            results=[result],
            execution_time=0.1,
            status="success"
        )

        assert validation_result.id == "vr_1"
        assert validation_result.query.query_text == "test query"
        assert len(validation_result.results) == 1
        assert validation_result.execution_time == 0.1
        assert validation_result.status.value == "success"

    def test_validation_result_validation(self):
        """
        Test ValidationResult model validation rules
        """
        query = SearchQuery(query_text="test query")
        metadata = ResultMetadata(url="https://example.com", section="Section 1", chunk_index=1)
        result = SearchResult(
            id="result_1",
            score=0.85,
            content="test content",
            metadata=metadata,
            query_id="query_1"
        )

        # Test results list validation
        with pytest.raises(ValueError):
            ValidationResult(
                id="vr_1",
                query=query,
                results=[],
                execution_time=0.1,
                status="success"
            )

        # Test execution time validation
        with pytest.raises(ValueError):
            ValidationResult(
                id="vr_1",
                query=query,
                results=[result],
                execution_time=0,
                status="success"
            )

        with pytest.raises(ValueError):
            ValidationResult(
                id="vr_1",
                query=query,
                results=[result],
                execution_time=-0.1,
                status="success"
            )

    def test_validation_report_model(self):
        """
        Test ValidationReport model creation and validation
        """
        query = SearchQuery(query_text="test query")
        metadata = ResultMetadata(url="https://example.com", section="Section 1", chunk_index=1)
        result = SearchResult(
            id="result_1",
            score=0.85,
            content="test content",
            metadata=metadata,
            query_id="query_1"
        )
        validation_result = ValidationResult(
            id="vr_1",
            query=query,
            results=[result],
            execution_time=0.1,
            status="success"
        )

        report = ValidationReport(
            id="report_1",
            test_suite="test_suite_1",
            total_tests=1,
            passed_tests=1,
            failed_tests=0,
            results=[validation_result],
            status="pass"
        )

        assert report.id == "report_1"
        assert report.test_suite == "test_suite_1"
        assert report.total_tests == 1
        assert report.passed_tests == 1
        assert report.failed_tests == 0
        assert report.status.value == "pass"

    def test_validation_report_validation(self):
        """
        Test ValidationReport model validation rules
        """
        query = SearchQuery(query_text="test query")
        metadata = ResultMetadata(url="https://example.com", section="Section 1", chunk_index=1)
        result = SearchResult(
            id="result_1",
            score=0.85,
            content="test content",
            metadata=metadata,
            query_id="query_1"
        )
        validation_result = ValidationResult(
            id="vr_1",
            query=query,
            results=[result],
            execution_time=0.1,
            status="success"
        )

        # Test total_tests validation
        with pytest.raises(ValueError):
            ValidationReport(
                id="report_1",
                test_suite="test_suite_1",
                total_tests=0,
                passed_tests=0,
                failed_tests=0,
                results=[validation_result],
                status="pass"
            )

        # Test test counts validation
        with pytest.raises(ValueError):
            ValidationReport(
                id="report_1",
                test_suite="test_suite_1",
                total_tests=5,
                passed_tests=3,
                failed_tests=3,  # 3 + 3 = 6 != 5
                results=[validation_result],
                status="pass"
            )

    def test_validation_test_model(self):
        """
        Test ValidationTest model creation and validation
        """
        test = ValidationTest(
            name="Test 1",
            description="Test description",
            query="test query",
            category="accuracy"
        )

        assert test.name == "Test 1"
        assert test.description == "Test description"
        assert test.query == "test query"
        assert test.category == "accuracy"
        assert test.enabled is True  # default value

    def test_validation_test_validation(self):
        """
        Test ValidationTest model validation rules
        """
        # Test name validation
        with pytest.raises(ValueError):
            ValidationTest(
                name="",
                description="Test description",
                query="test query",
                category="accuracy"
            )

        # Test description validation
        with pytest.raises(ValueError):
            ValidationTest(
                name="Test 1",
                description="",
                query="test query",
                category="accuracy"
            )

        # Test query validation
        with pytest.raises(ValueError):
            ValidationTest(
                name="Test 1",
                description="Test description",
                query="",
                category="accuracy"
            )

        # Test thresholds validation
        with pytest.raises(ValueError):
            ValidationTest(
                name="Test 1",
                description="Test description",
                query="test query",
                category="accuracy",
                thresholds={"invalid_metric": 0.8}
            )

    def test_validation_config_model(self):
        """
        Test ValidationConfig model creation and validation
        """
        config = ValidationConfig(
            batch_size=20,
            concurrency=10,
            timeout=60.0,
            min_score_threshold=0.5,
            max_execution_time=600.0
        )

        assert config.batch_size == 20
        assert config.concurrency == 10
        assert config.timeout == 60.0
        assert config.min_score_threshold == 0.5
        assert config.max_execution_time == 600.0

    def test_validation_config_validation(self):
        """
        Test ValidationConfig model validation rules
        """
        # Test batch_size validation
        with pytest.raises(ValueError):
            ValidationConfig(batch_size=0)

        with pytest.raises(ValueError):
            ValidationConfig(batch_size=1001)

        # Test concurrency validation
        with pytest.raises(ValueError):
            ValidationConfig(concurrency=0)

        with pytest.raises(ValueError):
            ValidationConfig(concurrency=51)

        # Test timeout validation
        with pytest.raises(ValueError):
            ValidationConfig(timeout=0)

        with pytest.raises(ValueError):
            ValidationConfig(timeout=-1)

        # Test min_score_threshold validation
        with pytest.raises(ValueError):
            ValidationConfig(min_score_threshold=-0.1)

        with pytest.raises(ValueError):
            ValidationConfig(min_score_threshold=1.1)