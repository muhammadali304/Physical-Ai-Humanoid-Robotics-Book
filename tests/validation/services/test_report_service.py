import pytest
import json
from datetime import datetime
from backend.src.validation.services.report_service import ReportService
from backend.src.validation.models.validation_models import (
    ValidationReport, ValidationResult, SearchQuery, ValidationStatus
)


class TestReportService:
    """
    Unit tests for ReportService
    """

    @pytest.fixture
    def report_service(self):
        return ReportService()

    def test_generate_json_report(self, report_service):
        """
        Test generate_json_report method
        """
        query = SearchQuery(query_text="test query")
        result = ValidationResult(
            id="vr_1",
            query=query,
            results=[],
            relevance_metrics={"accuracy": 0.9, "precision": 0.85},
            metadata_validation={"url_integrity": 1.0},
            execution_time=0.1,
            status=ValidationStatus.SUCCESS
        )
        report = ValidationReport(
            id="report_1",
            test_suite="test_suite_1",
            total_tests=1,
            passed_tests=1,
            failed_tests=0,
            results=[result],
            summary_metrics={"avg_accuracy": 0.9},
            status="pass"
        )

        json_report = report_service.generate_json_report(report)
        parsed = json.loads(json_report)

        assert parsed["id"] == "report_1"
        assert parsed["test_suite"] == "test_suite_1"
        assert parsed["total_tests"] == 1

    def test_generate_csv_report(self, report_service):
        """
        Test generate_csv_report method
        """
        query = SearchQuery(query_text="test query")
        result = ValidationResult(
            id="vr_1",
            query=query,
            results=[],
            relevance_metrics={"accuracy": 0.9, "precision": 0.85},
            metadata_validation={"url_integrity": 1.0},
            execution_time=0.1,
            status=ValidationStatus.SUCCESS
        )
        report = ValidationReport(
            id="report_1",
            test_suite="test_suite_1",
            total_tests=1,
            passed_tests=1,
            failed_tests=0,
            results=[result],
            summary_metrics={"avg_accuracy": 0.9},
            status="pass"
        )

        csv_report = report_service.generate_csv_report(report)

        # Check that CSV contains expected headers
        assert "Report ID" in csv_report
        assert "Test Suite" in csv_report
        assert "Total Tests" in csv_report
        assert "report_1" in csv_report
        assert "test_suite_1" in csv_report

    def test_store_and_get_report(self, report_service):
        """
        Test store_report and get_report methods
        """
        query = SearchQuery(query_text="test query")
        result = ValidationResult(
            id="vr_1",
            query=query,
            results=[],
            relevance_metrics={"accuracy": 0.9},
            metadata_validation={"url_integrity": 1.0},
            execution_time=0.1,
            status=ValidationStatus.SUCCESS
        )
        report = ValidationReport(
            id="report_1",
            test_suite="test_suite_1",
            total_tests=1,
            passed_tests=1,
            failed_tests=0,
            results=[result],
            summary_metrics={"avg_accuracy": 0.9},
            status="pass"
        )

        # Store the report
        stored_id = report_service.store_report(report)
        assert stored_id == "report_1"

        # Retrieve the report
        retrieved_report = report_service.get_report("report_1")
        assert retrieved_report is not None
        assert retrieved_report.id == "report_1"
        assert retrieved_report.test_suite == "test_suite_1"

    def test_get_report_not_found(self, report_service):
        """
        Test get_report with non-existent report ID
        """
        report = report_service.get_report("nonexistent_id")
        assert report is None

    def test_list_reports(self, report_service):
        """
        Test list_reports method
        """
        # Create and store some reports
        query = SearchQuery(query_text="test query")
        result = ValidationResult(
            id="vr_1",
            query=query,
            results=[],
            relevance_metrics={"accuracy": 0.9},
            metadata_validation={"url_integrity": 1.0},
            execution_time=0.1,
            status=ValidationStatus.SUCCESS
        )
        report1 = ValidationReport(
            id="report_1",
            test_suite="test_suite_1",
            total_tests=1,
            passed_tests=1,
            failed_tests=0,
            results=[result],
            summary_metrics={"avg_accuracy": 0.9},
            status="pass"
        )
        report2 = ValidationReport(
            id="report_2",
            test_suite="test_suite_2",
            total_tests=1,
            passed_tests=1,
            failed_tests=0,
            results=[result],
            summary_metrics={"avg_accuracy": 0.85},
            status="pass"
        )

        report_service.store_report(report1)
        report_service.store_report(report2)

        # List reports with pagination
        reports = report_service.list_reports(limit=10, offset=0)
        assert len(reports) == 2

        reports = report_service.list_reports(limit=1, offset=0)
        assert len(reports) == 1
        assert reports[0].id == "report_1"

    def test_export_report(self, report_service):
        """
        Test export_report method
        """
        query = SearchQuery(query_text="test query")
        result = ValidationResult(
            id="vr_1",
            query=query,
            results=[],
            relevance_metrics={"accuracy": 0.9},
            metadata_validation={"url_integrity": 1.0},
            execution_time=0.1,
            status=ValidationStatus.SUCCESS
        )
        report = ValidationReport(
            id="report_1",
            test_suite="test_suite_1",
            total_tests=1,
            passed_tests=1,
            failed_tests=0,
            results=[result],
            summary_metrics={"avg_accuracy": 0.9},
            status="pass"
        )

        report_service.store_report(report)

        # Export as JSON
        json_export = report_service.export_report("report_1", "json")
        assert "report_1" in json_export

        # Export as CSV
        csv_export = report_service.export_report("report_1", "csv")
        assert "Report ID" in csv_export

    def test_export_report_invalid_format(self, report_service):
        """
        Test export_report with invalid format
        """
        query = SearchQuery(query_text="test query")
        result = ValidationResult(
            id="vr_1",
            query=query,
            results=[],
            relevance_metrics={"accuracy": 0.9},
            metadata_validation={"url_integrity": 1.0},
            execution_time=0.1,
            status=ValidationStatus.SUCCESS
        )
        report = ValidationReport(
            id="report_1",
            test_suite="test_suite_1",
            total_tests=1,
            passed_tests=1,
            failed_tests=0,
            results=[result],
            summary_metrics={"avg_accuracy": 0.9},
            status="pass"
        )

        report_service.store_report(report)

        with pytest.raises(ValueError, match="Unsupported format: xml"):
            report_service.export_report("report_1", "xml")

    def test_export_report_not_found(self, report_service):
        """
        Test export_report with non-existent report ID
        """
        with pytest.raises(ValueError, match="Report with ID nonexistent not found"):
            report_service.export_report("nonexistent", "json")

    def test_calculate_report_summary_metrics(self, report_service):
        """
        Test calculate_report_summary_metrics method
        """
        query = SearchQuery(query_text="test query")
        result = ValidationResult(
            id="vr_1",
            query=query,
            results=[],
            relevance_metrics={"accuracy": 0.9},
            metadata_validation={"url_integrity": 1.0},
            execution_time=0.1,
            status=ValidationStatus.SUCCESS
        )
        report1 = ValidationReport(
            id="report_1",
            test_suite="test_suite_1",
            total_tests=10,
            passed_tests=9,
            failed_tests=1,
            results=[result],
            summary_metrics={"avg_accuracy": 0.9},
            status="pass",
            duration=5.0
        )
        report2 = ValidationReport(
            id="report_2",
            test_suite="test_suite_2",
            total_tests=5,
            passed_tests=4,
            failed_tests=1,
            results=[result],
            summary_metrics={"avg_accuracy": 0.85},
            status="warning",
            duration=3.0
        )

        reports = [report1, report2]
        summary = report_service.calculate_report_summary_metrics(reports)

        assert summary["total_reports"] == 2
        assert summary["total_tests"] == 15  # 10 + 5
        assert summary["total_passed"] == 13  # 9 + 4
        assert summary["total_failed"] == 2   # 1 + 1
        assert summary["success_rate"] == 13/15
        assert summary["avg_duration"] == 4.0  # (5.0 + 3.0) / 2
        assert summary["overall_status"] == "pass"  # Based on success rate

    def test_calculate_report_summary_metrics_empty(self, report_service):
        """
        Test calculate_report_summary_metrics with empty list
        """
        summary = report_service.calculate_report_summary_metrics([])
        assert summary == {}

    def test_update_and_get_validation_config(self, report_service):
        """
        Test update_validation_config and get_validation_config methods
        """
        from backend.src.validation.models.validation_models import ValidationConfig

        new_config = ValidationConfig(
            batch_size=20,
            concurrency=10,
            timeout=60.0,
            min_score_threshold=0.5,
            max_execution_time=600.0
        )

        updated_config = report_service.update_validation_config(new_config)
        assert updated_config.batch_size == 20
        assert updated_config.concurrency == 10

        retrieved_config = report_service.get_validation_config()
        assert retrieved_config.batch_size == 20
        assert retrieved_config.concurrency == 10