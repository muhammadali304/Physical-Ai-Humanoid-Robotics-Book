import csv
import json
from typing import List, Dict, Any, Optional
from datetime import datetime
import io
from ..models.validation_models import ValidationReport, ValidationConfig


class ReportService:
    """
    Service for generating validation reports in different formats
    """

    def __init__(self):
        self.reports_storage = {}  # In-memory storage for demo purposes
        self.config = ValidationConfig()

    def generate_json_report(self, report: ValidationReport) -> str:
        """
        Generate validation report in JSON format
        """
        report_dict = report.dict()
        return json.dumps(report_dict, indent=2, default=str)

    def generate_csv_report(self, report: ValidationReport) -> str:
        """
        Generate validation report in CSV format
        """
        output = io.StringIO()
        writer = csv.writer(output)

        # Write header
        writer.writerow([
            "Report ID", "Test Suite", "Total Tests", "Passed Tests", "Failed Tests",
            "Status", "Created At", "Duration"
        ])

        # Write report data
        writer.writerow([
            report.id, report.test_suite, report.total_tests,
            report.passed_tests, report.failed_tests,
            report.status.value, report.created_at, report.duration
        ])

        # Add summary metrics as additional rows if available
        if report.summary_metrics:
            writer.writerow([])
            writer.writerow(["Summary Metrics"])
            for key, value in report.summary_metrics.items():
                writer.writerow([key, value])

        # Add individual results if available
        if report.results:
            writer.writerow([])
            writer.writerow(["Individual Results"])
            writer.writerow([
                "Result ID", "Query Text", "Status", "Execution Time",
                "Accuracy", "Precision", "Recall"
            ])

            for result in report.results:
                accuracy = result.relevance_metrics.get("accuracy", "") if result.relevance_metrics else ""
                precision = result.relevance_metrics.get("precision", "") if result.relevance_metrics else ""
                recall = result.relevance_metrics.get("recall", "") if result.relevance_metrics else ""

                writer.writerow([
                    result.id,
                    result.query.query_text[:50] + "..." if len(result.query.query_text) > 50 else result.query.query_text,
                    result.status.value,
                    result.execution_time,
                    accuracy,
                    precision,
                    recall
                ])

        return output.getvalue()

    def store_report(self, report: ValidationReport) -> str:
        """
        Store validation report in the reports storage
        """
        self.reports_storage[report.id] = report
        return report.id

    def get_report(self, report_id: str) -> Optional[ValidationReport]:
        """
        Retrieve validation report by ID
        """
        return self.reports_storage.get(report_id)

    def list_reports(self, limit: int = 20, offset: int = 0) -> List[ValidationReport]:
        """
        List validation reports with pagination
        """
        all_reports = list(self.reports_storage.values())
        start_idx = offset
        end_idx = offset + limit
        return all_reports[start_idx:end_idx]

    def export_report(self, report_id: str, format_type: str = "json") -> str:
        """
        Export validation report in specified format
        """
        report = self.get_report(report_id)
        if not report:
            raise ValueError(f"Report with ID {report_id} not found")

        if format_type.lower() == "json":
            return self.generate_json_report(report)
        elif format_type.lower() == "csv":
            return self.generate_csv_report(report)
        else:
            raise ValueError(f"Unsupported format: {format_type}. Supported formats: json, csv")

    def calculate_report_summary_metrics(self, reports: List[ValidationReport]) -> Dict[str, Any]:
        """
        Calculate summary metrics across multiple validation reports
        """
        if not reports:
            return {}

        total_tests = sum(r.total_tests for r in reports)
        total_passed = sum(r.passed_tests for r in reports)
        total_failed = sum(r.failed_tests for r in reports)

        # Calculate average duration
        durations = [r.duration for r in reports if r.duration is not None]
        avg_duration = sum(durations) / len(durations) if durations else 0.0

        # Calculate success rate
        success_rate = total_passed / total_tests if total_tests > 0 else 0.0

        # Determine overall status
        if success_rate >= 0.9:
            overall_status = "pass"
        elif success_rate >= 0.7:
            overall_status = "warning"
        else:
            overall_status = "fail"

        return {
            "total_reports": len(reports),
            "total_tests": total_tests,
            "total_passed": total_passed,
            "total_failed": total_failed,
            "success_rate": success_rate,
            "avg_duration": avg_duration,
            "overall_status": overall_status
        }

    def cleanup_old_reports(self, days_to_keep: int = 30) -> int:
        """
        Remove reports older than specified number of days
        """
        cutoff_date = datetime.utcnow().timestamp() - (days_to_keep * 24 * 60 * 60)
        old_report_ids = [
            report_id for report_id, report in self.reports_storage.items()
            if report.created_at.timestamp() < cutoff_date
        ]

        for report_id in old_report_ids:
            del self.reports_storage[report_id]

        return len(old_report_ids)

    def update_validation_config(self, new_config: ValidationConfig) -> ValidationConfig:
        """
        Update validation configuration
        """
        self.config = new_config
        return self.config

    def get_validation_config(self) -> ValidationConfig:
        """
        Get current validation configuration
        """
        return self.config