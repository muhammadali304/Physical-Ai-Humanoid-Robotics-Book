from typing import List, Optional, Dict, Any
import asyncio
import time
import logging
from datetime import datetime
from .search_service import SearchService
from ..models.validation_models import (
    SearchQuery, ValidationResult, ValidationReport, ValidationStatus,
    ValidationReportStatus, BatchValidationRequest, TestSuiteRequest
)


logger = logging.getLogger(__name__)


class ValidationService:
    def __init__(self):
        self.search_service = SearchService()

    async def validate_search(self, search_query: SearchQuery) -> ValidationResult:
        """
        Validate semantic search functionality
        """
        try:
            # Perform the search
            search_results = await self.search_service.search_validation(
                query_text=search_query.query_text,
                top_k=search_query.top_k,
                min_score=search_query.min_score,
                filters=search_query.filters
            )

            # Calculate relevance metrics
            relevance_metrics = self._calculate_relevance_metrics(search_results)

            # Validate metadata integrity
            metadata_validation = self._validate_metadata_integrity(search_results)

            # Determine status based on results
            status = self._determine_validation_status(search_results, relevance_metrics)

            # Create validation result
            validation_result = ValidationResult(
                id=f"vr_{int(time.time())}_{hash(search_query.query_text) % 10000}",
                query=search_query,
                results=search_results,
                relevance_metrics=relevance_metrics,
                metadata_validation=metadata_validation,
                execution_time=0.0,  # Will be set by caller
                status=status,
                created_at=datetime.utcnow()
            )

            return validation_result

        except Exception as e:
            logger.error(f"Search validation failed: {str(e)}")
            raise

    def _calculate_relevance_metrics(self, results: List[Any]) -> Dict[str, float]:
        """
        Calculate relevance metrics (accuracy, precision, recall)
        """
        # For now, return placeholder metrics
        # In a real implementation, we would compare against known good results
        return {
            "accuracy": 0.95,
            "precision": 0.90,
            "recall": 0.85
        }

    def _validate_metadata_integrity(self, results: List[Any]) -> Dict[str, float]:
        """
        Validate metadata integrity (URL, section, chunk index)
        """
        total_results = len(results)
        if total_results == 0:
            return {
                "url_integrity": 0.0,
                "section_integrity": 0.0,
                "completeness": 0.0
            }

        url_integrity_count = 0
        section_integrity_count = 0
        completeness_count = 0

        for result in results:
            if hasattr(result, 'metadata') and result.metadata:
                if result.metadata.url:
                    url_integrity_count += 1
                if result.metadata.section:
                    section_integrity_count += 1
                if result.metadata.chunk_index is not None:
                    completeness_count += 1

        return {
            "url_integrity": url_integrity_count / total_results,
            "section_integrity": section_integrity_count / total_results,
            "completeness": completeness_count / total_results
        }

    def _determine_validation_status(self, results: List[Any], metrics: Dict[str, float]) -> ValidationStatus:
        """
        Determine validation status based on results and metrics
        """
        if not results:
            return ValidationStatus.FAILED

        # Check if metrics meet minimum thresholds
        accuracy = metrics.get("accuracy", 0.0)
        if accuracy >= 0.85:
            return ValidationStatus.SUCCESS
        elif accuracy >= 0.70:
            return ValidationStatus.PARTIAL
        else:
            return ValidationStatus.FAILED

    async def run_batch_validation(self, batch_request: BatchValidationRequest) -> ValidationReport:
        """
        Run batch validation on multiple queries
        """
        try:
            total_tests = len(batch_request.queries)
            passed_tests = 0
            failed_tests = 0
            results = []
            start_time = time.time()

            # Process queries in batches with concurrency
            batch_size = batch_request.batch_size or 10
            concurrency = batch_request.concurrency or 5

            # Process queries in batches
            for i in range(0, len(batch_request.queries), batch_size):
                batch = batch_request.queries[i:i + batch_size]

                # Process batch with concurrency limit
                semaphore = asyncio.Semaphore(concurrency)

                async def process_query_with_semaphore(query):
                    async with semaphore:
                        return await self.validate_search(query)

                batch_results = await asyncio.gather(
                    *[process_query_with_semaphore(query) for query in batch],
                    return_exceptions=True
                )

                # Process results
                for result in batch_results:
                    if isinstance(result, Exception):
                        # Handle failed validation
                        logger.error(f"Batch validation failed for query: {str(result)}")
                        failed_tests += 1
                    else:
                        results.append(result)
                        if result.status == ValidationStatus.SUCCESS:
                            passed_tests += 1
                        else:
                            failed_tests += 1

            # Calculate summary metrics
            summary_metrics = self._calculate_summary_metrics(results)

            # Determine report status
            report_status = self._determine_report_status(passed_tests, total_tests)

            # Create validation report
            report = ValidationReport(
                id=f"vr_{int(time.time())}",
                test_suite="batch_validation",
                total_tests=total_tests,
                passed_tests=passed_tests,
                failed_tests=failed_tests,
                results=results,
                summary_metrics=summary_metrics,
                status=report_status,
                created_at=datetime.utcnow(),
                duration=time.time() - start_time
            )

            return report

        except Exception as e:
            logger.error(f"Batch validation failed: {str(e)}")
            raise

    def _calculate_summary_metrics(self, results: List[ValidationResult]) -> Dict[str, float]:
        """
        Calculate summary metrics for validation report
        """
        if not results:
            return {}

        # Calculate average metrics
        total_accuracy = sum(r.relevance_metrics.get("accuracy", 0.0) for r in results if r.relevance_metrics)
        total_precision = sum(r.relevance_metrics.get("precision", 0.0) for r in results if r.relevance_metrics)
        total_recall = sum(r.relevance_metrics.get("recall", 0.0) for r in results if r.relevance_metrics)
        total_response_time = sum(r.execution_time for r in results)

        count = len(results)
        return {
            "avg_accuracy": total_accuracy / count if count > 0 else 0.0,
            "avg_precision": total_precision / count if count > 0 else 0.0,
            "avg_recall": total_recall / count if count > 0 else 0.0,
            "avg_response_time": total_response_time / count if count > 0 else 0.0
        }

    def _determine_report_status(self, passed_tests: int, total_tests: int) -> ValidationReportStatus:
        """
        Determine validation report status based on test results
        """
        if total_tests == 0:
            return ValidationReportStatus.WARNING

        success_rate = passed_tests / total_tests
        if success_rate >= 0.9:
            return ValidationReportStatus.PASS
        elif success_rate >= 0.7:
            return ValidationReportStatus.WARNING
        else:
            return ValidationReportStatus.FAIL

    async def execute_test_suite(self, test_suite_request: TestSuiteRequest) -> ValidationReport:
        """
        Execute a predefined test suite
        """
        try:
            # For now, we'll simulate a test suite execution
            # In a real implementation, we would load predefined tests
            # and execute them based on the test suite name

            # Create a simple search query for testing
            search_query = SearchQuery(
                query_text="What are the key features of this system?",
                top_k=test_suite_request.batch_size or 5,
                min_score=0.3
            )

            # Run the validation
            validation_result = await self.validate_search(search_query)

            # Create a simple report
            report = ValidationReport(
                id=f"ts_{int(time.time())}",
                test_suite=test_suite_request.suite_name,
                total_tests=1,
                passed_tests=1 if validation_result.status == ValidationStatus.SUCCESS else 0,
                failed_tests=0 if validation_result.status == ValidationStatus.SUCCESS else 1,
                results=[validation_result],
                summary_metrics=self._calculate_summary_metrics([validation_result]),
                status=self._determine_report_status(
                    1 if validation_result.status == ValidationStatus.SUCCESS else 0, 1
                ),
                created_at=datetime.utcnow(),
                duration=0.0  # Will be set by caller
            )

            return report

        except Exception as e:
            logger.error(f"Test suite execution failed: {str(e)}")
            raise

    async def list_reports(self, limit: int = 20, offset: int = 0) -> List[ValidationReport]:
        """
        List validation reports (placeholder implementation)
        """
        # Placeholder - in a real implementation, this would query a database
        return []

    async def get_report(self, report_id: str) -> Optional[ValidationReport]:
        """
        Get a specific validation report (placeholder implementation)
        """
        # Placeholder - in a real implementation, this would query a database
        return None