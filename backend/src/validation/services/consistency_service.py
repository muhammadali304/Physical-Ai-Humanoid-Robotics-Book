from typing import List, Dict, Any
import asyncio
import time
from .search_service import SearchService
from .metrics_service import MetricsService
from ..models.validation_models import (
    SearchResult, ValidationReport, ValidationTest,
    ValidationReportStatus, ValidationStatus
)


class ConsistencyService:
    """
    Service for validating pipeline consistency and repeated query validation
    """

    def __init__(self):
        self.search_service = SearchService()
        self.metrics_service = MetricsService()

    async def validate_consistency_for_query(
        self,
        query_text: str,
        num_repetitions: int = 5,
        top_k: int = 5,
        min_score: float = 0.0,
        filters: Dict[str, Any] = None
    ) -> Dict[str, Any]:
        """
        Validate consistency by running the same query multiple times
        """
        results = []
        execution_times = []

        for i in range(num_repetitions):
            start_time = time.time()
            search_results = await self.search_service.search_validation(
                query_text=query_text,
                top_k=top_k,
                min_score=min_score,
                filters=filters
            )
            execution_time = time.time() - start_time
            results.append(search_results)
            execution_times.append(execution_time)

        # Calculate consistency metrics
        consistency_metrics = self.metrics_service.calculate_consistency_metrics(query_text, results)

        # Calculate average execution time
        avg_execution_time = sum(execution_times) / len(execution_times) if execution_times else 0.0

        return {
            "query": query_text,
            "repetitions": num_repetitions,
            "results": results,
            "consistency_metrics": consistency_metrics,
            "avg_execution_time": avg_execution_time,
            "execution_times": execution_times
        }

    async def run_test_suite_execution(
        self,
        test_suite_name: str,
        tests: List[ValidationTest],
        concurrency: int = 5
    ) -> ValidationReport:
        """
        Execute a predefined test suite with multiple validation tests
        """
        start_time = time.time()
        total_tests = len(tests)
        passed_tests = 0
        failed_tests = 0
        results = []

        # Use semaphore to limit concurrency
        semaphore = asyncio.Semaphore(concurrency)

        async def execute_test_with_semaphore(test: ValidationTest):
            async with semaphore:
                try:
                    search_query = {
                        "query_text": test.query,
                        "top_k": 5,
                        "min_score": 0.3
                    }
                    # In a real implementation, we would execute the actual test
                    # For now, we'll simulate the test execution
                    search_results = await self.search_service.search_validation(
                        query_text=test.query,
                        top_k=5,
                        min_score=0.3
                    )

                    # Create a mock validation result
                    from ..models.validation_models import (
                        SearchQuery, ValidationResult
                    )
                    from datetime import datetime
                    import time as time_module

                    search_query_obj = SearchQuery(
                        query_text=test.query,
                        top_k=5,
                        min_score=0.3
                    )

                    # Calculate mock metrics
                    relevance_metrics = self.metrics_service.calculate_relevance_metrics(search_results)
                    metadata_validation = self.metrics_service.calculate_metadata_validation(search_results)

                    # Determine status based on thresholds
                    accuracy = relevance_metrics.get("accuracy", 0.0)
                    status = ValidationStatus.SUCCESS if accuracy >= 0.8 else ValidationStatus.FAILED

                    validation_result = ValidationResult(
                        id=f"vr_{int(time_module.time())}_{hash(test.query) % 10000}",
                        query=search_query_obj,
                        results=search_results,
                        relevance_metrics=relevance_metrics,
                        metadata_validation=metadata_validation,
                        execution_time=0.1,  # Mock execution time
                        status=status,
                        created_at=datetime.utcnow()
                    )

                    return validation_result
                except Exception as e:
                    # Return failed result if test execution fails
                    from ..models.validation_models import (
                        SearchQuery, ValidationResult
                    )
                    from datetime import datetime
                    import time as time_module

                    search_query_obj = SearchQuery(
                        query_text=test.query,
                        top_k=5,
                        min_score=0.3
                    )

                    validation_result = ValidationResult(
                        id=f"vr_{int(time_module.time())}_{hash(test.query) % 10000}_error",
                        query=search_query_obj,
                        results=[],
                        relevance_metrics={},
                        metadata_validation={},
                        execution_time=0.0,
                        status=ValidationStatus.FAILED,
                        created_at=datetime.utcnow()
                    )
                    return validation_result

        # Execute all tests concurrently with limited concurrency
        test_results = await asyncio.gather(
            *[execute_test_with_semaphore(test) for test in tests],
            return_exceptions=True
        )

        # Process results
        for result in test_results:
            if isinstance(result, Exception):
                # If there was an exception during test execution, count as failed
                failed_tests += 1
            else:
                results.append(result)
                if result.status == ValidationStatus.SUCCESS:
                    passed_tests += 1
                else:
                    failed_tests += 1

        # Calculate summary metrics
        summary_metrics = self.metrics_service.calculate_validation_summary_metrics(results)

        # Determine report status
        if total_tests > 0:
            success_rate = passed_tests / total_tests
            if success_rate >= 0.9:
                report_status = ValidationReportStatus.PASS
            elif success_rate >= 0.7:
                report_status = ValidationReportStatus.WARNING
            else:
                report_status = ValidationReportStatus.FAIL
        else:
            report_status = ValidationReportStatus.WARNING

        # Create validation report
        report = ValidationReport(
            id=f"ts_{int(time.time())}_{test_suite_name}",
            test_suite=test_suite_name,
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

    async def run_batch_validation_processing(
        self,
        queries: List[Dict[str, Any]],
        batch_size: int = 10,
        concurrency: int = 5
    ) -> ValidationReport:
        """
        Process multiple validation queries in batches with configurable concurrency
        """
        start_time = time.time()
        total_tests = len(queries)
        passed_tests = 0
        failed_tests = 0
        results = []

        # Process queries in batches
        for i in range(0, len(queries), batch_size):
            batch = queries[i:i + batch_size]

            # Use semaphore to limit concurrency
            semaphore = asyncio.Semaphore(concurrency)

            async def process_query_with_semaphore(query_params: Dict[str, Any]):
                async with semaphore:
                    try:
                        search_results = await self.search_service.search_validation(
                            query_text=query_params.get("query_text", ""),
                            top_k=query_params.get("top_k", 5),
                            min_score=query_params.get("min_score", 0.3),
                            filters=query_params.get("filters", {})
                        )

                        # Create validation result
                        from ..models.validation_models import (
                            SearchQuery, ValidationResult
                        )
                        from datetime import datetime
                        import time as time_module

                        search_query = SearchQuery(
                            query_text=query_params.get("query_text", ""),
                            top_k=query_params.get("top_k", 5),
                            min_score=query_params.get("min_score", 0.3),
                            filters=query_params.get("filters", {})
                        )

                        # Calculate metrics
                        relevance_metrics = self.metrics_service.calculate_relevance_metrics(search_results)
                        metadata_validation = self.metrics_service.calculate_metadata_validation(search_results)

                        # Determine status
                        accuracy = relevance_metrics.get("accuracy", 0.0)
                        status = ValidationStatus.SUCCESS if accuracy >= 0.8 else ValidationStatus.FAILED

                        validation_result = ValidationResult(
                            id=f"bv_{int(time_module.time())}_{hash(query_params.get('query_text', '')) % 10000}",
                            query=search_query,
                            results=search_results,
                            relevance_metrics=relevance_metrics,
                            metadata_validation=metadata_validation,
                            execution_time=0.1,  # Mock execution time
                            status=status,
                            created_at=datetime.utcnow()
                        )

                        return validation_result
                    except Exception as e:
                        # Return failed result if query processing fails
                        from ..models.validation_models import (
                            SearchQuery, ValidationResult
                        )
                        from datetime import datetime
                        import time as time_module

                        search_query = SearchQuery(
                            query_text=query_params.get("query_text", ""),
                            top_k=query_params.get("top_k", 5),
                            min_score=query_params.get("min_score", 0.3),
                            filters=query_params.get("filters", {})
                        )

                        validation_result = ValidationResult(
                            id=f"bv_{int(time_module.time())}_{hash(query_params.get('query_text', '')) % 10000}_error",
                            query=search_query,
                            results=[],
                            relevance_metrics={},
                            metadata_validation={},
                            execution_time=0.0,
                            status=ValidationStatus.FAILED,
                            created_at=datetime.utcnow()
                        )
                        return validation_result

            # Process batch with limited concurrency
            batch_results = await asyncio.gather(
                *[process_query_with_semaphore(query) for query in batch],
                return_exceptions=True
            )

            # Process batch results
            for result in batch_results:
                if isinstance(result, Exception):
                    failed_tests += 1
                else:
                    results.append(result)
                    if result.status == ValidationStatus.SUCCESS:
                        passed_tests += 1
                    else:
                        failed_tests += 1

        # Calculate summary metrics
        summary_metrics = self.metrics_service.calculate_validation_summary_metrics(results)

        # Determine report status
        if total_tests > 0:
            success_rate = passed_tests / total_tests
            if success_rate >= 0.9:
                report_status = ValidationReportStatus.PASS
            elif success_rate >= 0.7:
                report_status = ValidationReportStatus.WARNING
            else:
                report_status = ValidationReportStatus.FAIL
        else:
            report_status = ValidationReportStatus.WARNING

        # Create validation report
        report = ValidationReport(
            id=f"bv_{int(time.time())}",
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