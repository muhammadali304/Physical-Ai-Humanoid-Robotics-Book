"""
Performance monitoring for the RAG Ingestion Pipeline.
Implements performance monitoring for key operations with Python-specific syntax highlighting.
"""

import time
import asyncio
import functools
from typing import Dict, Any, Optional, Callable, Union, List
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
import threading
import psutil
import os

from src.utils.logging import get_logger
from src.monitoring.metrics import get_metrics_collector, observe_histogram, increment_counter


class PerformanceOperation(str, Enum):
    """
    Enum for different types of operations that can be monitored.
    """
    CRAWL_PAGE = "crawl_page"
    EXTRACT_CONTENT = "extract_content"
    CHUNK_CONTENT = "chunk_content"
    GENERATE_EMBEDDING = "generate_embedding"
    STORE_EMBEDDING = "store_embedding"
    SEARCH_EMBEDDING = "search_embedding"
    PROCESS_JOB = "process_job"
    DATABASE_QUERY = "database_query"
    API_CALL = "api_call"


@dataclass
class PerformanceResult:
    """
    Result of a performance measurement.
    """
    operation: PerformanceOperation
    duration: float
    success: bool
    timestamp: datetime
    details: Optional[Dict[str, Any]] = None
    error: Optional[str] = None


class PerformanceMonitor:
    """
    Performance monitoring service for tracking operation performance.
    """

    def __init__(self):
        self.logger = get_logger("performance_monitor")
        self.metrics_collector = get_metrics_collector()
        self.performance_history: List[PerformanceResult] = []
        self.history_lock = threading.Lock()
        self.max_history_size = 1000  # Limit history to prevent memory issues

        # Performance thresholds (in seconds)
        self.thresholds = {
            PerformanceOperation.CRAWL_PAGE: 10.0,      # 10 seconds
            PerformanceOperation.EXTRACT_CONTENT: 2.0,  # 2 seconds
            PerformanceOperation.CHUNK_CONTENT: 1.0,    # 1 second
            PerformanceOperation.GENERATE_EMBEDDING: 5.0,  # 5 seconds
            PerformanceOperation.STORE_EMBEDDING: 2.0,  # 2 seconds
            PerformanceOperation.SEARCH_EMBEDDING: 1.0, # 1 second
            PerformanceOperation.PROCESS_JOB: 300.0,    # 5 minutes
            PerformanceOperation.DATABASE_QUERY: 5.0,   # 5 seconds
            PerformanceOperation.API_CALL: 30.0         # 30 seconds
        }

        self.logger.info("Performance monitor initialized")

    def set_threshold(self, operation: PerformanceOperation, threshold_seconds: float):
        """
        Set a performance threshold for an operation.

        Args:
            operation: Operation to set threshold for
            threshold_seconds: Threshold in seconds
        """
        self.thresholds[operation] = threshold_seconds
        self.logger.info(f"Set threshold for {operation.value}: {threshold_seconds}s",
                        operation=operation.value, threshold=threshold_seconds)

    def get_threshold(self, operation: PerformanceOperation) -> float:
        """
        Get the performance threshold for an operation.

        Args:
            operation: Operation to get threshold for

        Returns:
            Threshold in seconds
        """
        return self.thresholds.get(operation, 5.0)  # Default to 5 seconds

    def measure_operation(self, operation: PerformanceOperation, func: Callable,
                         *args, labels: Optional[Dict[str, str]] = None, **kwargs) -> PerformanceResult:
        """
        Measure the performance of a synchronous operation.

        Args:
            operation: Type of operation being measured
            func: Function to execute and measure
            *args: Arguments to pass to the function
            labels: Labels to attach to metrics
            **kwargs: Keyword arguments to pass to the function

        Returns:
            PerformanceResult with timing information
        """
        start_time = time.time()
        start_memory = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024  # MB

        try:
            result = func(*args, **kwargs)
            success = True
            error = None
        except Exception as e:
            success = False
            error = str(e)
            result = None
            self.logger.error(f"Operation {operation.value} failed: {str(e)}",
                            operation=operation.value, error=str(e))

        end_time = time.time()
        end_memory = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024  # MB
        duration = end_time - start_time
        memory_change = end_memory - start_memory

        # Create performance result
        perf_result = PerformanceResult(
            operation=operation,
            duration=duration,
            success=success,
            timestamp=datetime.utcnow(),
            details={
                "memory_change_mb": memory_change,
                "start_memory_mb": start_memory,
                "end_memory_mb": end_memory
            },
            error=error
        )

        # Record metrics
        self._record_performance_metrics(perf_result, labels)

        # Check if performance is above threshold
        threshold = self.get_threshold(operation)
        if duration > threshold:
            self.logger.warning(
                f"Performance threshold exceeded for {operation.value}: {duration:.2f}s > {threshold}s",
                operation=operation.value,
                duration=duration,
                threshold=threshold
            )

        # Add to history
        self._add_to_history(perf_result)

        return perf_result

    async def measure_async_operation(self, operation: PerformanceOperation, func: Callable,
                                     *args, labels: Optional[Dict[str, str]] = None, **kwargs) -> PerformanceResult:
        """
        Measure the performance of an asynchronous operation.

        Args:
            operation: Type of operation being measured
            func: Async function to execute and measure
            *args: Arguments to pass to the function
            labels: Labels to attach to metrics
            **kwargs: Keyword arguments to pass to the function

        Returns:
            PerformanceResult with timing information
        """
        start_time = time.time()
        start_memory = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024  # MB

        try:
            result = await func(*args, **kwargs)
            success = True
            error = None
        except Exception as e:
            success = False
            error = str(e)
            result = None
            self.logger.error(f"Async operation {operation.value} failed: {str(e)}",
                            operation=operation.value, error=str(e))

        end_time = time.time()
        end_memory = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024  # MB
        duration = end_time - start_time
        memory_change = end_memory - start_memory

        # Create performance result
        perf_result = PerformanceResult(
            operation=operation,
            duration=duration,
            success=success,
            timestamp=datetime.utcnow(),
            details={
                "memory_change_mb": memory_change,
                "start_memory_mb": start_memory,
                "end_memory_mb": end_memory
            },
            error=error
        )

        # Record metrics
        self._record_performance_metrics(perf_result, labels)

        # Check if performance is above threshold
        threshold = self.get_threshold(operation)
        if duration > threshold:
            self.logger.warning(
                f"Performance threshold exceeded for {operation.value}: {duration:.2f}s > {threshold}s",
                operation=operation.value,
                duration=duration,
                threshold=threshold
            )

        # Add to history
        self._add_to_history(perf_result)

        return perf_result

    def _record_performance_metrics(self, result: PerformanceResult, labels: Optional[Dict[str, str]] = None):
        """
        Record performance metrics to the metrics collector.

        Args:
            result: Performance result to record
            labels: Additional labels to attach to metrics
        """
        try:
            # Create base labels
            base_labels = {
                "operation": result.operation.value,
                "success": str(result.success).lower()
            }

            if labels:
                base_labels.update(labels)

            # Record duration histogram
            observe_histogram(
                "operation_duration_seconds",
                result.duration,
                base_labels
            )

            # Increment operation counter
            increment_counter(
                "operations_total",
                {**base_labels, "operation": result.operation.value}
            )

            # Record memory usage if available
            if result.details and "memory_change_mb" in result.details:
                observe_histogram(
                    "operation_memory_change_mb",
                    result.details["memory_change_mb"],
                    base_labels
                )

        except Exception as e:
            self.logger.error(f"Error recording performance metrics: {str(e)}", error=str(e))

    def _add_to_history(self, result: PerformanceResult):
        """
        Add a performance result to the history.

        Args:
            result: Performance result to add to history
        """
        try:
            with self.history_lock:
                self.performance_history.append(result)

                # Trim history if it exceeds max size
                if len(self.performance_history) > self.max_history_size:
                    self.performance_history = self.performance_history[-self.max_history_size:]

        except Exception as e:
            self.logger.error(f"Error adding to performance history: {str(e)}", error=str(e))

    def get_performance_summary(self) -> Dict[str, Any]:
        """
        Get a summary of performance metrics.

        Returns:
            Dictionary with performance summary statistics
        """
        try:
            if not self.performance_history:
                return {
                    "total_operations": 0,
                    "average_duration": 0.0,
                    "success_rate": 0.0,
                    "slow_operations": [],
                    "memory_usage_trends": {}
                }

            total_ops = len(self.performance_history)
            successful_ops = [op for op in self.performance_history if op.success]
            success_rate = len(successful_ops) / total_ops if total_ops > 0 else 0.0

            durations = [op.duration for op in successful_ops]
            avg_duration = sum(durations) / len(durations) if durations else 0.0

            # Find slow operations (above 95th percentile)
            if durations:
                sorted_durations = sorted(durations)
                percentile_95_idx = int(0.95 * len(sorted_durations))
                slow_threshold = sorted_durations[percentile_95_idx] if percentile_95_idx < len(sorted_durations) else sorted_durations[-1] if sorted_durations else 0.0
                slow_operations = [
                    {
                        "operation": op.operation.value,
                        "duration": op.duration,
                        "timestamp": op.timestamp.isoformat(),
                        "success": op.success
                    }
                    for op in self.performance_history
                    if op.duration > slow_threshold
                ]
            else:
                slow_operations = []

            # Memory usage trends
            memory_changes = [
                op.details.get("memory_change_mb", 0) if op.details else 0
                for op in self.performance_history
            ]
            avg_memory_change = sum(memory_changes) / len(memory_changes) if memory_changes else 0.0

            summary = {
                "total_operations": total_ops,
                "successful_operations": len(successful_ops),
                "failed_operations": total_ops - len(successful_ops),
                "average_duration": avg_duration,
                "success_rate": success_rate,
                "slow_operations": slow_operations,
                "memory_usage_trends": {
                    "average_memory_change_mb": avg_memory_change,
                    "total_memory_changes": len([mc for mc in memory_changes if mc != 0])
                }
            }

            return summary

        except Exception as e:
            self.logger.error(f"Error generating performance summary: {str(e)}", error=str(e))
            return {
                "total_operations": 0,
                "average_duration": 0.0,
                "success_rate": 0.0,
                "slow_operations": [],
                "memory_usage_trends": {}
            }

    def get_operation_performance(self, operation: PerformanceOperation) -> Dict[str, Any]:
        """
        Get performance statistics for a specific operation.

        Args:
            operation: Operation to get statistics for

        Returns:
            Dictionary with operation performance statistics
        """
        try:
            ops = [op for op in self.performance_history if op.operation == operation]
            if not ops:
                return {
                    "operation": operation.value,
                    "count": 0,
                    "average_duration": 0.0,
                    "success_rate": 0.0,
                    "min_duration": 0.0,
                    "max_duration": 0.0
                }

            successful_ops = [op for op in ops if op.success]
            durations = [op.duration for op in successful_ops]

            if not durations:
                return {
                    "operation": operation.value,
                    "count": len(ops),
                    "average_duration": 0.0,
                    "success_rate": 0.0,
                    "min_duration": 0.0,
                    "max_duration": 0.0
                }

            avg_duration = sum(durations) / len(durations)
            min_duration = min(durations)
            max_duration = max(durations)
            success_rate = len(successful_ops) / len(ops)

            return {
                "operation": operation.value,
                "count": len(ops),
                "successful_count": len(successful_ops),
                "average_duration": avg_duration,
                "success_rate": success_rate,
                "min_duration": min_duration,
                "max_duration": max_duration,
                "threshold_exceeded_count": len([d for d in durations if d > self.get_threshold(operation)])
            }

        except Exception as e:
            self.logger.error(f"Error getting operation performance for {operation.value}: {str(e)}",
                            operation=operation.value, error=str(e))
            return {
                "operation": operation.value,
                "count": 0,
                "average_duration": 0.0,
                "success_rate": 0.0,
                "min_duration": 0.0,
                "max_duration": 0.0
            }

    def reset_history(self):
        """
        Reset the performance history.
        """
        with self.history_lock:
            self.performance_history.clear()
            self.logger.info("Performance history reset")


def performance_monitor(operation: PerformanceOperation, labels: Optional[Dict[str, str]] = None):
    """
    Decorator to monitor the performance of a synchronous function.

    Args:
        operation: Type of operation being monitored
        labels: Labels to attach to metrics

    Returns:
        Decorator function
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            # Get the performance monitor instance
            from src.monitoring.performance import get_performance_monitor
            monitor = get_performance_monitor()

            # Execute and measure the operation
            result = monitor.measure_operation(operation, func, *args, labels=labels, **kwargs)
            return result

        return wrapper
    return decorator


def async_performance_monitor(operation: PerformanceOperation, labels: Optional[Dict[str, str]] = None):
    """
    Decorator to monitor the performance of an asynchronous function.

    Args:
        operation: Type of operation being monitored
        labels: Labels to attach to metrics

    Returns:
        Decorator function
    """
    def decorator(func):
        @functools.wraps(func)
        async def wrapper(*args, **kwargs):
            # Get the performance monitor instance
            from src.monitoring.performance import get_performance_monitor
            monitor = get_performance_monitor()

            # Execute and measure the operation
            result = await monitor.measure_async_operation(operation, func, *args, labels=labels, **kwargs)
            return result

        return wrapper
    return decorator


def time_it(operation: PerformanceOperation, labels: Optional[Dict[str, str]] = None):
    """
    Context manager to time a block of code.

    Args:
        operation: Type of operation being timed
        labels: Labels to attach to metrics

    Returns:
        Context manager
    """
    class Timer:
        def __enter__(self):
            self.start_time = time.time()
            self.start_memory = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024  # MB
            return self

        def __exit__(self, exc_type, exc_val, exc_tb):
            end_time = time.time()
            end_memory = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024  # MB
            duration = end_time - self.start_time
            memory_change = end_memory - self.start_memory

            # Get the performance monitor instance
            monitor = get_performance_monitor()

            # Create performance result
            success = exc_type is None
            error = str(exc_val) if exc_type else None

            perf_result = PerformanceResult(
                operation=operation,
                duration=duration,
                success=success,
                timestamp=datetime.utcnow(),
                details={
                    "memory_change_mb": memory_change,
                    "start_memory_mb": self.start_memory,
                    "end_memory_mb": end_memory
                },
                error=error
            )

            # Record metrics
            monitor._record_performance_metrics(perf_result, labels)

            # Check threshold
            threshold = monitor.get_threshold(operation)
            if duration > threshold:
                monitor.logger.warning(
                    f"Performance threshold exceeded for {operation.value}: {duration:.2f}s > {threshold}s",
                    operation=operation.value,
                    duration=duration,
                    threshold=threshold
                )

            # Add to history
            monitor._add_to_history(perf_result)

    return Timer()


# Global performance monitor instance
performance_monitor_instance = PerformanceMonitor()


def get_performance_monitor() -> PerformanceMonitor:
    """
    Get the global performance monitor instance.

    Returns:
        PerformanceMonitor instance
    """
    return performance_monitor_instance


# Convenience functions
def measure_operation(operation: PerformanceOperation, func: Callable,
                    *args, labels: Optional[Dict[str, str]] = None, **kwargs) -> PerformanceResult:
    """
    Convenience function to measure a synchronous operation.
    """
    return performance_monitor_instance.measure_operation(operation, func, *args, labels=labels, **kwargs)


async def measure_async_operation(operation: PerformanceOperation, func: Callable,
                                 *args, labels: Optional[Dict[str, str]] = None, **kwargs) -> PerformanceResult:
    """
    Convenience function to measure an asynchronous operation.
    """
    return await performance_monitor_instance.measure_async_operation(operation, func, *args, labels=labels, **kwargs)


def get_performance_summary() -> Dict[str, Any]:
    """
    Convenience function to get performance summary.
    """
    return performance_monitor_instance.get_performance_summary()


def get_operation_performance(operation: PerformanceOperation) -> Dict[str, Any]:
    """
    Convenience function to get performance for a specific operation.
    """
    return performance_monitor_instance.get_operation_performance(operation)


def set_threshold(operation: PerformanceOperation, threshold_seconds: float):
    """
    Convenience function to set a performance threshold.
    """
    performance_monitor_instance.set_threshold(operation, threshold_seconds)


def reset_performance_history():
    """
    Convenience function to reset performance history.
    """
    performance_monitor_instance.reset_history()


# Export for use in other modules
__all__ = [
    'PerformanceMonitor',
    'PerformanceOperation',
    'PerformanceResult',
    'performance_monitor',
    'async_performance_monitor',
    'time_it',
    'get_performance_monitor',
    'measure_operation',
    'measure_async_operation',
    'get_performance_summary',
    'get_operation_performance',
    'set_threshold',
    'reset_performance_history',
    'performance_monitor_instance'
]