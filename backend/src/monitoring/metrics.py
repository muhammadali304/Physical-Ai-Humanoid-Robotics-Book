"""
Application metrics collection for the RAG Ingestion Pipeline.
Implements metrics collection with Python-specific syntax highlighting and Prometheus-style metrics.
"""

import time
import asyncio
from typing import Dict, Any, Optional, Callable, Union
from enum import Enum
from dataclasses import dataclass
from datetime import datetime
import functools
import inspect

from prometheus_client import Counter, Histogram, Gauge, Summary, CollectorRegistry, generate_latest, REGISTRY
from prometheus_client.exposition import MetricsHandler
from fastapi import Request, Response

from src.utils.logging import get_logger


class MetricType(Enum):
    """
    Types of metrics that can be collected.
    """
    COUNTER = "counter"
    HISTOGRAM = "histogram"
    GAUGE = "gauge"
    SUMMARY = "summary"


@dataclass
class MetricInfo:
    """
    Information about a metric.
    """
    name: str
    type: MetricType
    description: str
    labels: Optional[Dict[str, str]] = None


class MetricsCollector:
    """
    Centralized metrics collector for the application.
    """

    def __init__(self):
        self.logger = get_logger("metrics_collector")
        self.registry = CollectorRegistry()

        # Initialize various metric types
        self.metrics: Dict[str, Union[Counter, Histogram, Gauge, Summary]] = {}

        # System-level metrics
        self.system_metrics = {
            "api_requests_total": Counter(
                "api_requests_total",
                "Total number of API requests",
                ["method", "endpoint", "status_code"],
                registry=self.registry
            ),
            "api_request_duration_seconds": Histogram(
                "api_request_duration_seconds",
                "Duration of API requests in seconds",
                ["method", "endpoint"],
                registry=self.registry
            ),
            "active_crawl_jobs": Gauge(
                "active_crawl_jobs",
                "Number of currently active crawl jobs",
                registry=self.registry
            ),
            "processed_documents_total": Counter(
                "processed_documents_total",
                "Total number of documents processed",
                ["source_type"],
                registry=self.registry
            ),
            "embedding_generation_duration_seconds": Histogram(
                "embedding_generation_duration_seconds",
                "Duration of embedding generation in seconds",
                registry=self.registry
            ),
            "qdrant_operations_total": Counter(
                "qdrant_operations_total",
                "Total number of Qdrant operations",
                ["operation", "status"],
                registry=self.registry
            ),
            "crawling_rate_pages_per_minute": Gauge(
                "crawling_rate_pages_per_minute",
                "Current crawling rate in pages per minute",
                registry=self.registry
            )
        }

        # Add all system metrics to our metrics dict
        self.metrics.update(self.system_metrics)

        self.logger.info("Metrics collector initialized")

    def increment_counter(self, name: str, labels: Optional[Dict[str, str]] = None, amount: float = 1.0):
        """
        Increment a counter metric.

        Args:
            name: Name of the counter metric
            labels: Labels to attach to the metric
            amount: Amount to increment by (default: 1.0)
        """
        try:
            if name in self.metrics:
                metric = self.metrics[name]
                if isinstance(metric, Counter):
                    if labels:
                        metric.labels(**labels).inc(amount)
                    else:
                        metric.inc(amount)
                else:
                    self.logger.warning(f"Metric {name} is not a Counter type")
            else:
                self.logger.warning(f"Counter metric {name} not found")
        except Exception as e:
            self.logger.error(f"Error incrementing counter {name}: {str(e)}", error=str(e))

    def observe_histogram(self, name: str, value: float, labels: Optional[Dict[str, str]] = None):
        """
        Observe a value in a histogram metric.

        Args:
            name: Name of the histogram metric
            value: Value to observe
            labels: Labels to attach to the metric
        """
        try:
            if name in self.metrics:
                metric = self.metrics[name]
                if isinstance(metric, Histogram):
                    if labels:
                        metric.labels(**labels).observe(value)
                    else:
                        metric.observe(value)
                else:
                    self.logger.warning(f"Metric {name} is not a Histogram type")
            else:
                self.logger.warning(f"Histogram metric {name} not found")
        except Exception as e:
            self.logger.error(f"Error observing histogram {name}: {str(e)}", error=str(e))

    def set_gauge(self, name: str, value: float, labels: Optional[Dict[str, str]] = None):
        """
        Set a gauge metric to a specific value.

        Args:
            name: Name of the gauge metric
            value: Value to set
            labels: Labels to attach to the metric
        """
        try:
            if name in self.metrics:
                metric = self.metrics[name]
                if isinstance(metric, Gauge):
                    if labels:
                        metric.labels(**labels).set(value)
                    else:
                        metric.set(value)
                else:
                    self.logger.warning(f"Metric {name} is not a Gauge type")
            else:
                self.logger.warning(f"Gauge metric {name} not found")
        except Exception as e:
            self.logger.error(f"Error setting gauge {name}: {str(e)}", error=str(e))

    def observe_summary(self, name: str, value: float, labels: Optional[Dict[str, str]] = None):
        """
        Observe a value in a summary metric.

        Args:
            name: Name of the summary metric
            value: Value to observe
            labels: Labels to attach to the metric
        """
        try:
            if name in self.metrics:
                metric = self.metrics[name]
                if isinstance(metric, Summary):
                    if labels:
                        metric.labels(**labels).observe(value)
                    else:
                        metric.observe(value)
                else:
                    self.logger.warning(f"Metric {name} is not a Summary type")
            else:
                self.logger.warning(f"Summary metric {name} not found")
        except Exception as e:
            self.logger.error(f"Error observing summary {name}: {str(e)}", error=str(e))

    def create_custom_metric(self, name: str, metric_type: MetricType, description: str,
                           label_names: Optional[list] = None) -> Optional[Union[Counter, Histogram, Gauge, Summary]]:
        """
        Create a custom metric.

        Args:
            name: Name of the metric
            metric_type: Type of metric to create
            description: Description of the metric
            label_names: Names of labels for the metric

        Returns:
            The created metric object or None if creation failed
        """
        try:
            if name in self.metrics:
                self.logger.warning(f"Metric {name} already exists")
                return self.metrics[name]

            if metric_type == MetricType.COUNTER:
                if label_names:
                    metric = Counter(name, description, label_names, registry=self.registry)
                else:
                    metric = Counter(name, description, registry=self.registry)
            elif metric_type == MetricType.HISTOGRAM:
                if label_names:
                    metric = Histogram(name, description, label_names, registry=self.registry)
                else:
                    metric = Histogram(name, description, registry=self.registry)
            elif metric_type == MetricType.GAUGE:
                if label_names:
                    metric = Gauge(name, description, label_names, registry=self.registry)
                else:
                    metric = Gauge(name, description, registry=self.registry)
            elif metric_type == MetricType.SUMMARY:
                if label_names:
                    metric = Summary(name, description, label_names, registry=self.registry)
                else:
                    metric = Summary(name, description, registry=self.registry)
            else:
                self.logger.error(f"Unknown metric type: {metric_type}")
                return None

            self.metrics[name] = metric
            self.logger.info(f"Created custom metric: {name} ({metric_type.value})")
            return metric

        except Exception as e:
            self.logger.error(f"Error creating custom metric {name}: {str(e)}", error=str(e))
            return None

    def get_metrics(self) -> str:
        """
        Get all collected metrics in Prometheus format.

        Returns:
            String containing metrics in Prometheus format
        """
        try:
            return generate_latest(self.registry).decode('utf-8')
        except Exception as e:
            self.logger.error(f"Error generating metrics: {str(e)}", error=str(e))
            return ""

    def api_metrics_middleware(self, request: Request, response: Response, start_time: float):
        """
        Middleware function to collect API request metrics.

        Args:
            request: The incoming request
            response: The outgoing response
            start_time: Start time of the request (for duration calculation)
        """
        try:
            duration = time.time() - start_time

            # Extract endpoint from path
            endpoint = request.url.path
            method = request.method
            status_code = response.status_code

            # Increment request counter
            self.increment_counter(
                "api_requests_total",
                labels={
                    "method": method,
                    "endpoint": endpoint,
                    "status_code": str(status_code)
                }
            )

            # Record request duration
            self.observe_histogram(
                "api_request_duration_seconds",
                duration,
                labels={
                    "method": method,
                    "endpoint": endpoint
                }
            )

            self.logger.debug(
                f"Recorded API metrics: {method} {endpoint} -> {status_code} in {duration:.3f}s",
                method=method,
                endpoint=endpoint,
                status_code=status_code,
                duration=duration
            )
        except Exception as e:
            self.logger.error(f"Error recording API metrics: {str(e)}", error=str(e))

    def embedding_generation_timer(self) -> float:
        """
        Start a timer for embedding generation duration measurement.

        Returns:
            Start time for the timer
        """
        return time.time()

    def record_embedding_generation_time(self, start_time: float):
        """
        Record the duration of embedding generation.

        Args:
            start_time: Start time of the embedding generation
        """
        try:
            duration = time.time() - start_time
            self.observe_histogram("embedding_generation_duration_seconds", duration)
            self.logger.debug(f"Recorded embedding generation time: {duration:.3f}s", duration=duration)
        except Exception as e:
            self.logger.error(f"Error recording embedding generation time: {str(e)}", error=str(e))

    def record_qdrant_operation(self, operation: str, success: bool):
        """
        Record a Qdrant operation.

        Args:
            operation: Type of operation (e.g., "search", "store", "retrieve")
            success: Whether the operation was successful
        """
        try:
            status = "success" if success else "failure"
            self.increment_counter(
                "qdrant_operations_total",
                labels={
                    "operation": operation,
                    "status": status
                }
            )
            self.logger.debug(f"Recorded Qdrant operation: {operation} -> {status}",
                            operation=operation, status=status)
        except Exception as e:
            self.logger.error(f"Error recording Qdrant operation: {str(e)}", error=str(e))

    def update_crawl_job_count(self, active_jobs: int):
        """
        Update the count of active crawl jobs.

        Args:
            active_jobs: Number of currently active crawl jobs
        """
        try:
            self.set_gauge("active_crawl_jobs", active_jobs)
            self.logger.debug(f"Updated active crawl jobs count: {active_jobs}", count=active_jobs)
        except Exception as e:
            self.logger.error(f"Error updating crawl job count: {str(e)}", error=str(e))

    def record_document_processed(self, source_type: str = "web"):
        """
        Record that a document has been processed.

        Args:
            source_type: Type of source (e.g., "web", "pdf", "docx")
        """
        try:
            self.increment_counter(
                "processed_documents_total",
                labels={"source_type": source_type}
            )
            self.logger.debug(f"Recorded document processed: {source_type}", source_type=source_type)
        except Exception as e:
            self.logger.error(f"Error recording document processing: {str(e)}", error=str(e))

    def update_crawling_rate(self, pages_per_minute: float):
        """
        Update the current crawling rate.

        Args:
            pages_per_minute: Current crawling rate in pages per minute
        """
        try:
            self.set_gauge("crawling_rate_pages_per_minute", pages_per_minute)
            self.logger.debug(f"Updated crawling rate: {pages_per_minute} pages/min",
                            rate=pages_per_minute)
        except Exception as e:
            self.logger.error(f"Error updating crawling rate: {str(e)}", error=str(e))


def metrics_middleware(metrics_collector: MetricsCollector):
    """
    Create a FastAPI middleware function for metrics collection.

    Args:
        metrics_collector: MetricsCollector instance

    Returns:
        Middleware function
    """
    async def middleware(request: Request, call_next):
        start_time = time.time()

        try:
            response = await call_next(request)
        finally:
            # Record metrics regardless of whether the request succeeded
            metrics_collector.api_metrics_middleware(request, response, start_time)

        return response

    return middleware


def measure_duration(metrics_collector: MetricsCollector, metric_name: str,
                    labels: Optional[Dict[str, str]] = None):
    """
    Decorator to measure the duration of a function execution.

    Args:
        metrics_collector: MetricsCollector instance
        metric_name: Name of the histogram metric to record duration in
        labels: Labels to attach to the metric

    Returns:
        Decorator function
    """
    def decorator(func):
        if asyncio.iscoroutinefunction(func):
            @functools.wraps(func)
            async def async_wrapper(*args, **kwargs):
                start_time = time.time()
                try:
                    result = await func(*args, **kwargs)
                    return result
                finally:
                    duration = time.time() - start_time
                    metrics_collector.observe_histogram(metric_name, duration, labels)
            return async_wrapper
        else:
            @functools.wraps(func)
            def sync_wrapper(*args, **kwargs):
                start_time = time.time()
                try:
                    result = func(*args, **kwargs)
                    return result
                finally:
                    duration = time.time() - start_time
                    metrics_collector.observe_histogram(metric_name, duration, labels)
            return sync_wrapper
    return decorator


def count_calls(metrics_collector: MetricsCollector, metric_name: str,
               labels: Optional[Dict[str, str]] = None):
    """
    Decorator to count the number of times a function is called.

    Args:
        metrics_collector: MetricsCollector instance
        metric_name: Name of the counter metric to increment
        labels: Labels to attach to the metric

    Returns:
        Decorator function
    """
    def decorator(func):
        if asyncio.iscoroutinefunction(func):
            @functools.wraps(func)
            async def async_wrapper(*args, **kwargs):
                metrics_collector.increment_counter(metric_name, labels, 1.0)
                return await func(*args, **kwargs)
            return async_wrapper
        else:
            @functools.wraps(func)
            def sync_wrapper(*args, **kwargs):
                metrics_collector.increment_counter(metric_name, labels, 1.0)
                return func(*args, **kwargs)
            return sync_wrapper
    return decorator


# Global metrics collector instance
metrics_collector = MetricsCollector()


def get_metrics_collector() -> MetricsCollector:
    """
    Get the global metrics collector instance.

    Returns:
        MetricsCollector instance
    """
    return metrics_collector


# Convenience functions
def increment_counter(name: str, labels: Optional[Dict[str, str]] = None, amount: float = 1.0):
    """
    Convenience function to increment a counter metric.
    """
    metrics_collector.increment_counter(name, labels, amount)


def observe_histogram(name: str, value: float, labels: Optional[Dict[str, str]] = None):
    """
    Convenience function to observe a value in a histogram.
    """
    metrics_collector.observe_histogram(name, value, labels)


def set_gauge(name: str, value: float, labels: Optional[Dict[str, str]] = None):
    """
    Convenience function to set a gauge value.
    """
    metrics_collector.set_gauge(name, value, labels)


def observe_summary(name: str, value: float, labels: Optional[Dict[str, str]] = None):
    """
    Convenience function to observe a value in a summary.
    """
    metrics_collector.observe_summary(name, value, labels)


def get_metrics() -> str:
    """
    Convenience function to get all metrics in Prometheus format.
    """
    return metrics_collector.get_metrics()


def record_embedding_generation_time(start_time: float):
    """
    Convenience function to record embedding generation time.
    """
    metrics_collector.record_embedding_generation_time(start_time)


def record_qdrant_operation(operation: str, success: bool):
    """
    Convenience function to record Qdrant operation.
    """
    metrics_collector.record_qdrant_operation(operation, success)


def update_crawl_job_count(active_jobs: int):
    """
    Convenience function to update crawl job count.
    """
    metrics_collector.update_crawl_job_count(active_jobs)


def record_document_processed(source_type: str = "web"):
    """
    Convenience function to record document processing.
    """
    metrics_collector.record_document_processed(source_type)


def update_crawling_rate(pages_per_minute: float):
    """
    Convenience function to update crawling rate.
    """
    metrics_collector.update_crawling_rate(pages_per_minute)


# Export the metrics collector for use in other modules
__all__ = [
    'MetricsCollector',
    'MetricType',
    'metrics_middleware',
    'measure_duration',
    'count_calls',
    'get_metrics_collector',
    'increment_counter',
    'observe_histogram',
    'set_gauge',
    'observe_summary',
    'get_metrics',
    'record_embedding_generation_time',
    'record_qdrant_operation',
    'update_crawl_job_count',
    'record_document_processed',
    'update_crawling_rate',
    'metrics_collector'
]