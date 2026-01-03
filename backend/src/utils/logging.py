"""
Structured logging utility for the RAG Ingestion Pipeline.
Provides consistent logging across the application with structured formats.
"""

import logging
import json
import sys
from datetime import datetime
from typing import Any, Dict, Optional
from enum import Enum
import traceback

from src.config.settings import settings


class LogType(Enum):
    """Types of log messages for structured logging."""
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    DEBUG = "debug"
    CRITICAL = "critical"
    METRIC = "metric"
    EVENT = "event"
    AUDIT = "audit"
    SECURITY = "security"


class StructuredLogger:
    """Structured logger that outputs JSON-formatted logs."""

    def __init__(self, name: str, log_level: Optional[str] = None):
        self.name = name
        self.logger = logging.getLogger(name)

        # Set log level from settings or parameter
        level = log_level or settings.log_level
        self.logger.setLevel(getattr(logging, level.upper()))

        # Prevent adding handlers multiple times
        if not self.logger.handlers:
            # Add console handler
            console_handler = logging.StreamHandler(sys.stdout)
            if settings.log_json_format:
                # For JSON format, we'll handle formatting in _log method
                console_handler.setFormatter(logging.Formatter('%(message)s'))
            else:
                console_handler.setFormatter(logging.Formatter(settings.log_format))
            self.logger.addHandler(console_handler)

            # Add file handler if specified
            if settings.log_file:
                from logging.handlers import RotatingFileHandler
                file_handler = RotatingFileHandler(
                    settings.log_file,
                    maxBytes=settings.log_max_bytes,
                    backupCount=settings.log_backup_count
                )
                if settings.log_json_format:
                    file_handler.setFormatter(logging.Formatter('%(message)s'))
                else:
                    file_handler.setFormatter(logging.Formatter(settings.log_format))
                self.logger.addHandler(file_handler)

            # Prevent propagation to avoid duplicate logs
            self.logger.propagate = False

    def _log(self, log_type: LogType, message: str, **kwargs):
        """Internal method to create structured log entries."""
        log_entry = {
            "timestamp": datetime.utcnow().isoformat(),
            "level": log_type.value.upper(),
            "logger": self.name,
            "message": message,
            "context": kwargs
        }

        # Add stack trace for error and critical logs
        if log_type in [LogType.ERROR, LogType.CRITICAL] and settings.log_include_traceback:
            log_entry["stack_trace"] = traceback.format_stack()

        # Output as JSON string
        log_json = json.dumps(log_entry, default=str)

        # Log to the underlying logger
        # Use appropriate standard logging method based on log type
        if log_type == LogType.METRIC:
            self.logger.info(log_json)  # Metrics are logged as info
        elif log_type == LogType.EVENT:
            self.logger.info(log_json)  # Events are logged as info
        elif log_type == LogType.AUDIT:
            self.logger.info(log_json)  # Audit logs are logged as info
        elif log_type == LogType.SECURITY:
            self.logger.warning(log_json)  # Security events are logged as warnings
        else:
            getattr(self.logger, log_type.value)(log_json)

    def info(self, message: str, **kwargs):
        """Log an info message."""
        self._log(LogType.INFO, message, **kwargs)

    def warning(self, message: str, **kwargs):
        """Log a warning message."""
        self._log(LogType.WARNING, message, **kwargs)

    def error(self, message: str, **kwargs):
        """Log an error message."""
        self._log(LogType.ERROR, message, **kwargs)

    def debug(self, message: str, **kwargs):
        """Log a debug message."""
        self._log(LogType.DEBUG, message, **kwargs)

    def critical(self, message: str, **kwargs):
        """Log a critical message."""
        self._log(LogType.CRITICAL, message, **kwargs)

    def metric(self, name: str, value: Any, **kwargs):
        """Log a metric."""
        self._log(LogType.METRIC, f"Metric: {name}={value}", metric_name=name, metric_value=value, **kwargs)

    def event(self, event_name: str, **kwargs):
        """Log an event."""
        self._log(LogType.EVENT, f"Event: {event_name}", event_name=event_name, **kwargs)

    def audit(self, action: str, user: str = None, resource: str = None, **kwargs):
        """Log an audit event."""
        self._log(
            LogType.AUDIT,
            f"Audit: {action}",
            action=action,
            user=user,
            resource=resource,
            **kwargs
        )

    def security(self, event_type: str, severity: str = "medium", **kwargs):
        """Log a security event."""
        self._log(
            LogType.SECURITY,
            f"Security: {event_type}",
            event_type=event_type,
            severity=severity,
            **kwargs
        )

    def request(self, method: str, path: str, status_code: int, duration: float, **kwargs):
        """Log an HTTP request."""
        self._log(
            LogType.EVENT,
            f"Request: {method} {path} -> {status_code}",
            method=method,
            path=path,
            status_code=status_code,
            duration=duration,
            **kwargs
        )

    def database(self, operation: str, table: str, duration: float = None, **kwargs):
        """Log a database operation."""
        self._log(
            LogType.EVENT,
            f"Database: {operation} on {table}",
            operation=operation,
            table=table,
            duration=duration,
            **kwargs
        )

    def api_call(self, service: str, endpoint: str, status: str, duration: float = None, **kwargs):
        """Log an external API call."""
        self._log(
            LogType.EVENT,
            f"API Call: {service} -> {endpoint}",
            service=service,
            endpoint=endpoint,
            status=status,
            duration=duration,
            **kwargs
        )


# Global logger instances
def get_logger(name: str) -> StructuredLogger:
    """Get a structured logger instance."""
    return StructuredLogger(name)


# Convenience functions for common loggers
def get_crawler_logger() -> StructuredLogger:
    """Get a logger for crawler operations."""
    return get_logger("crawler")


def get_chunker_logger() -> StructuredLogger:
    """Get a logger for chunking operations."""
    return get_logger("chunker")


def get_embedding_logger() -> StructuredLogger:
    """Get a logger for embedding operations."""
    return get_logger("embedding")


def get_api_logger() -> StructuredLogger:
    """Get a logger for API operations."""
    return get_logger("api")


def get_queue_logger() -> StructuredLogger:
    """Get a logger for queue operations."""
    return get_logger("queue")


# Context manager for performance logging
class PerformanceLogger:
    """Context manager for logging performance metrics."""

    def __init__(self, operation_name: str, logger: StructuredLogger):
        self.operation_name = operation_name
        self.logger = logger
        self.start_time = None

    def __enter__(self):
        self.start_time = datetime.utcnow()
        self.logger.info(f"Starting {self.operation_name}")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        duration = (datetime.utcnow() - self.start_time).total_seconds()

        if exc_type is None:
            self.logger.metric(
                f"{self.operation_name}_duration",
                duration,
                operation=self.operation_name,
                status="success"
            )
            self.logger.info(f"Completed {self.operation_name} in {duration:.2f}s")
        else:
            self.logger.error(
                f"Failed {self.operation_name} after {duration:.2f}s",
                error_type=exc_type.__name__,
                error_message=str(exc_val)
            )
            self.logger.metric(
                f"{self.operation_name}_duration",
                duration,
                operation=self.operation_name,
                status="failed"
            )


import asyncio
from functools import wraps

def log_performance(operation_name: str):
    """Decorator for logging performance of functions."""
    def decorator(func):
        if asyncio.iscoroutinefunction(func):
            @wraps(func)
            async def async_wrapper(*args, **kwargs):
                logger = get_logger(func.__module__)
                perf_logger = PerformanceLogger(operation_name, logger)

                start_time = datetime.utcnow()
                try:
                    result = await func(*args, **kwargs)
                    duration = (datetime.utcnow() - start_time).total_seconds()

                    logger.metric(
                        f"{operation_name}_duration",
                        duration,
                        operation=operation_name,
                        status="success"
                    )
                    logger.info(f"Completed {operation_name} in {duration:.2f}s")

                    return result
                except Exception as e:
                    duration = (datetime.utcnow() - start_time).total_seconds()

                    logger.error(
                        f"Failed {operation_name} after {duration:.2f}s",
                        error_type=type(e).__name__,
                        error_message=str(e)
                    )
                    logger.metric(
                        f"{operation_name}_duration",
                        duration,
                        operation=operation_name,
                        status="failed"
                    )
                    raise
            return async_wrapper
        else:
            @wraps(func)
            def sync_wrapper(*args, **kwargs):
                logger = get_logger(func.__module__)
                perf_logger = PerformanceLogger(operation_name, logger)

                with perf_logger:
                    return func(*args, **kwargs)
            return sync_wrapper
    return decorator