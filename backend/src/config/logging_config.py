"""
Centralized logging configuration for the RAG Ingestion Pipeline.
Provides consistent logging configuration across the application.
"""

import logging
import logging.config
import json
import sys
from datetime import datetime
from typing import Dict, Any, Optional

from src.config.settings import settings


def setup_logging():
    """
    Set up centralized logging configuration based on settings.
    """
    if settings.log_json_format:
        # Configure JSON logging
        configure_json_logging()
    else:
        # Configure standard logging
        configure_standard_logging()


def configure_json_logging():
    """
    Configure JSON-formatted logging for structured output.
    """
    class JSONFormatter(logging.Formatter):
        """
        Custom formatter to output logs in JSON format.
        """
        def format(self, record):
            log_entry = {
                "timestamp": datetime.utcnow().isoformat(),
                "level": record.levelname,
                "logger": record.name,
                "message": record.getMessage(),
                "module": record.module,
                "function": record.funcName,
                "line": record.lineno,
            }

            # Add exception info if present
            if record.exc_info:
                log_entry["exception"] = self.formatException(record.exc_info)

            # Add extra fields if present
            if hasattr(record, 'context'):
                log_entry["context"] = record.context

            # Add any extra fields
            for key, value in record.__dict__.items():
                if key not in ['name', 'msg', 'args', 'levelname', 'levelno', 'pathname',
                              'filename', 'module', 'lineno', 'funcName', 'created',
                              'msecs', 'relativeCreated', 'thread', 'threadName',
                              'processName', 'process', 'getMessage', 'exc_info',
                              'exc_text', 'stack_info', 'context']:
                    log_entry[key] = value

            return json.dumps(log_entry, default=str)

    # Create formatter
    json_formatter = JSONFormatter()

    # Create handlers
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(json_formatter)

    handlers = [console_handler]

    # Add file handler if specified
    if settings.log_file:
        from logging.handlers import RotatingFileHandler
        file_handler = RotatingFileHandler(
            settings.log_file,
            maxBytes=settings.log_max_bytes,
            backupCount=settings.log_backup_count
        )
        file_handler.setFormatter(json_formatter)
        handlers.append(file_handler)

    # Configure root logger
    logging.basicConfig(
        level=getattr(logging, settings.log_level.upper()),
        handlers=handlers,
        force=True  # Override any existing configuration
    )


def configure_standard_logging():
    """
    Configure standard logging with custom format.
    """
    # Standard formatter
    standard_formatter = logging.Formatter(settings.log_format)

    # Create handlers
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(standard_formatter)

    handlers = [console_handler]

    # Add file handler if specified
    if settings.log_file:
        from logging.handlers import RotatingFileHandler
        file_handler = RotatingFileHandler(
            settings.log_file,
            maxBytes=settings.log_max_bytes,
            backupCount=settings.log_backup_count
        )
        file_handler.setFormatter(standard_formatter)
        handlers.append(file_handler)

    # Configure root logger
    logging.basicConfig(
        level=getattr(logging, settings.log_level.upper()),
        handlers=handlers,
        force=True  # Override any existing configuration
    )


def get_structured_logger(name: str) -> logging.Logger:
    """
    Get a structured logger with the specified name.

    Args:
        name: Name of the logger

    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name)
    logger.setLevel(getattr(logging, settings.log_level.upper()))
    return logger


def log_api_request(logger: logging.Logger, method: str, path: str, status_code: int,
                   duration: float, client_ip: str = None, **kwargs):
    """
    Log an API request with structured format.

    Args:
        logger: Logger instance to use
        method: HTTP method
        path: Request path
        status_code: Response status code
        duration: Request duration in seconds
        client_ip: Client IP address
        **kwargs: Additional context information
    """
    logger.info(
        "API Request",
        extra={
            "context": {
                "event_type": "api_request",
                "method": method,
                "path": path,
                "status_code": status_code,
                "duration_ms": round(duration * 1000, 2),
                "client_ip": client_ip,
                **kwargs
            }
        }
    )


def log_database_operation(logger: logging.Logger, operation: str, table: str,
                         duration: float, rows_affected: int = None, **kwargs):
    """
    Log a database operation with structured format.

    Args:
        logger: Logger instance to use
        operation: Type of database operation (SELECT, INSERT, UPDATE, DELETE)
        table: Table name
        duration: Operation duration in seconds
        rows_affected: Number of rows affected (for write operations)
        **kwargs: Additional context information
    """
    logger.info(
        "Database Operation",
        extra={
            "context": {
                "event_type": "database_operation",
                "operation": operation,
                "table": table,
                "duration_ms": round(duration * 1000, 2),
                "rows_affected": rows_affected,
                **kwargs
            }
        }
    )


def log_external_api_call(logger: logging.Logger, service: str, endpoint: str,
                        status: str, duration: float, **kwargs):
    """
    Log an external API call with structured format.

    Args:
        logger: Logger instance to use
        service: External service name
        endpoint: API endpoint called
        status: Status of the call (success, error, timeout, etc.)
        duration: Call duration in seconds
        **kwargs: Additional context information
    """
    logger.info(
        "External API Call",
        extra={
            "context": {
                "event_type": "external_api_call",
                "service": service,
                "endpoint": endpoint,
                "status": status,
                "duration_ms": round(duration * 1000, 2),
                **kwargs
            }
        }
    )


def log_performance_metric(logger: logging.Logger, metric_name: str, value: Any,
                         unit: str = None, **kwargs):
    """
    Log a performance metric with structured format.

    Args:
        logger: Logger instance to use
        metric_name: Name of the metric
        value: Metric value
        unit: Unit of measurement
        **kwargs: Additional context information
    """
    logger.info(
        "Performance Metric",
        extra={
            "context": {
                "event_type": "performance_metric",
                "metric_name": metric_name,
                "metric_value": value,
                "unit": unit,
                **kwargs
            }
        }
    )


def log_security_event(logger: logging.Logger, event_type: str, severity: str = "medium",
                      **kwargs):
    """
    Log a security event with structured format.

    Args:
        logger: Logger instance to use
        event_type: Type of security event
        severity: Severity level (low, medium, high, critical)
        **kwargs: Additional context information
    """
    logger.warning(
        f"Security Event: {event_type}",
        extra={
            "context": {
                "event_type": "security_event",
                "security_event_type": event_type,
                "severity": severity,
                **kwargs
            }
        }
    )


def log_audit_event(logger: logging.Logger, action: str, user: str = None,
                   resource: str = None, **kwargs):
    """
    Log an audit event with structured format.

    Args:
        logger: Logger instance to use
        action: Action performed
        user: User who performed the action
        resource: Resource affected by the action
        **kwargs: Additional context information
    """
    logger.info(
        f"Audit Event: {action}",
        extra={
            "context": {
                "event_type": "audit_event",
                "action": action,
                "user": user,
                "resource": resource,
                **kwargs
            }
        }
    )


# Initialize logging configuration when module is imported
setup_logging()


# Example usage functions
def example_usage():
    """
    Example usage of the logging configuration.
    """
    # Get a logger
    logger = get_structured_logger("example_service")

    # Log different types of events
    log_api_request(logger, "GET", "/api/v1/search", 200, 0.123, "192.168.1.1")
    log_database_operation(logger, "SELECT", "documents", 0.045, rows_affected=10)
    log_external_api_call(logger, "Cohere", "/embed", "success", 1.234)
    log_performance_metric(logger, "request_duration", 123.45, "milliseconds")
    log_security_event(logger, "unauthorized_access", "high", user_id="12345")
    log_audit_event(logger, "document_access", "user@example.com", "doc_123")


if __name__ == "__main__":
    example_usage()