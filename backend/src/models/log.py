from pydantic import BaseModel, Field
from typing import Optional, Dict, Any
from datetime import datetime
from uuid import UUID, uuid4
from enum import Enum


class LogLevel(str, Enum):
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    DEBUG = "debug"


class ProcessingLog(BaseModel):
    """
    Represents logs of processing operations for debugging and monitoring purposes.
    """
    id: UUID = Field(default_factory=uuid4)
    job_id: UUID = Field(..., description="Reference to the associated CrawlJob")
    chunk_id: Optional[UUID] = Field(None, description="Reference to the associated ContentChunk if applicable")
    level: LogLevel = Field(..., description="Severity level of the log")
    message: str = Field(..., description="Log message content")
    context: Dict[str, Any] = Field(default_factory=dict, description="Additional context information about the log event")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Timestamp when the log was created")

    def __init__(self, **data):
        super().__init__(**data)
        self.created_at = datetime.utcnow()

    @classmethod
    def info(cls, job_id: UUID, message: str, chunk_id: Optional[UUID] = None, context: Dict[str, Any] = None):
        """Create an info level log entry."""
        return cls(
            job_id=job_id,
            chunk_id=chunk_id,
            level=LogLevel.INFO,
            message=message,
            context=context or {}
        )

    @classmethod
    def warning(cls, job_id: UUID, message: str, chunk_id: Optional[UUID] = None, context: Dict[str, Any] = None):
        """Create a warning level log entry."""
        return cls(
            job_id=job_id,
            chunk_id=chunk_id,
            level=LogLevel.WARNING,
            message=message,
            context=context or {}
        )

    @classmethod
    def error(cls, job_id: UUID, message: str, chunk_id: Optional[UUID] = None, context: Dict[str, Any] = None):
        """Create an error level log entry."""
        return cls(
            job_id=job_id,
            chunk_id=chunk_id,
            level=LogLevel.ERROR,
            message=message,
            context=context or {}
        )

    @classmethod
    def debug(cls, job_id: UUID, message: str, chunk_id: Optional[UUID] = None, context: Dict[str, Any] = None):
        """Create a debug level log entry."""
        return cls(
            job_id=job_id,
            chunk_id=chunk_id,
            level=LogLevel.DEBUG,
            message=message,
            context=context or {}
        )