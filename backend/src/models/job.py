from pydantic import BaseModel, Field, validator
from typing import Optional, Dict, Any
from datetime import datetime
from uuid import UUID, uuid4
from enum import Enum


class JobStatus(str, Enum):
    PENDING = "pending"
    PROCESSING = "processing"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class CrawlJob(BaseModel):
    """
    Represents a crawling operation that processes a target website and creates content chunks.
    """
    id: UUID = Field(default_factory=uuid4)
    target_url: str = Field(..., description="Root URL to crawl")
    status: JobStatus = Field(default=JobStatus.PENDING, description="Current status of the job")
    progress: int = Field(default=0, ge=0, le=100, description="Percentage completion of the job")
    total_pages: int = Field(default=0, description="Total number of pages identified to process")
    processed_pages: int = Field(default=0, description="Number of pages processed so far")
    failed_pages: int = Field(default=0, description="Number of pages that failed processing")
    options: Dict[str, Any] = Field(default_factory=dict, description="Configuration options for the crawl (selectors, filters, etc.)")
    error_log: Optional[str] = Field(None, description="Details about errors encountered during the crawl")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Timestamp when the job was created")
    updated_at: datetime = Field(default_factory=datetime.utcnow, description="Timestamp when the job was last updated")
    completed_at: Optional[datetime] = Field(None, description="Timestamp when the job was completed")

    @validator('target_url')
    def validate_target_url(cls, v):
        """Validate that the target URL is properly formatted."""
        if not v or not v.startswith(('http://', 'https://')):
            raise ValueError('target_url must be a valid URL starting with http:// or https://')
        return v

    @validator('progress')
    def validate_progress(cls, v):
        """Validate that progress is between 0 and 100."""
        if v < 0 or v > 100:
            raise ValueError('Progress must be between 0 and 100')
        return v

    @validator('status')
    def validate_status(cls, v):
        """Validate that status is one of the defined enum values."""
        if v not in JobStatus.__members__.values():
            raise ValueError(f'Status must be one of: {list(JobStatus.__members__.values())}')
        return v

    def __init__(self, **data):
        super().__init__(**data)
        self.updated_at = datetime.utcnow()

    def update_progress(self, processed: int, failed: int, total: int = None):
        """Update job progress based on processed and failed pages."""
        self.processed_pages = processed
        self.failed_pages = failed
        if total is not None:
            self.total_pages = total

        if total and total > 0:
            self.progress = min(100, int((processed / total) * 100))
        else:
            self.progress = 0

        self.updated_at = datetime.utcnow()