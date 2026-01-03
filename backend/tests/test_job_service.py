"""
Unit tests for the Crawl Job service.
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from typing import List
from datetime import datetime, timedelta
from uuid import UUID

from src.services.job_service import CrawlJobService
from src.models.job import CrawlJob, JobStatus


@pytest.fixture
async def job_service():
    """Create a job service instance for testing."""
    service = CrawlJobService()
    # Override external services with mocks
    service.repository = AsyncMock()
    return service


@pytest.mark.asyncio
async def test_create_job_success(job_service):
    """Test successful job creation."""
    # Mock repository behavior
    job_service.repository.create.return_value = CrawlJob(
        id=UUID("12345678-1234-5678-1234-567812345678"),
        target_url="https://example.com",
        status=JobStatus.CREATED,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )

    target_url = "https://example.com"
    max_depth = 3
    metadata = {"test": "metadata"}

    result = await job_service.create_job(target_url, max_depth, metadata)

    # Verify the result
    assert result is not None
    assert result.target_url == target_url
    assert result.options["max_depth"] == max_depth
    assert result.metadata == metadata
    assert result.status == JobStatus.CREATED

    # Verify repository call
    job_service.repository.create.assert_called_once()


@pytest.mark.asyncio
async def test_get_job_by_id_success(job_service):
    """Test successful job retrieval by ID."""
    expected_job = CrawlJob(
        id=UUID("12345678-1234-5678-1234-567812345678"),
        target_url="https://example.com",
        status=JobStatus.PROCESSING,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )

    # Mock repository behavior
    job_service.repository.get_by_id.return_value = expected_job

    job_id = UUID("12345678-1234-5678-1234-567812345678")
    result = await job_service.get_job_by_id(job_id)

    # Verify the result
    assert result == expected_job
    job_service.repository.get_by_id.assert_called_once_with(job_id)


@pytest.mark.asyncio
async def test_get_job_by_id_not_found(job_service):
    """Test job retrieval when job doesn't exist."""
    # Mock repository to return None
    job_service.repository.get_by_id.return_value = None

    job_id = UUID("12345678-1234-5678-1234-567812345678")
    result = await job_service.get_job_by_id(job_id)

    # Should return None when not found
    assert result is None
    job_service.repository.get_by_id.assert_called_once_with(job_id)


@pytest.mark.asyncio
async def test_list_jobs_success(job_service):
    """Test successful job listing."""
    expected_jobs = [
        CrawlJob(
            id=UUID("12345678-1234-5678-1234-567812345678"),
            target_url="https://example1.com",
            status=JobStatus.COMPLETED,
            created_at=datetime.utcnow(),
            updated_at=datetime.utcnow()
        ),
        CrawlJob(
            id=UUID("87654321-4321-8765-4321-876543218765"),
            target_url="https://example2.com",
            status=JobStatus.PROCESSING,
            created_at=datetime.utcnow(),
            updated_at=datetime.utcnow()
        )
    ]

    # Mock repository behavior
    job_service.repository.list.return_value = expected_jobs

    result = await job_service.list_jobs(status=JobStatus.COMPLETED, limit=10, offset=0)

    # Verify the result
    assert result == expected_jobs
    job_service.repository.list.assert_called_once()


@pytest.mark.asyncio
async def test_list_jobs_filtered(job_service):
    """Test job listing with filters."""
    expected_jobs = [
        CrawlJob(
            id=UUID("12345678-1234-5678-1234-567812345678"),
            target_url="https://example.com",
            status=JobStatus.FAILED,
            created_at=datetime.utcnow(),
            updated_at=datetime.utcnow()
        )
    ]

    # Mock repository behavior
    job_service.repository.list.return_value = expected_jobs

    result = await job_service.list_jobs(status=JobStatus.FAILED, limit=5, offset=0)

    # Verify the result
    assert result == expected_jobs
    job_service.repository.list.assert_called_once()


@pytest.mark.asyncio
async def test_update_job_status_success(job_service):
    """Test successful job status update."""
    job_id = UUID("12345678-1234-5678-1234-567812345678")
    new_status = JobStatus.PROCESSING

    # Mock repository behavior
    job_service.repository.update.return_value = True

    result = await job_service.update_job_status(job_id, new_status)

    # Verify the result
    assert result is True
    job_service.repository.update.assert_called_once()


@pytest.mark.asyncio
async def test_update_job_status_failure(job_service):
    """Test job status update failure."""
    job_id = UUID("12345678-1234-5678-1234-567812345678")
    new_status = JobStatus.COMPLETED

    # Mock repository behavior to return False (failure)
    job_service.repository.update.return_value = False

    result = await job_service.update_job_status(job_id, new_status)

    # Verify the result
    assert result is False
    job_service.repository.update.assert_called_once()


@pytest.mark.asyncio
async def test_update_job_progress_success(job_service):
    """Test successful job progress update."""
    job_id = UUID("12345678-1234-5678-1234-567812345678")

    # Mock repository behavior
    job_service.repository.update.return_value = True

    result = await job_service.update_job_progress(job_id, 10, 2, 12)

    # Verify the result
    assert result is True
    job_service.repository.update.assert_called_once()


@pytest.mark.asyncio
async def test_update_job_completion_time_success(job_service):
    """Test successful job completion time update."""
    job_id = UUID("12345678-1234-5678-1234-567812345678")

    # Mock repository behavior
    job_service.repository.update.return_value = True

    result = await job_service.update_job_completion_time(job_id)

    # Verify the result
    assert result is True
    job_service.repository.update.assert_called_once()


@pytest.mark.asyncio
async def test_update_job_error_log_success(job_service):
    """Test successful job error log update."""
    job_id = UUID("12345678-1234-5678-1234-567812345678")
    error_message = "Test error message"

    # Mock repository behavior
    job_service.repository.update.return_value = True

    result = await job_service.update_job_error_log(job_id, error_message)

    # Verify the result
    assert result is True
    job_service.repository.update.assert_called_once()


@pytest.mark.asyncio
async def test_cancel_job_success(job_service):
    """Test successful job cancellation."""
    job_id = UUID("12345678-1234-5678-1234-567812345678")

    # Mock repository to return a job in PROCESSING status
    mock_job = CrawlJob(
        id=job_id,
        target_url="https://example.com",
        status=JobStatus.PROCESSING,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )
    job_service.repository.get_by_id.return_value = mock_job
    job_service.repository.update.return_value = True

    result = await job_service.cancel_job(job_id)

    # Verify the result
    assert result is True
    job_service.repository.update.assert_called_once()


@pytest.mark.asyncio
async def test_cancel_job_not_found(job_service):
    """Test job cancellation when job doesn't exist."""
    job_id = UUID("12345678-1234-5678-1234-567812345678")

    # Mock repository to return None (job not found)
    job_service.repository.get_by_id.return_value = None

    result = await job_service.cancel_job(job_id)

    # Should return False when job doesn't exist
    assert result is False
    job_service.repository.update.assert_not_called()


@pytest.mark.asyncio
async def test_cancel_job_already_completed(job_service):
    """Test job cancellation when job is already completed."""
    job_id = UUID("12345678-1234-5678-1234-567812345678")

    # Mock repository to return a job in COMPLETED status
    mock_job = CrawlJob(
        id=job_id,
        target_url="https://example.com",
        status=JobStatus.COMPLETED,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )
    job_service.repository.get_by_id.return_value = mock_job

    result = await job_service.cancel_job(job_id)

    # Should return False when job is already completed
    assert result is False
    job_service.repository.update.assert_not_called()


@pytest.mark.asyncio
async def test_get_job_statistics(job_service):
    """Test job statistics retrieval."""
    # Mock repository to return a list of jobs
    mock_jobs = [
        CrawlJob(id=UUID("12345678-1234-5678-1234-567812345678"), status=JobStatus.COMPLETED, created_at=datetime.utcnow()),
        CrawlJob(id=UUID("23456789-2345-6789-2345-678923456789"), status=JobStatus.PROCESSING, created_at=datetime.utcnow()),
        CrawlJob(id=UUID("34567890-3456-7890-3456-789034567890"), status=JobStatus.FAILED, created_at=datetime.utcnow()),
    ]
    job_service.repository.list.return_value = mock_jobs

    stats = await job_service.get_job_statistics()

    # Verify statistics
    assert stats["total_jobs"] == 3
    assert stats["completed_jobs"] == 1
    assert stats["processing_jobs"] == 1
    assert stats["failed_jobs"] == 1


@pytest.mark.asyncio
async def test_get_job_statistics_empty(job_service):
    """Test job statistics retrieval with no jobs."""
    # Mock repository to return empty list
    job_service.repository.list.return_value = []

    stats = await job_service.get_job_statistics()

    # Verify statistics
    assert stats["total_jobs"] == 0
    assert stats["completed_jobs"] == 0
    assert stats["processing_jobs"] == 0
    assert stats["failed_jobs"] == 0


@pytest.mark.asyncio
async def test_delete_job_success(job_service):
    """Test successful job deletion."""
    job_id = UUID("12345678-1234-5678-1234-567812345678")

    # Mock repository behavior
    job_service.repository.delete.return_value = True

    result = await job_service.delete_job(job_id)

    # Verify the result
    assert result is True
    job_service.repository.delete.assert_called_once_with(job_id)


@pytest.mark.asyncio
async def test_delete_job_failure(job_service):
    """Test job deletion failure."""
    job_id = UUID("12345678-1234-5678-1234-567812345678")

    # Mock repository behavior to return False (failure)
    job_service.repository.delete.return_value = False

    result = await job_service.delete_job(job_id)

    # Verify the result
    assert result is False
    job_service.repository.delete.assert_called_once_with(job_id)


@pytest.mark.asyncio
async def test_get_active_jobs(job_service):
    """Test retrieval of active jobs."""
    # Mock repository to return jobs with different statuses
    mock_jobs = [
        CrawlJob(id=UUID("12345678-1234-5678-1234-567812345678"), status=JobStatus.CREATED, created_at=datetime.utcnow()),
        CrawlJob(id=UUID("23456789-2345-6789-2345-678923456789"), status=JobStatus.PROCESSING, created_at=datetime.utcnow()),
        CrawlJob(id=UUID("34567890-3456-7890-3456-789034567890"), status=JobStatus.COMPLETED, created_at=datetime.utcnow()),
        CrawlJob(id=UUID("45678901-4567-8901-4567-890145678901"), status=JobStatus.FAILED, created_at=datetime.utcnow()),
    ]
    job_service.repository.list.return_value = mock_jobs

    active_jobs = await job_service.get_active_jobs()

    # Active jobs should be CREATED or PROCESSING
    assert len(active_jobs) == 2
    active_statuses = [job.status for job in active_jobs]
    assert JobStatus.CREATED in active_statuses
    assert JobStatus.PROCESSING in active_statuses
    assert JobStatus.COMPLETED not in active_statuses
    assert JobStatus.FAILED not in active_statuses


@pytest.mark.asyncio
async def test_get_recent_jobs(job_service):
    """Test retrieval of recent jobs."""
    # Mock repository to return jobs with different creation times
    now = datetime.utcnow()
    past = now - timedelta(hours=1)

    mock_jobs = [
        CrawlJob(id=UUID("12345678-1234-5678-1234-567812345678"), status=JobStatus.COMPLETED, created_at=now),
        CrawlJob(id=UUID("23456789-2345-6789-2345-678923456789"), status=JobStatus.PROCESSING, created_at=past),
    ]
    job_service.repository.list.return_value = mock_jobs

    recent_jobs = await job_service.get_recent_jobs(hours=2)

    # Should include jobs from the last 2 hours
    assert len(recent_jobs) == 2


@pytest.mark.asyncio
async def test_retry_failed_job(job_service):
    """Test retrying a failed job."""
    job_id = UUID("12345678-1234-5678-1234-567812345678")

    # Mock repository to return a failed job
    failed_job = CrawlJob(
        id=job_id,
        target_url="https://example.com",
        status=JobStatus.FAILED,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow(),
        metadata={"retry_count": 0}
    )
    job_service.repository.get_by_id.return_value = failed_job
    job_service.repository.update.return_value = True

    result = await job_service.retry_failed_job(job_id)

    # Should return a new job with reset status
    assert result is not None
    assert result.status == JobStatus.CREATED
    assert result.metadata["retry_count"] == 1
    assert result.metadata["original_job_id"] == str(job_id)


@pytest.mark.asyncio
async def test_retry_non_failed_job(job_service):
    """Test retrying a job that is not in failed status."""
    job_id = UUID("12345678-1234-5678-1234-567812345678")

    # Mock repository to return a completed job
    completed_job = CrawlJob(
        id=job_id,
        target_url="https://example.com",
        status=JobStatus.COMPLETED,
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )
    job_service.repository.get_by_id.return_value = completed_job

    with pytest.raises(ValueError, match="Cannot retry job that is not in FAILED status"):
        await job_service.retry_failed_job(job_id)