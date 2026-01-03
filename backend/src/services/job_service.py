"""
Crawl job service for the RAG Ingestion Pipeline.
Manages crawl job lifecycle, status tracking, and progress reporting.
"""

from datetime import datetime
from typing import Optional, List, Dict, Any
from uuid import UUID

from src.models.job import CrawlJob, JobStatus
from src.models.chunk import ContentChunk
from src.utils.logging import get_logger
from src.config.settings import settings


class CrawlJobService:
    """Service to manage crawl jobs and their lifecycle."""

    def __init__(self):
        self.logger = get_logger("job_service")
        # In a real implementation, this would connect to a database
        # For now, using in-memory storage for demonstration
        self._jobs: Dict[UUID, CrawlJob] = {}
        # Index for faster lookup by URL
        self._url_to_job: Dict[str, UUID] = {}

    async def create_job(self, target_url: str, max_depth: int = 3, max_workers: int = 4, force_create: bool = False) -> CrawlJob:
        """
        Create a new crawl job.

        Args:
            target_url: URL to crawl
            max_depth: Maximum depth to crawl
            max_workers: Maximum number of concurrent workers
            force_create: If True, create a new job even if one exists for the same URL

        Returns:
            Created CrawlJob object
        """
        from src.utils.url import is_valid_url, normalize_url

        # Validate and normalize the URL
        if not is_valid_url(target_url):
            raise ValueError(f"Invalid target URL: {target_url}")

        normalized_url = normalize_url(target_url)
        if not normalized_url:
            raise ValueError(f"Could not normalize target URL: {target_url}")

        # Check for existing job with the same URL to implement idempotency
        existing_job = await self.get_job_by_url(normalized_url)
        if existing_job and not force_create:
            self.logger.info(
                f"Idempotency check: Job already exists for URL {target_url}, returning existing job {existing_job.id}",
                target_url=target_url,
                existing_job_id=str(existing_job.id)
            )
            return existing_job

        # Create job options
        options = {
            "max_depth": max_depth,
            "max_workers": max_workers,
            "selectors": {
                "content": [".main-wrapper .markdown", ".theme-doc-markdown", ".doc-content"],
                "title": ["h1", ".hero__title"],
                "navigation": [".menu", ".nav", ".sidebar"]
            }
        }

        # Create the job
        job = CrawlJob(
            target_url=normalized_url,
            options=options
        )

        # Store the job
        self._jobs[job.id] = job
        # Add to URL index for idempotency checks
        self._url_to_job[job.target_url] = job.id

        self.logger.info(
            f"Created crawl job {job.id} for {job.target_url}",
            job_id=str(job.id),
            target_url=job.target_url,
            options=options
        )

        return job

    async def get_job_by_url(self, target_url: str) -> Optional[CrawlJob]:
        """
        Get a crawl job by its target URL.

        Args:
            target_url: The target URL to search for

        Returns:
            CrawlJob object if found, None otherwise
        """
        job_id = self._url_to_job.get(target_url)
        if job_id:
            return self._jobs.get(job_id)
        return None

    async def get_job_by_id(self, job_id: UUID) -> Optional[CrawlJob]:
        """
        Get a crawl job by its ID.

        Args:
            job_id: ID of the job to retrieve

        Returns:
            CrawlJob object or None if not found
        """
        job = self._jobs.get(job_id)
        if job:
            self.logger.debug(f"Retrieved job {job_id}", job_id=str(job_id))
        else:
            self.logger.warning(f"Job {job_id} not found", job_id=str(job_id))

        return job

    async def update_job_status(self, job_id: UUID, status: JobStatus) -> bool:
        """
        Update the status of a crawl job.

        Args:
            job_id: ID of the job to update
            status: New status for the job

        Returns:
            True if successful, False otherwise
        """
        job = self._jobs.get(job_id)
        if not job:
            self.logger.warning(f"Cannot update status, job {job_id} not found", job_id=str(job_id))
            return False

        old_status = job.status
        job.status = status

        self.logger.info(
            f"Updated job {job_id} status from {old_status} to {status}",
            job_id=str(job_id),
            old_status=old_status.value,
            new_status=status.value
        )

        return True

    async def update_job_progress(self, job_id: UUID, processed: int, failed: int, total: int = None) -> bool:
        """
        Update the progress of a crawl job.

        Args:
            job_id: ID of the job to update
            processed: Number of pages processed
            failed: Number of pages that failed
            total: Total number of pages (optional, will be updated if provided)

        Returns:
            True if successful, False otherwise
        """
        job = self._jobs.get(job_id)
        if not job:
            self.logger.warning(f"Cannot update progress, job {job_id} not found", job_id=str(job_id))
            return False

        job.update_progress(processed, failed, total)

        self.logger.debug(
            f"Updated job {job_id} progress: {processed} processed, {failed} failed, {job.total_pages} total",
            job_id=str(job_id),
            processed=processed,
            failed=failed,
            total=job.total_pages,
            progress=job.progress
        )

        return True

    async def update_job_error_log(self, job_id: UUID, error_message: str) -> bool:
        """
        Update the error log of a crawl job.

        Args:
            job_id: ID of the job to update
            error_message: Error message to log

        Returns:
            True if successful, False otherwise
        """
        job = self._jobs.get(job_id)
        if not job:
            self.logger.warning(f"Cannot update error log, job {job_id} not found", job_id=str(job_id))
            return False

        job.error_log = error_message
        job.updated_at = datetime.utcnow()

        self.logger.error(
            f"Updated job {job_id} error log: {error_message}",
            job_id=str(job_id),
            error_message=error_message
        )

        return True

    async def update_job_completion_time(self, job_id: UUID) -> bool:
        """
        Update the completion time of a crawl job.

        Args:
            job_id: ID of the job to update

        Returns:
            True if successful, False otherwise
        """
        job = self._jobs.get(job_id)
        if not job:
            self.logger.warning(f"Cannot update completion time, job {job_id} not found", job_id=str(job_id))
            return False

        job.completed_at = datetime.utcnow()
        job.updated_at = datetime.utcnow()

        self.logger.info(
            f"Updated job {job_id} completion time to {job.completed_at}",
            job_id=str(job_id),
            completed_at=job.completed_at.isoformat()
        )

        return True

    async def list_jobs(self, status: Optional[JobStatus] = None, limit: Optional[int] = None, offset: Optional[int] = None) -> List[CrawlJob]:
        """
        List crawl jobs with optional filtering.

        Args:
            status: Filter by job status
            limit: Maximum number of jobs to return
            offset: Number of jobs to skip

        Returns:
            List of CrawlJob objects
        """
        jobs = list(self._jobs.values())

        # Filter by status if specified
        if status:
            jobs = [job for job in jobs if job.status == status]

        # Apply offset
        if offset:
            jobs = jobs[offset:]

        # Apply limit
        if limit:
            jobs = jobs[:limit]

        self.logger.info(
            f"Listed {len(jobs)} jobs",
            total_count=len(jobs),
            filter_status=status.value if status else "all",
            limit=limit,
            offset=offset
        )

        return jobs

    async def delete_job(self, job_id: UUID) -> bool:
        """
        Delete a crawl job.

        Args:
            job_id: ID of the job to delete

        Returns:
            True if successful, False otherwise
        """
        if job_id in self._jobs:
            del self._jobs[job_id]
            self.logger.info(f"Deleted job {job_id}", job_id=str(job_id))
            return True
        else:
            self.logger.warning(f"Cannot delete job {job_id}, not found", job_id=str(job_id))
            return False

    async def cancel_job(self, job_id: UUID) -> bool:
        """
        Cancel a crawl job.

        Args:
            job_id: ID of the job to cancel

        Returns:
            True if successful, False otherwise
        """
        job = self._jobs.get(job_id)
        if not job:
            self.logger.warning(f"Cannot cancel job {job_id}, not found", job_id=str(job_id))
            return False

        if job.status in [JobStatus.COMPLETED, JobStatus.FAILED]:
            self.logger.warning(
                f"Cannot cancel job {job_id}, already in final state: {job.status}",
                job_id=str(job_id),
                status=job.status.value
            )
            return False

        old_status = job.status
        job.status = JobStatus.CANCELLED
        job.updated_at = datetime.utcnow()

        self.logger.info(
            f"Cancelled job {job_id}, was {old_status}",
            job_id=str(job_id),
            old_status=old_status.value
        )

        return True

    async def get_job_statistics(self) -> Dict[str, Any]:
        """
        Get overall statistics for all jobs.

        Returns:
            Dictionary with job statistics
        """
        stats = {
            "total_jobs": len(self._jobs),
            "status_counts": {},
            "total_processed_pages": 0,
            "total_failed_pages": 0
        }

        # Count jobs by status
        for job in self._jobs.values():
            status_str = job.status.value
            stats["status_counts"][status_str] = stats["status_counts"].get(status_str, 0) + 1

        # Sum up processed and failed pages
        for job in self._jobs.values():
            stats["total_processed_pages"] += job.processed_pages
            stats["total_failed_pages"] += job.failed_pages

        self.logger.info(
            "Retrieved job statistics",
            stats=stats
        )

        return stats


# Global job service instance
_job_service: Optional[CrawlJobService] = None


def get_job_service() -> CrawlJobService:
    """Get the global job service instance."""
    global _job_service
    if _job_service is None:
        _job_service = CrawlJobService()
    return _job_service


# Convenience functions for common operations
async def create_crawl_job(target_url: str, max_depth: int = 3, max_workers: int = 4, force_create: bool = False) -> CrawlJob:
    """Convenience function to create a crawl job."""
    service = get_job_service()
    return await service.create_job(target_url, max_depth, max_workers, force_create)


async def get_crawl_job(job_id: UUID) -> Optional[CrawlJob]:
    """Convenience function to get a crawl job."""
    service = get_job_service()
    return await service.get_job_by_id(job_id)


async def update_crawl_job_status(job_id: UUID, status: JobStatus) -> bool:
    """Convenience function to update job status."""
    service = get_job_service()
    return await service.update_job_status(job_id, status)


async def update_crawl_job_progress(job_id: UUID, processed: int, failed: int, total: int = None) -> bool:
    """Convenience function to update job progress."""
    service = get_job_service()
    return await service.update_job_progress(job_id, processed, failed, total)


async def list_crawl_jobs(status: Optional[JobStatus] = None, limit: Optional[int] = None, offset: Optional[int] = None) -> List[CrawlJob]:
    """Convenience function to list crawl jobs."""
    service = get_job_service()
    return await service.list_jobs(status, limit, offset)


async def cancel_crawl_job(job_id: UUID) -> bool:
    """Convenience function to cancel a crawl job."""
    service = get_job_service()
    return await service.cancel_job(job_id)