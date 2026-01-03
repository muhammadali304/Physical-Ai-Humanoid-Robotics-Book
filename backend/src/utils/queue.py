"""
Message queue utility for the RAG Ingestion Pipeline using Redis/RQ.
Provides task queuing and job processing capabilities.
"""

import asyncio
import json
from typing import Any, Callable, Optional, Dict
from uuid import UUID
import redis
from rq import Queue, Worker
from rq.job import Job, JobStatus
from rq.exceptions import NoSuchJobError
from src.config.settings import settings
from src.utils.logging import get_logger


class QueueManager:
    """Manages Redis/RQ queues for the application."""

    def __init__(self):
        self.redis_conn = redis.Redis(
            host=settings.redis_host,
            port=settings.redis_port,
            db=settings.redis_db,
            password=settings.redis_password,
            decode_responses=True
        )
        self.queues: Dict[str, Queue] = {}
        self.logger = get_logger("queue_manager")

    def get_queue(self, name: str = "default") -> Queue:
        """Get or create a queue with the specified name."""
        if name not in self.queues:
            self.queues[name] = Queue(name, connection=self.redis_conn)
        return self.queues[name]

    def enqueue(self, func: Callable, *args, queue_name: str = "default", **kwargs) -> Job:
        """Enqueue a function to be executed."""
        queue = self.get_queue(queue_name)
        job = queue.enqueue(func, *args, **kwargs)

        self.logger.info(
            f"Enqueued job {job.id} to queue {queue_name}",
            job_id=job.id,
            queue_name=queue_name,
            function=func.__name__
        )

        return job

    def get_job(self, job_id: str) -> Optional[Job]:
        """Get a job by its ID."""
        try:
            job = Job.fetch(job_id, connection=self.redis_conn)
            return job
        except NoSuchJobError:
            self.logger.warning(f"Job {job_id} not found")
            return None

    def job_status(self, job_id: str) -> Optional[str]:
        """Get the status of a job."""
        job = self.get_job(job_id)
        if job:
            return job.get_status()
        return None

    def job_result(self, job_id: str) -> Optional[Any]:
        """Get the result of a completed job."""
        job = self.get_job(job_id)
        if job and job.get_status() == JobStatus.FINISHED:
            return job.result
        return None

    def cancel_job(self, job_id: str) -> bool:
        """Cancel a queued or running job."""
        try:
            job = Job.fetch(job_id, connection=self.redis_conn)
            job.cancel()
            self.logger.info(f"Cancelled job {job_id}")
            return True
        except NoSuchJobError:
            self.logger.warning(f"Cannot cancel job {job_id}, not found")
            return False

    def clean_queue(self, queue_name: str = "default") -> int:
        """Remove all jobs from a queue."""
        queue = self.get_queue(queue_name)
        count = len(queue)
        queue.empty()
        self.logger.info(f"Cleaned {count} jobs from queue {queue_name}")
        return count

    def queue_length(self, queue_name: str = "default") -> int:
        """Get the length of a queue."""
        queue = self.get_queue(queue_name)
        return len(queue)

    def get_failed_jobs(self, queue_name: str = "default") -> list:
        """Get all failed jobs in a queue."""
        queue = self.get_queue(queue_name)
        failed_queue = queue.failed_job_registry
        return [job.id for job in failed_queue]


class CrawlJobQueue:
    """Specialized queue for managing crawl jobs."""

    def __init__(self, queue_manager: QueueManager):
        self.queue_manager = queue_manager
        self.logger = get_logger("crawl_job_queue")

    def enqueue_crawl_job(self, crawl_job_id: UUID, target_url: str, options: Dict[str, Any]) -> str:
        """Enqueue a crawl job."""
        # Import here to avoid circular dependencies
        from src.services.crawler import CrawlerService

        # Prepare job data
        job_data = {
            'crawl_job_id': str(crawl_job_id),
            'target_url': target_url,
            'options': options
        }

        # Enqueue the crawl job
        job = self.queue_manager.enqueue(
            CrawlerService.process_crawl_job,
            job_data,
            queue_name="crawling"
        )

        self.logger.info(
            f"Enqueued crawl job {crawl_job_id} for URL {target_url}",
            crawl_job_id=str(crawl_job_id),
            job_id=job.id,
            target_url=target_url
        )

        return job.id

    def enqueue_chunk_job(self, content_chunk: Dict[str, Any]) -> str:
        """Enqueue a chunk processing job."""
        from src.services.chunker import ChunkerService

        job = self.queue_manager.enqueue(
            ChunkerService.process_content_chunk,
            content_chunk,
            queue_name="chunking"
        )

        self.logger.info(
            f"Enqueued chunk job for URL: {content_chunk.get('source_url', 'unknown')}",
            job_id=job.id,
            source_url=content_chunk.get('source_url', 'unknown')
        )

        return job.id

    def enqueue_embedding_job(self, chunk_id: UUID) -> str:
        """Enqueue an embedding generation job."""
        from src.services.embedding_service import EmbeddingService

        job = self.queue_manager.enqueue(
            EmbeddingService.generate_embedding_for_chunk,
            str(chunk_id),
            queue_name="embedding"
        )

        self.logger.info(
            f"Enqueued embedding job for chunk {chunk_id}",
            job_id=job.id,
            chunk_id=str(chunk_id)
        )

        return job.id


class QueueWorker:
    """Manages a worker that processes jobs from queues."""

    def __init__(self, queue_names: list = None):
        if queue_names is None:
            queue_names = ["default", "crawling", "chunking", "embedding"]
        self.queue_names = queue_names
        self.redis_conn = redis.Redis(
            host=settings.redis_host,
            port=settings.redis_port,
            db=settings.redis_db,
            password=settings.redis_password,
            decode_responses=True
        )
        self.worker = Worker(self.queue_names, connection=self.redis_conn)
        self.logger = get_logger("queue_worker")

    def start(self):
        """Start the worker to process jobs."""
        self.logger.info(
            f"Starting worker for queues: {self.queue_names}",
            queues=self.queue_names
        )
        self.worker.work()

    async def start_async(self):
        """Start the worker asynchronously."""
        # Note: RQ workers are synchronous, so we can't truly make them async
        # This is a wrapper that would allow integration with async code
        self.logger.info(
            f"Starting async worker for queues: {self.queue_names}",
            queues=self.queue_names
        )
        self.worker.work(burst=True)  # Process available jobs then exit


# Global queue manager instance
_queue_manager: Optional[QueueManager] = None


def get_queue_manager() -> QueueManager:
    """Get the global queue manager instance."""
    global _queue_manager
    if _queue_manager is None:
        _queue_manager = QueueManager()
    return _queue_manager


def get_crawl_job_queue() -> CrawlJobQueue:
    """Get the crawl job queue instance."""
    queue_manager = get_queue_manager()
    return CrawlJobQueue(queue_manager)


# Convenience functions for common operations
def enqueue_job(func: Callable, *args, queue_name: str = "default", **kwargs) -> str:
    """Convenience function to enqueue a job."""
    queue_manager = get_queue_manager()
    job = queue_manager.enqueue(func, *args, queue_name=queue_name, **kwargs)
    return job.id


def get_job_status(job_id: str) -> Optional[str]:
    """Convenience function to get job status."""
    queue_manager = get_queue_manager()
    return queue_manager.job_status(job_id)


def get_job_result(job_id: str) -> Optional[Any]:
    """Convenience function to get job result."""
    queue_manager = get_queue_manager()
    return queue_manager.job_result(job_id)