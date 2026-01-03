"""
Queue monitoring service for the RAG Ingestion Pipeline.
Implements dead letter queue functionality and queue monitoring.
"""

import time
from typing import List, Dict, Any, Optional
from datetime import datetime, timedelta
import logging

from rq import get_failed_queue, Retry, Worker
from rq.job import Job, JobStatus
from redis import Redis

from src.config.redis_config import create_default_redis_connection
from src.services.event_publisher import EventPublisher
from src.utils.logging import get_logger


class QueueMonitorService:
    """
    Service to monitor RQ queues and handle failed jobs with dead letter queue functionality.
    """

    def __init__(self, redis_conn: Redis = None):
        self.logger = get_logger("queue_monitor")
        self.redis_conn = redis_conn or create_default_redis_connection()
        self.event_publisher = EventPublisher()
        self.failed_queue = get_failed_queue(connection=self.redis_conn)

    def get_failed_jobs(self) -> List[Job]:
        """
        Get all failed jobs from the failed queue.

        Returns:
            List of failed Job objects
        """
        try:
            failed_jobs = self.failed_queue.jobs
            self.logger.info(f"Found {len(failed_jobs)} failed jobs", failed_count=len(failed_jobs))
            return failed_jobs
        except Exception as e:
            self.logger.error(f"Error getting failed jobs: {str(e)}", error=str(e))
            return []

    def retry_failed_job(self, job_id: str, max_retries: int = 3) -> bool:
        """
        Retry a specific failed job.

        Args:
            job_id: ID of the job to retry
            max_retries: Maximum number of retry attempts

        Returns:
            True if retry was successful, False otherwise
        """
        try:
            # Get the failed job
            failed_job = Job.fetch(job_id, connection=self.redis_conn)

            if failed_job.status != JobStatus.FAILED:
                self.logger.warning(f"Job {job_id} is not in failed state", job_id=job_id, status=failed_job.status)
                return False

            # Check if we've exceeded retry attempts
            if hasattr(failed_job, 'retries_left'):
                if failed_job.retries_left is not None and failed_job.retries_left <= 0:
                    self.logger.info(
                        f"Job {job_id} has exceeded maximum retry attempts",
                        job_id=job_id,
                        max_retries=max_retries
                    )
                    return False
            else:
                # Track retry attempts manually if not already tracked
                retry_count = failed_job.meta.get('retry_count', 0)
                if retry_count >= max_retries:
                    self.logger.info(
                        f"Job {job_id} has exceeded maximum retry attempts ({max_retries})",
                        job_id=job_id,
                        retry_count=retry_count
                    )
                    return False

            # Retry the job
            failed_job.requeue()

            # Update retry count
            retry_count = failed_job.meta.get('retry_count', 0) + 1
            failed_job.meta['retry_count'] = retry_count
            failed_job.save()

            self.logger.info(
                f"Requeued failed job {job_id} (attempt {retry_count}/{max_retries})",
                job_id=job_id,
                retry_count=retry_count,
                max_retries=max_retries
            )

            # Publish event for job retry
            self.event_publisher.publish_event_payload({
                "event_type": "job_retry",
                "source": "queue_monitor",
                "job_id": job_id,
                "retry_count": retry_count,
                "timestamp": datetime.utcnow().isoformat(),
                "data": {
                    "original_exc": str(failed_job.exc_info) if failed_job.exc_info else None
                }
            })

            return True

        except Exception as e:
            self.logger.error(f"Error retrying failed job {job_id}: {str(e)}", job_id=job_id, error=str(e))
            return False

    def move_job_to_dead_letter(self, job_id: str, reason: str = "Manual move to dead letter") -> bool:
        """
        Move a failed job to dead letter queue (remove from failed queue).

        Args:
            job_id: ID of the job to move to dead letter
            reason: Reason for moving to dead letter queue

        Returns:
            True if successful, False otherwise
        """
        try:
            # Get the failed job
            failed_job = Job.fetch(job_id, connection=self.redis_conn)

            if failed_job.status != JobStatus.FAILED:
                self.logger.warning(f"Job {job_id} is not in failed state", job_id=job_id, status=failed_job.status)
                return False

            # Remove the job from the failed queue
            self.failed_queue.remove(failed_job)

            self.logger.info(
                f"Moved job {job_id} to dead letter queue",
                job_id=job_id,
                reason=reason
            )

            # Publish event for dead letter move
            self.event_publisher.publish_event_payload({
                "event_type": "job_moved_to_dead_letter",
                "source": "queue_monitor",
                "job_id": job_id,
                "reason": reason,
                "timestamp": datetime.utcnow().isoformat(),
                "data": {
                    "original_exc": str(failed_job.exc_info) if failed_job.exc_info else None,
                    "enqueued_at": failed_job.enqueued_at.isoformat() if failed_job.enqueued_at else None,
                    "failed_at": failed_job.failed_at.isoformat() if failed_job.failed_at else None
                }
            })

            return True

        except Exception as e:
            self.logger.error(f"Error moving job {job_id} to dead letter: {str(e)}", job_id=job_id, error=str(e))
            return False

    def retry_all_failed_jobs(self, max_retries: int = 3) -> Dict[str, Any]:
        """
        Retry all failed jobs.

        Args:
            max_retries: Maximum number of retry attempts per job

        Returns:
            Dictionary with retry results
        """
        failed_jobs = self.get_failed_jobs()
        results = {
            "total_failed": len(failed_jobs),
            "retried": 0,
            "failed_to_retry": 0,
            "already_retried": 0
        }

        for job in failed_jobs:
            try:
                # Check if job has already been retried maximum times
                retry_count = job.meta.get('retry_count', 0)
                if retry_count >= max_retries:
                    results["already_retried"] += 1
                    continue

                success = self.retry_failed_job(job.id, max_retries)
                if success:
                    results["retried"] += 1
                else:
                    results["failed_to_retry"] += 1

            except Exception as e:
                self.logger.error(f"Error processing failed job {job.id}: {str(e)}", job_id=job.id, error=str(e))
                results["failed_to_retry"] += 1

        self.logger.info(
            f"Retry all failed jobs completed",
            total_failed=results["total_failed"],
            retried=results["retried"],
            failed_to_retry=results["failed_to_retry"],
            already_retried=results["already_retried"]
        )

        return results

    def clear_failed_queue(self) -> bool:
        """
        Clear all failed jobs from the failed queue.

        Returns:
            True if successful, False otherwise
        """
        try:
            failed_jobs = self.get_failed_jobs()
            for job in failed_jobs:
                self.failed_queue.remove(job)
                self.logger.debug(f"Removed job {job.id} from failed queue", job_id=job.id)

            self.logger.info(f"Cleared {len(failed_jobs)} failed jobs", cleared_count=len(failed_jobs))
            return True

        except Exception as e:
            self.logger.error(f"Error clearing failed queue: {str(e)}", error=str(e))
            return False

    def get_queue_stats(self) -> Dict[str, Any]:
        """
        Get statistics for all queues.

        Returns:
            Dictionary with queue statistics
        """
        try:
            from rq import Queue

            # Get all queues
            queues = Queue.all(connection=self.redis_conn)

            stats = {
                "timestamp": datetime.utcnow().isoformat(),
                "queues": {},
                "total_jobs": 0,
                "total_failed_jobs": len(self.failed_queue.jobs),
                "workers": {}
            }

            # Collect stats for each queue
            for queue in queues:
                queue_stats = {
                    "name": queue.name,
                    "count": queue.count,
                    "failed_count": queue.failed_job_registry.count,
                    "started_count": queue.started_job_registry.count,
                    "deferred_count": queue.deferred_job_registry.count,
                    "finished_count": queue.finished_job_registry.count,
                    "scheduled_count": queue.scheduled_job_registry.count if hasattr(queue, 'scheduled_job_registry') else 0
                }
                stats["queues"][queue.name] = queue_stats
                stats["total_jobs"] += queue.count

            # Get worker stats
            workers = Worker.all(connection=self.redis_conn)
            for worker in workers:
                worker_stats = {
                    "name": worker.name,
                    "hostname": worker.hostname,
                    "pid": worker.pid,
                    "queues": [q.name for q in worker.queues],
                    "state": worker.state,
                    "birth_date": worker.birth_date.isoformat() if worker.birth_date else None,
                    "last_heartbeat": worker.last_heartbeat.isoformat() if worker.last_heartbeat else None,
                    "successful_job_count": worker.successful_job_count,
                    "failed_job_count": worker.failed_job_count
                }
                stats["workers"][worker.name] = worker_stats

            self.logger.debug("Collected queue statistics", stats=stats)
            return stats

        except Exception as e:
            self.logger.error(f"Error getting queue stats: {str(e)}", error=str(e))
            return {}

    def monitor_and_handle_failed_jobs(self, max_retries: int = 3, auto_retry: bool = True) -> Dict[str, Any]:
        """
        Monitor failed jobs and handle them according to configuration.

        Args:
            max_retries: Maximum number of retry attempts
            auto_retry: Whether to automatically retry failed jobs

        Returns:
            Dictionary with monitoring results
        """
        results = {
            "checked_jobs": 0,
            "retried": 0,
            "moved_to_dead_letter": 0,
            "errors": 0
        }

        failed_jobs = self.get_failed_jobs()
        results["checked_jobs"] = len(failed_jobs)

        for job in failed_jobs:
            try:
                # Get retry count
                retry_count = job.meta.get('retry_count', 0)

                if auto_retry and retry_count < max_retries:
                    # Retry the job
                    success = self.retry_failed_job(job.id, max_retries)
                    if success:
                        results["retried"] += 1
                    else:
                        results["errors"] += 1
                else:
                    # Move to dead letter queue if max retries exceeded
                    success = self.move_job_to_dead_letter(
                        job.id,
                        f"Exceeded maximum retry attempts ({max_retries})"
                    )
                    if success:
                        results["moved_to_dead_letter"] += 1
                    else:
                        results["errors"] += 1

            except Exception as e:
                self.logger.error(f"Error handling failed job {job.id}: {str(e)}", job_id=job.id, error=str(e))
                results["errors"] += 1

        self.logger.info(
            f"Monitoring completed",
            checked_jobs=results["checked_jobs"],
            retried=results["retried"],
            moved_to_dead_letter=results["moved_to_dead_letter"],
            errors=results["errors"]
        )

        return results

    def cleanup_old_jobs(self, days_to_keep: int = 30) -> Dict[str, Any]:
        """
        Clean up old jobs from various registries to free up Redis memory.

        Args:
            days_to_keep: Number of days to keep jobs

        Returns:
            Dictionary with cleanup results
        """
        try:
            from rq import Queue
            from rq.registry import StartedJobRegistry, FinishedJobRegistry, FailedJobRegistry

            cutoff_date = datetime.utcnow() - timedelta(days=days_to_keep)
            results = {
                "cleaned_finished": 0,
                "cleaned_started": 0,
                "cleaned_failed": 0,
                "errors": 0
            }

            # Get all queues
            queues = Queue.all(connection=self.redis_conn)

            for queue in queues:
                try:
                    # Clean finished jobs
                    finished_registry = FinishedJobRegistry(queue.name, connection=self.redis_conn)
                    cleaned = finished_registry.cleanup(cutoff_date)
                    results["cleaned_finished"] += cleaned

                    # Clean started jobs
                    started_registry = StartedJobRegistry(queue.name, connection=self.redis_conn)
                    cleaned = started_registry.cleanup(cutoff_date)
                    results["cleaned_started"] += cleaned

                except Exception as e:
                    self.logger.error(
                        f"Error cleaning up jobs in queue {queue.name}: {str(e)}",
                        queue_name=queue.name,
                        error=str(e)
                    )
                    results["errors"] += 1

            self.logger.info(
                f"Cleanup completed",
                days_to_keep=days_to_keep,
                cutoff_date=cutoff_date.isoformat(),
                cleaned_finished=results["cleaned_finished"],
                cleaned_started=results["cleaned_started"],
                errors=results["errors"]
            )

            return results

        except Exception as e:
            self.logger.error(f"Error in cleanup_old_jobs: {str(e)}", error=str(e))
            return {"cleaned_finished": 0, "cleaned_started": 0, "cleaned_failed": 0, "errors": 1}


class DeadLetterQueueService:
    """
    Service specifically for handling dead letter queue functionality.
    In RQ, we simulate a dead letter queue by tracking jobs that have been moved elsewhere.
    """

    def __init__(self, redis_conn: Redis = None):
        self.logger = get_logger("dead_letter_queue")
        self.redis_conn = redis_conn or create_default_redis_connection()
        self.dead_letter_key = "rag_pipeline:dead_letter_queue"
        self.event_publisher = EventPublisher()

    def add_to_dead_letter(self, job_id: str, original_queue: str, error_info: Dict[str, Any], reason: str) -> bool:
        """
        Add a job to the dead letter queue tracking.

        Args:
            job_id: ID of the job to add to dead letter
            original_queue: Original queue name
            error_info: Error information
            reason: Reason for dead letter placement

        Returns:
            True if successful, False otherwise
        """
        try:
            import json

            dead_letter_entry = {
                "job_id": job_id,
                "original_queue": original_queue,
                "error_info": error_info,
                "reason": reason,
                "timestamp": datetime.utcnow().isoformat(),
                "processed": False
            }

            # Store in Redis as a JSON string
            entry_key = f"{self.dead_letter_key}:{job_id}"
            self.redis_conn.set(entry_key, json.dumps(dead_letter_entry))

            self.logger.info(
                f"Added job {job_id} to dead letter queue",
                job_id=job_id,
                original_queue=original_queue,
                reason=reason
            )

            return True

        except Exception as e:
            self.logger.error(f"Error adding job to dead letter: {str(e)}", job_id=job_id, error=str(e))
            return False

    def get_dead_letter_jobs(self) -> List[Dict[str, Any]]:
        """
        Get all jobs in the dead letter queue.

        Returns:
            List of dead letter queue entries
        """
        try:
            import json

            # Get all keys matching the pattern
            dead_letter_keys = self.redis_conn.keys(f"{self.dead_letter_key}:*")
            dead_letter_jobs = []

            for key in dead_letter_keys:
                try:
                    entry_json = self.redis_conn.get(key)
                    if entry_json:
                        entry = json.loads(entry_json)
                        dead_letter_jobs.append(entry)
                except Exception as e:
                    self.logger.error(f"Error parsing dead letter entry: {str(e)}", key=key.decode(), error=str(e))

            self.logger.info(f"Found {len(dead_letter_jobs)} dead letter jobs", count=len(dead_letter_jobs))
            return dead_letter_jobs

        except Exception as e:
            self.logger.error(f"Error getting dead letter jobs: {str(e)}", error=str(e))
            return []

    def process_dead_letter_job(self, job_id: str, handler_func) -> bool:
        """
        Process a job from the dead letter queue using a handler function.

        Args:
            job_id: ID of the job to process
            handler_func: Function to handle the dead letter job

        Returns:
            True if successful, False otherwise
        """
        try:
            import json

            entry_key = f"{self.dead_letter_key}:{job_id}"
            entry_json = self.redis_conn.get(entry_key)

            if not entry_json:
                self.logger.warning(f"Dead letter job {job_id} not found", job_id=job_id)
                return False

            entry = json.loads(entry_json)

            if entry.get("processed", False):
                self.logger.warning(f"Dead letter job {job_id} already processed", job_id=job_id)
                return False

            # Process the job with the handler function
            success = handler_func(entry)

            if success:
                # Mark as processed
                entry["processed"] = True
                entry["processed_at"] = datetime.utcnow().isoformat()
                self.redis_conn.set(entry_key, json.dumps(entry))

                self.logger.info(f"Processed dead letter job {job_id}", job_id=job_id)

                # Optionally remove from dead letter queue after successful processing
                # self.redis_conn.delete(entry_key)
            else:
                self.logger.warning(f"Failed to process dead letter job {job_id}", job_id=job_id)

            return success

        except Exception as e:
            self.logger.error(f"Error processing dead letter job {job_id}: {str(e)}", job_id=job_id, error=str(e))
            return False

    def remove_from_dead_letter(self, job_id: str) -> bool:
        """
        Remove a job from the dead letter queue.

        Args:
            job_id: ID of the job to remove

        Returns:
            True if successful, False otherwise
        """
        try:
            entry_key = f"{self.dead_letter_key}:{job_id}"
            result = self.redis_conn.delete(entry_key)

            if result:
                self.logger.info(f"Removed job {job_id} from dead letter queue", job_id=job_id)
            else:
                self.logger.warning(f"Job {job_id} not found in dead letter queue", job_id=job_id)

            return bool(result)

        except Exception as e:
            self.logger.error(f"Error removing job from dead letter: {str(e)}", job_id=job_id, error=str(e))
            return False

    def get_dead_letter_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the dead letter queue.

        Returns:
            Dictionary with dead letter queue statistics
        """
        try:
            dead_letter_jobs = self.get_dead_letter_jobs()
            processed_count = sum(1 for job in dead_letter_jobs if job.get("processed", False))
            unprocessed_count = len(dead_letter_jobs) - processed_count

            stats = {
                "total_dead_letter_jobs": len(dead_letter_jobs),
                "processed_jobs": processed_count,
                "unprocessed_jobs": unprocessed_count,
                "timestamp": datetime.utcnow().isoformat()
            }

            self.logger.info("Retrieved dead letter queue stats", stats=stats)
            return stats

        except Exception as e:
            self.logger.error(f"Error getting dead letter stats: {str(e)}", error=str(e))
            return {}


def create_default_queue_monitor() -> QueueMonitorService:
    """
    Create a default queue monitor service instance.

    Returns:
        QueueMonitorService instance
    """
    return QueueMonitorService()


def create_default_dead_letter_queue() -> DeadLetterQueueService:
    """
    Create a default dead letter queue service instance.

    Returns:
        DeadLetterQueueService instance
    """
    return DeadLetterQueueService()