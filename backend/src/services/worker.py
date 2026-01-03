"""
RQ Worker service for the RAG Ingestion Pipeline.
Implements job processing workers for handling background tasks.
"""

import os
import sys
from typing import List, Dict, Any, Callable
from datetime import datetime
import logging

from rq import Worker, Queue
from redis import Redis

from src.config.redis_config import redis_config, create_default_redis_connection
from src.utils.logging import get_logger


class RQWorkerService:
    """
    Service to manage RQ workers for processing background jobs.
    """

    def __init__(self):
        self.logger = get_logger("rq_worker")
        self.redis_conn = create_default_redis_connection()
        self.workers: List[Worker] = []

    def create_worker(self, queue_names: List[str], name: str = None, **kwargs) -> Worker:
        """
        Create an RQ worker for the specified queues.

        Args:
            queue_names: List of queue names to listen to
            name: Name of the worker (optional)
            **kwargs: Additional worker configuration

        Returns:
            RQ Worker instance
        """
        try:
            # Create queues
            queues = [Queue(name=qname, connection=self.redis_conn) for qname in queue_names]

            # Set up worker configuration
            worker_config = {
                'connection': self.redis_conn,
                'name': name,
                'default_worker_ttl': 300,  # 5 minutes
            }
            worker_config.update(kwargs)

            # Create worker
            worker = Worker(queues, **worker_config)

            self.logger.info(
                f"Created worker for queues: {queue_names}",
                worker_name=name,
                queues=queue_names
            )

            return worker

        except Exception as e:
            self.logger.error(
                f"Error creating worker: {str(e)}",
                error=str(e),
                queue_names=queue_names,
                worker_name=name
            )
            raise

    def start_worker(self, queue_names: List[str], name: str = None, **kwargs) -> Worker:
        """
        Start a worker for the specified queues.

        Args:
            queue_names: List of queue names to listen to
            name: Name of the worker (optional)
            **kwargs: Additional worker configuration

        Returns:
            Started RQ Worker instance
        """
        try:
            worker = self.create_worker(queue_names, name, **kwargs)

            self.logger.info(
                f"Starting worker for queues: {queue_names}",
                worker_name=name,
                queues=queue_names
            )

            # Add to tracked workers
            self.workers.append(worker)

            return worker

        except Exception as e:
            self.logger.error(
                f"Error starting worker: {str(e)}",
                error=str(e),
                queue_names=queue_names
            )
            raise

    def start_default_workers(self) -> List[Worker]:
        """
        Start default workers for common queues used in the RAG pipeline.

        Returns:
            List of started worker instances
        """
        workers = []

        # Define default queue configuration
        queue_configs = [
            {
                "name": "crawling",
                "queues": ["crawling"],
                "description": "Handles web crawling jobs"
            },
            {
                "name": "chunking",
                "queues": ["chunking"],
                "description": "Handles content chunking jobs"
            },
            {
                "name": "embedding",
                "queues": ["embedding"],
                "description": "Handles embedding generation jobs"
            },
            {
                "name": "default",
                "queues": ["default"],
                "description": "Handles general background jobs"
            }
        ]

        for config in queue_configs:
            try:
                worker = self.start_worker(
                    queue_names=config["queues"],
                    name=config["name"],
                    # Additional worker options
                    job_monitoring_interval=30,
                    disable_default_exception_handler=False
                )

                workers.append(worker)

                self.logger.info(
                    f"Started {config['name']} worker for queues: {config['queues']}",
                    worker_name=config["name"],
                    queues=config["queues"],
                    description=config["description"]
                )

            except Exception as e:
                self.logger.error(
                    f"Failed to start {config['name']} worker: {str(e)}",
                    worker_name=config["name"],
                    error=str(e)
                )
                # Continue with other workers even if one fails

        return workers

    def run_worker_forever(self, queue_names: List[str], name: str = None, **kwargs):
        """
        Run a worker forever (blocking operation).
        This method is typically used as the main entry point for worker processes.

        Args:
            queue_names: List of queue names to listen to
            name: Name of the worker (optional)
            **kwargs: Additional worker configuration
        """
        try:
            worker = self.create_worker(queue_names, name, **kwargs)

            self.logger.info(
                f"Running worker {name or 'unnamed'} for queues: {queue_names}",
                worker_name=name,
                queues=queue_names
            )

            # Run the worker (this is a blocking call)
            worker.work()

        except KeyboardInterrupt:
            self.logger.info("Worker interrupted by user")
            sys.exit(0)
        except Exception as e:
            self.logger.error(
                f"Error in worker process: {str(e)}",
                error=str(e),
                queue_names=queue_names
            )
            raise

    def get_worker_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the workers.

        Returns:
            Dictionary with worker statistics
        """
        try:
            # Get information about all workers
            all_workers = Worker.all(connection=self.redis_conn)

            stats = {
                "total_workers": len(all_workers),
                "workers": [],
                "queues": [],
                "job_counts": {}
            }

            for worker in all_workers:
                worker_info = {
                    "name": worker.name,
                    "hostname": worker.hostname,
                    "pid": worker.pid,
                    "queues": [q.name for q in worker.queues],
                    "state": worker.state,
                    "birth_date": worker.birth_date.isoformat() if worker.birth_date else None,
                    "last_heartbeat": worker.last_heartbeat.isoformat() if worker.last_heartbeat else None,
                    "successful_job_count": worker.successful_job_count,
                    "failed_job_count": worker.failed_job_count,
                    "total_working_time": worker.total_working_time
                }
                stats["workers"].append(worker_info)

                # Track unique queues
                for queue_name in worker_info["queues"]:
                    if queue_name not in stats["queues"]:
                        stats["queues"].append(queue_name)

            # Get job counts per queue
            for queue_name in stats["queues"]:
                queue = Queue(name=queue_name, connection=self.redis_conn)
                stats["job_counts"][queue_name] = {
                    "queued": queue.count,
                    "failed": queue.failed_job_registry.count,
                    "started": queue.started_job_registry.count,
                    "deferred": queue.deferred_job_registry.count
                }

            self.logger.debug("Retrieved worker statistics", stats=stats)
            return stats

        except Exception as e:
            self.logger.error(
                f"Error getting worker stats: {str(e)}",
                error=str(e)
            )
            return {}

    def stop_all_workers(self):
        """
        Stop all tracked workers.
        """
        for worker in self.workers:
            try:
                # Workers are typically stopped via system signals in real usage
                # For programmatic control, we can cancel jobs
                self.logger.info(f"Stopping worker: {worker.name}")
            except Exception as e:
                self.logger.error(
                    f"Error stopping worker {worker.name}: {str(e)}",
                    worker_name=worker.name,
                    error=str(e)
                )

        # Clear the workers list
        self.workers.clear()

    def schedule_job(self, func: Callable, args: tuple = (), kwargs: dict = {},
                     queue_name: str = "default", timeout: str = "10m") -> Any:
        """
        Schedule a job to run in the background.

        Args:
            func: Function to execute
            args: Arguments to pass to the function
            kwargs: Keyword arguments to pass to the function
            queue_name: Name of the queue to use
            timeout: Job timeout (e.g., '10m' for 10 minutes)

        Returns:
            Job instance
        """
        try:
            queue = Queue(name=queue_name, connection=self.redis_conn)

            job = queue.enqueue(
                func,
                *args,
                **kwargs,
                timeout=timeout
            )

            self.logger.info(
                f"Scheduled job {job.id} on queue {queue_name}",
                job_id=job.id,
                queue_name=queue_name,
                function=func.__name__ if hasattr(func, '__name__') else str(func)
            )

            return job

        except Exception as e:
            self.logger.error(
                f"Error scheduling job: {str(e)}",
                error=str(e),
                function=func.__name__ if hasattr(func, '__name__') else str(func)
            )
            raise


def create_default_worker_service() -> RQWorkerService:
    """
    Create a default RQ worker service instance.

    Returns:
        RQWorkerService instance
    """
    return RQWorkerService()


# Convenience function to run a worker process
def run_worker_process(queue_names: List[str], name: str = None, **kwargs):
    """
    Main function to run a worker process.
    This is typically called from a dedicated worker process/script.

    Args:
        queue_names: List of queues to listen to
        name: Name of the worker
        **kwargs: Additional worker options
    """
    worker_service = create_default_worker_service()
    worker_service.run_worker_forever(queue_names, name, **kwargs)


if __name__ == "__main__":
    # If this script is run directly, start a default worker
    # This is useful for development/testing
    import argparse

    parser = argparse.ArgumentParser(description="RQ Worker for RAG Pipeline")
    parser.add_argument("--queue", "-q", nargs="+", default=["default"],
                       help="Queue names to listen to (default: default)")
    parser.add_argument("--name", "-n", default=None,
                       help="Worker name")

    args = parser.parse_args()

    print(f"Starting RQ worker for queues: {args.queue}")
    print(f"Worker name: {args.name or 'unnamed'}")

    # Create and run the worker
    worker_service = create_default_worker_service()
    worker_service.run_worker_forever(
        queue_names=args.queue,
        name=args.name
    )