"""
Workflow service for the RAG Ingestion Pipeline.
Orchestrates the complete workflow from crawling to embedding storage.
"""

import asyncio
from typing import Dict, Any, List, Optional
from datetime import datetime
from uuid import UUID

from src.models.job import CrawlJob, JobStatus
from src.models.chunk import ContentChunk
from src.models.embedding import EmbeddingVector
from src.services.crawler import CrawlerService
from src.services.chunker import ChunkerService
from src.services.chunk_storage_service import ChunkStorageService
from src.services.embedding_service import CohereEmbeddingService
from src.services.job_service import CrawlJobService
from src.services.event_publisher import EventPublisher
from src.utils.logging import get_logger


class WorkflowService:
    """
    Service to orchestrate the complete RAG ingestion workflow.
    Coordinates crawling, chunking, embedding, and storage operations.
    """

    def __init__(self):
        self.logger = get_logger("workflow_service")
        self.crawler_service = CrawlerService()
        self.chunker_service = ChunkerService()
        self.chunk_storage_service = ChunkStorageService()
        self.embedding_service = CohereEmbeddingService()
        self.job_service = CrawlJobService()
        self.event_publisher = EventPublisher()

    async def start_ingestion_workflow(
        self,
        target_url: str,
        max_depth: int = 3,
        chunk_options: Dict[str, Any] = None,
        embedding_options: Dict[str, Any] = None,
        job_metadata: Dict[str, Any] = None
    ) -> Dict[str, Any]:
        """
        Start the complete ingestion workflow.

        Args:
            target_url: URL of the documentation site to ingest
            max_depth: Maximum depth to crawl
            chunk_options: Options for chunking (size, overlap, etc.)
            embedding_options: Options for embedding generation
            job_metadata: Additional metadata for the job

        Returns:
            Dictionary with workflow results and statistics
        """
        try:
            self.logger.info(
                f"Starting ingestion workflow for {target_url}",
                target_url=target_url,
                max_depth=max_depth
            )

            # Create a crawl job
            crawl_job = await self.job_service.create_job(
                target_url=target_url,
                max_depth=max_depth,
                metadata=job_metadata or {}
            )

            # Publish event for workflow start
            await self.event_publisher.publish_crawl_job_started(
                job_id=str(crawl_job.id),
                target_url=target_url,
                options={
                    "max_depth": max_depth,
                    "chunk_options": chunk_options,
                    "embedding_options": embedding_options
                }
            )

            # Execute the workflow
            result = await self._execute_workflow(crawl_job, chunk_options, embedding_options)

            # Update job status to completed
            await self.job_service.update_job_status(crawl_job.id, JobStatus.COMPLETED)
            await self.job_service.update_job_completion_time(crawl_job.id)

            # Publish event for workflow completion
            await self.event_publisher.publish_crawl_job_completed(
                job_id=str(crawl_job.id),
                result=result
            )

            self.logger.info(
                f"Ingestion workflow completed for {target_url}",
                target_url=target_url,
                job_id=str(crawl_job.id),
                results=result
            )

            return result

        except Exception as e:
            self.logger.error(
                f"Ingestion workflow failed for {target_url}: {str(e)}",
                target_url=target_url,
                error=str(e)
            )
            # Update job status to failed
            if 'crawl_job' in locals():
                await self.job_service.update_job_status(crawl_job.id, JobStatus.FAILED)
                await self.job_service.update_job_error_log(crawl_job.id, str(e))

                # Publish event for workflow failure
                await self.event_publisher.publish_crawl_job_failed(
                    job_id=str(crawl_job.id),
                    error_message=str(e)
                )

            raise

    async def _execute_workflow(
        self,
        job: CrawlJob,
        chunk_options: Dict[str, Any] = None,
        embedding_options: Dict[str, Any] = None
    ) -> Dict[str, Any]:
        """
        Execute the core workflow steps: crawl -> chunk -> embed -> store.

        Args:
            job: The crawl job to execute
            chunk_options: Options for chunking
            embedding_options: Options for embedding generation

        Returns:
            Dictionary with workflow execution results
        """
        results = {
            "crawl_results": {},
            "chunking_results": {},
            "embedding_results": {},
            "storage_results": {},
            "total_processed": 0,
            "total_embeddings": 0,
            "start_time": datetime.utcnow().isoformat()
        }

        try:
            # Step 1: Crawl the website
            self.logger.info(f"Starting crawl for job {job.id}", job_id=str(job.id))
            crawl_result = await self.crawler_service.start_crawl_job(job)
            results["crawl_results"] = crawl_result.__dict__

            # Update job progress
            await self.job_service.update_job_progress(
                job.id,
                crawl_result.processed_pages,
                crawl_result.failed_pages,
                crawl_result.total_pages
            )

            # Step 2: Process crawled content (already chunked and embedded by crawler)
            # The crawler service already handles chunking and embedding internally
            self.logger.info(
                f"Crawl completed, processing {len(crawl_result.crawled_urls)} URLs",
                job_id=str(job.id),
                url_count=len(crawl_result.crawled_urls)
            )

            # For each crawled URL, we need to retrieve the stored chunks and embeddings
            total_chunks = 0
            total_embeddings = 0

            # Get all chunks associated with this job
            chunks = await self.chunk_storage_service.get_chunks_by_job_id(str(job.id))
            total_chunks = len(chunks)

            # Count embeddings by querying the embedding service
            # For now, we'll just return the counts we know about
            results["chunking_results"] = {
                "total_chunks": total_chunks,
                "processed_pages": crawl_result.processed_pages,
                "failed_pages": crawl_result.failed_pages
            }

            results["embedding_results"] = {
                "total_embeddings": total_embeddings  # This would be calculated based on stored embeddings
            }

            results["storage_results"] = {
                "chunks_stored": total_chunks,
                "embeddings_stored": total_embeddings
            }

            results["total_processed"] = crawl_result.processed_pages
            results["total_embeddings"] = total_embeddings

            results["end_time"] = datetime.utcnow().isoformat()
            results["duration_seconds"] = (
                datetime.fromisoformat(results["end_time"]) -
                datetime.fromisoformat(results["start_time"])
            ).total_seconds()

            return results

        except Exception as e:
            self.logger.error(
                f"Workflow execution failed for job {job.id}: {str(e)}",
                job_id=str(job.id),
                error=str(e)
            )
            raise

    async def start_batch_ingestion_workflow(
        self,
        urls: List[str],
        max_depth: int = 3,
        chunk_options: Dict[str, Any] = None,
        embedding_options: Dict[str, Any] = None
    ) -> Dict[str, Any]:
        """
        Start ingestion workflows for multiple URLs in parallel.

        Args:
            urls: List of URLs to ingest
            max_depth: Maximum depth to crawl for each URL
            chunk_options: Options for chunking
            embedding_options: Options for embedding generation

        Returns:
            Dictionary with batch workflow results
        """
        self.logger.info(f"Starting batch ingestion for {len(urls)} URLs", url_count=len(urls))

        # Create tasks for each URL
        tasks = [
            self.start_ingestion_workflow(
                url,
                max_depth=max_depth,
                chunk_options=chunk_options,
                embedding_options=embedding_options
            )
            for url in urls
        ]

        # Execute all tasks concurrently
        results = await asyncio.gather(*tasks, return_exceptions=True)

        # Process results
        successful_jobs = 0
        failed_jobs = 0
        total_results = []

        for i, result in enumerate(results):
            if isinstance(result, Exception):
                self.logger.error(
                    f"Batch job {i} (URL: {urls[i]}) failed: {str(result)}",
                    url=urls[i],
                    error=str(result)
                )
                failed_jobs += 1
                total_results.append({
                    "url": urls[i],
                    "success": False,
                    "error": str(result),
                    "timestamp": datetime.utcnow().isoformat()
                })
            else:
                successful_jobs += 1
                total_results.append({
                    "url": urls[i],
                    "success": True,
                    "result": result,
                    "timestamp": datetime.utcnow().isoformat()
                })

        batch_results = {
            "total_jobs": len(urls),
            "successful_jobs": successful_jobs,
            "failed_jobs": failed_jobs,
            "results": total_results,
            "start_time": datetime.utcnow().isoformat(),
            "end_time": datetime.utcnow().isoformat()
        }

        self.logger.info(
            f"Batch ingestion completed: {successful_jobs} successful, {failed_jobs} failed",
            successful=successful_jobs,
            failed=failed_jobs
        )

        return batch_results

    async def get_workflow_status(self, job_id: str) -> Dict[str, Any]:
        """
        Get the status of a specific workflow job.

        Args:
            job_id: ID of the job to check

        Returns:
            Dictionary with job status and progress information
        """
        try:
            # Get the job from the job service
            job = await self.job_service.get_job_by_id(UUID(job_id))
            if not job:
                raise ValueError(f"Job {job_id} not found")

            # Get job statistics
            chunks = await self.chunk_storage_service.get_chunks_by_job_id(job_id)
            chunk_count = len(chunks)

            # Get embedding count (this would require querying the embedding service)
            # For now, we'll just return what we know
            status_info = {
                "job_id": job_id,
                "status": job.status,
                "target_url": job.target_url,
                "created_at": job.created_at.isoformat(),
                "updated_at": job.updated_at.isoformat(),
                "chunks_processed": chunk_count,
                "progress": job.progress_percentage,
                "metadata": job.metadata
            }

            if job.completed_at:
                status_info["completed_at"] = job.completed_at.isoformat()

            if job.error_log:
                status_info["error_log"] = job.error_log

            return status_info

        except Exception as e:
            self.logger.error(
                f"Error getting workflow status for job {job_id}: {str(e)}",
                job_id=job_id,
                error=str(e)
            )
            raise

    async def cancel_workflow(self, job_id: str) -> bool:
        """
        Cancel a running workflow job.

        Args:
            job_id: ID of the job to cancel

        Returns:
            True if successfully cancelled, False otherwise
        """
        try:
            self.logger.info(f"Cancelling workflow job {job_id}", job_id=job_id)

            # Cancel the job in the job service
            success = await self.job_service.cancel_job(UUID(job_id))

            if success:
                # Publish cancellation event
                await self.event_publisher.publish_event_payload({
                    "event_type": "workflow_cancelled",
                    "source": "workflow_service",
                    "job_id": job_id,
                    "timestamp": datetime.utcnow().isoformat()
                })

            return success

        except Exception as e:
            self.logger.error(
                f"Error cancelling workflow job {job_id}: {str(e)}",
                job_id=job_id,
                error=str(e)
            )
            return False

    async def retry_failed_workflow(self, job_id: str) -> Dict[str, Any]:
        """
        Retry a failed workflow job.

        Args:
            job_id: ID of the job to retry

        Returns:
            Dictionary with retry results
        """
        try:
            self.logger.info(f"Retrying failed workflow job {job_id}", job_id=job_id)

            # Get the original job
            original_job = await self.job_service.get_job_by_id(UUID(job_id))
            if not original_job:
                raise ValueError(f"Job {job_id} not found")

            if original_job.status != JobStatus.FAILED:
                raise ValueError(f"Job {job_id} is not in FAILED status")

            # Create a new job with the same parameters
            new_job = await self.job_service.create_job(
                target_url=original_job.target_url,
                max_depth=original_job.options.get('max_depth', 3),
                metadata={
                    **original_job.metadata,
                    "retry_of": job_id,
                    "retry_count": original_job.metadata.get('retry_count', 0) + 1
                }
            )

            # Execute the workflow for the new job
            result = await self.start_ingestion_workflow(
                target_url=original_job.target_url,
                max_depth=original_job.options.get('max_depth', 3)
            )

            retry_result = {
                "original_job_id": job_id,
                "new_job_id": str(new_job.id),
                "success": True,
                "result": result
            }

            return retry_result

        except Exception as e:
            self.logger.error(
                f"Error retrying workflow job {job_id}: {str(e)}",
                job_id=job_id,
                error=str(e)
            )
            raise


def create_default_workflow_service() -> WorkflowService:
    """
    Create a default workflow service instance.

    Returns:
        WorkflowService instance
    """
    return WorkflowService()


# Convenience functions
async def start_ingestion_workflow(
    target_url: str,
    max_depth: int = 3,
    chunk_options: Dict[str, Any] = None,
    embedding_options: Dict[str, Any] = None,
    job_metadata: Dict[str, Any] = None
) -> Dict[str, Any]:
    """
    Convenience function to start an ingestion workflow.

    Args:
        target_url: URL of the documentation site to ingest
        max_depth: Maximum depth to crawl
        chunk_options: Options for chunking
        embedding_options: Options for embedding generation
        job_metadata: Additional metadata for the job

    Returns:
        Dictionary with workflow results and statistics
    """
    service = create_default_workflow_service()
    return await service.start_ingestion_workflow(
        target_url, max_depth, chunk_options, embedding_options, job_metadata
    )


async def start_batch_ingestion_workflow(
    urls: List[str],
    max_depth: int = 3,
    chunk_options: Dict[str, Any] = None,
    embedding_options: Dict[str, Any] = None
) -> Dict[str, Any]:
    """
    Convenience function to start a batch ingestion workflow.

    Args:
        urls: List of URLs to ingest
        max_depth: Maximum depth to crawl for each URL
        chunk_options: Options for chunking
        embedding_options: Options for embedding generation

    Returns:
        Dictionary with batch workflow results
    """
    service = create_default_workflow_service()
    return await service.start_batch_ingestion_workflow(
        urls, max_depth, chunk_options, embedding_options
    )


async def get_workflow_status(job_id: str) -> Dict[str, Any]:
    """
    Convenience function to get workflow status.

    Args:
        job_id: ID of the job to check

    Returns:
        Dictionary with job status and progress information
    """
    service = create_default_workflow_service()
    return await service.get_workflow_status(job_id)


async def cancel_workflow(job_id: str) -> bool:
    """
    Convenience function to cancel a workflow.

    Args:
        job_id: ID of the job to cancel

    Returns:
        True if successfully cancelled, False otherwise
    """
    service = create_default_workflow_service()
    return await service.cancel_workflow(job_id)