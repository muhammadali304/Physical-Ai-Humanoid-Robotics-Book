"""
Crawl job API endpoints for the RAG Ingestion Pipeline.
Provides REST API for managing crawl jobs.
"""

from fastapi import APIRouter, HTTPException, status, Query, Depends
from typing import List, Optional
from uuid import UUID
from pydantic import BaseModel, Field, HttpUrl

from src.models.job import CrawlJob, JobStatus
from src.services.job_service import get_job_service, create_crawl_job, get_crawl_job, list_crawl_jobs
from src.utils.url import is_valid_url, normalize_url

# Request and response models for validation
class CreateCrawlJobRequest(BaseModel):
    target_url: HttpUrl = Field(..., description="URL of the documentation site to crawl")
    max_depth: int = Field(default=3, ge=1, le=10, description="Maximum depth to crawl")
    max_workers: int = Field(default=4, ge=1, le=20, description="Maximum number of concurrent workers")
    selectors: Optional[dict] = Field(default=None, description="CSS selectors for content extraction")


class CreateCrawlJobResponse(CrawlJob):
    """Response model for crawl job creation, inherits from CrawlJob model"""


class ListCrawlJobsQueryParams(BaseModel):
    status: Optional[JobStatus] = Field(None, description="Filter by job status")
    limit: int = Field(default=20, ge=1, le=100, description="Maximum number of jobs to return")
    offset: int = Field(default=0, ge=0, description="Number of jobs to skip")


# Create the API router
router = APIRouter(prefix="/api/v1", tags=["crawl-jobs"])


@router.post(
    "/crawl-jobs",
    response_model=CrawlJob,
    status_code=status.HTTP_201_CREATED,
    summary="Create a new crawl job",
    description="Initiate a crawl job to process a documentation site"
)
async def create_crawl_job_endpoint(request: CreateCrawlJobRequest):
    """
    Create a new crawl job to process a documentation site.

    Args:
        request: Request object containing job parameters

    Returns:
        Created CrawlJob object

    Raises:
        HTTPException: If the request is invalid
    """
    # Extract parameters from the validated request
    target_url = str(request.target_url)
    max_depth = request.max_depth
    max_workers = request.max_workers

    try:
        # Create the crawl job
        job = await create_crawl_job(target_url, max_depth, max_workers)
        return job
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail=str(e)
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to create crawl job: {str(e)}"
        )


@router.get(
    "/crawl-jobs/{job_id}",
    response_model=CrawlJob,
    summary="Get a crawl job by ID",
    description="Retrieve details of a specific crawl job"
)
async def get_crawl_job_endpoint(job_id: str):
    """
    Get a crawl job by its ID.

    Args:
        job_id: ID of the job to retrieve

    Returns:
        CrawlJob object

    Raises:
        HTTPException: If the job is not found
    """
    try:
        # Validate that job_id is a valid UUID
        uuid_job_id = UUID(job_id)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid job ID format: {job_id}"
        )

    job = await get_crawl_job(uuid_job_id)
    if not job:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Crawl job not found: {job_id}"
        )

    return job


@router.get(
    "/crawl-jobs",
    response_model=List[CrawlJob],
    summary="List crawl jobs",
    description="Retrieve a list of crawl jobs with optional filtering"
)
async def list_crawl_jobs_endpoint(
    status: Optional[JobStatus] = Query(None, description="Filter by job status"),
    limit: int = Query(default=20, ge=1, le=100, description="Maximum number of jobs to return"),
    offset: int = Query(default=0, ge=0, description="Number of jobs to skip")
):
    """
    List crawl jobs with optional filtering.

    Args:
        status: Filter by job status (optional)
        limit: Maximum number of jobs to return (default: 20, max: 100)
        offset: Number of jobs to skip (default: 0)

    Returns:
        List of CrawlJob objects
    """
    jobs = await list_crawl_jobs(status=status, limit=limit, offset=offset)
    return jobs


@router.post(
    "/crawl-jobs/{job_id}/cancel",
    response_model=CrawlJob,
    summary="Cancel a crawl job",
    description="Cancel a running crawl job"
)
async def cancel_crawl_job_endpoint(job_id: str):
    """
    Cancel a crawl job.

    Args:
        job_id: ID of the job to cancel

    Returns:
        Updated CrawlJob object

    Raises:
        HTTPException: If the job is not found or cannot be cancelled
    """
    try:
        # Validate that job_id is a valid UUID
        uuid_job_id = UUID(job_id)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid job ID format: {job_id}"
        )

    from src.services.job_service import cancel_crawl_job

    success = await cancel_crawl_job(uuid_job_id)
    if not success:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Crawl job not found or cannot be cancelled: {job_id}"
        )

    # Return the updated job
    job = await get_crawl_job(uuid_job_id)
    if not job:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Job was cancelled but could not be retrieved"
        )

    return job


# Include this router in the main app
def include_router(app):
    """
    Include the crawl job routes in the main FastAPI app.

    Args:
        app: FastAPI application instance
    """
    app.include_router(router)