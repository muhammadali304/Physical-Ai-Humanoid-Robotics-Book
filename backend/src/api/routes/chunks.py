"""
API routes for content chunks in the RAG Ingestion Pipeline.
Provides endpoints to manage and retrieve content chunks.
"""

from typing import List, Optional
from fastapi import APIRouter, HTTPException, Query
from datetime import datetime

from src.models.chunk import ContentChunk
from src.services.chunk_storage_service import ChunkStorageService
from src.services.job_service import CrawlJobService


router = APIRouter(prefix="/api/v1/content-chunks", tags=["content-chunks"])


@router.get("/", response_model=List[ContentChunk])
async def get_content_chunks(
    source_url: Optional[str] = Query(None, description="Filter chunks by source URL"),
    job_id: Optional[str] = Query(None, description="Filter chunks by crawl job ID"),
    page_title: Optional[str] = Query(None, description="Filter chunks by page title"),
    limit: int = Query(100, ge=1, le=1000, description="Maximum number of chunks to return"),
    offset: int = Query(0, ge=0, description="Number of chunks to skip")
) -> List[ContentChunk]:
    """
    Get a list of content chunks with optional filtering and pagination.

    Args:
        source_url: Filter chunks by source URL
        job_id: Filter chunks by crawl job ID
        page_title: Filter chunks by page title
        limit: Maximum number of chunks to return (1-1000)
        offset: Number of chunks to skip for pagination

    Returns:
        List of ContentChunk objects
    """
    try:
        storage_service = ChunkStorageService()

        # Build filters based on query parameters
        filters = {}
        if source_url:
            filters["source_url"] = source_url
        if job_id:
            filters["metadata__crawl_job_id"] = job_id
        if page_title:
            filters["page_title"] = page_title

        # For now, we'll get all chunks and apply basic filtering in memory
        # In a production system, this would be handled by the repository
        all_chunks = await storage_service.get_all_chunks(limit=limit, offset=offset)

        # Apply filters
        filtered_chunks = []
        for chunk in all_chunks:
            match = True
            if source_url and chunk.source_url != source_url:
                match = False
            if job_id and chunk.metadata and chunk.metadata.get("crawl_job_id") != job_id:
                match = False
            if page_title and chunk.page_title != page_title:
                match = False

            if match:
                filtered_chunks.append(chunk)

        return filtered_chunks

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving content chunks: {str(e)}")


@router.get("/{chunk_id}", response_model=ContentChunk)
async def get_content_chunk(chunk_id: str) -> ContentChunk:
    """
    Get a specific content chunk by its ID.

    Args:
        chunk_id: ID of the chunk to retrieve

    Returns:
        ContentChunk object
    """
    try:
        storage_service = ChunkStorageService()
        chunk = await storage_service.get_chunk_by_id(chunk_id)

        if not chunk:
            raise HTTPException(status_code=404, detail=f"Content chunk {chunk_id} not found")

        return chunk

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving content chunk: {str(e)}")


@router.get("/by-source/{source_url:path}", response_model=List[ContentChunk])
async def get_chunks_by_source_url(source_url: str) -> List[ContentChunk]:
    """
    Get all content chunks for a specific source URL.

    Args:
        source_url: URL to filter chunks by

    Returns:
        List of ContentChunk objects
    """
    try:
        storage_service = ChunkStorageService()
        chunks = await storage_service.get_chunks_by_source_url(source_url)
        return chunks

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving chunks for URL: {str(e)}")


@router.get("/by-job/{job_id}", response_model=List[ContentChunk])
async def get_chunks_by_job_id(job_id: str) -> List[ContentChunk]:
    """
    Get all content chunks associated with a specific crawl job.

    Args:
        job_id: ID of the crawl job

    Returns:
        List of ContentChunk objects
    """
    try:
        storage_service = ChunkStorageService()
        chunks = await storage_service.get_chunks_by_job_id(job_id)
        return chunks

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving chunks for job: {str(e)}")


@router.get("/stats", response_model=dict)
async def get_chunk_stats() -> dict:
    """
    Get statistics about stored content chunks.

    Returns:
        Dictionary with chunk statistics
    """
    try:
        storage_service = ChunkStorageService()

        # For now, return basic stats
        # In a real implementation, this would query the database for aggregate stats
        all_chunks = await storage_service.get_all_chunks()

        stats = {
            "total_chunks": len(all_chunks),
            "total_tokens": sum(chunk.token_count for chunk in all_chunks),
            "unique_sources": len(set(chunk.source_url for chunk in all_chunks)),
            "avg_tokens_per_chunk": sum(chunk.token_count for chunk in all_chunks) / len(all_chunks) if all_chunks else 0,
            "last_updated": max((chunk.updated_at for chunk in all_chunks), default=datetime.now()) if all_chunks else datetime.now()
        }

        return stats

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving chunk statistics: {str(e)}")


# Additional endpoints can be added as needed:
# - POST endpoint to create chunks (if needed)
# - PUT endpoint to update chunks
# - DELETE endpoint to remove chunks