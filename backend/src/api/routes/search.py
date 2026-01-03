"""
Search API routes for the RAG Ingestion Pipeline.
Provides endpoints for semantic search functionality.
"""

from typing import List, Dict, Any, Optional
from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel

from src.services.embedding_service import CohereEmbeddingService
from src.models.chunk import ContentChunk


class SearchRequest(BaseModel):
    """Request model for search endpoint."""
    query: str
    top_k: int = Query(10, ge=1, le=100, description="Number of results to return")
    filters: Optional[Dict[str, Any]] = None


class SearchResponse(BaseModel):
    """Response model for search endpoint."""
    query: str
    results: List[Dict[str, Any]]
    total_results: int


router = APIRouter(prefix="/api/v1", tags=["search"])


@router.post("/search", response_model=SearchResponse)
async def semantic_search(request: SearchRequest) -> SearchResponse:
    """
    Perform semantic search using vector similarity.

    Args:
        request: Search request containing query text and parameters

    Returns:
        SearchResponse with similar content chunks and metadata
    """
    try:
        if not request.query.strip():
            raise HTTPException(status_code=400, detail="Query text cannot be empty")

        embedding_service = CohereEmbeddingService()

        # Perform semantic search
        similar_results = await embedding_service.search_similar_embeddings(
            query_text=request.query,
            top_k=request.top_k
        )

        # Format results
        formatted_results = []
        for result in similar_results:
            # Extract content chunk info from the payload
            payload = result.get("payload", {})
            formatted_result = {
                "id": result.get("id"),
                "score": result.get("score"),
                "content_chunk_id": payload.get("content_chunk_id"),
                "model_used": payload.get("model_used"),
                "content": None,  # Will need to fetch content separately if needed
                "metadata": payload
            }
            formatted_results.append(formatted_result)

        response = SearchResponse(
            query=request.query,
            results=formatted_results,
            total_results=len(formatted_results)
        )

        return response

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error performing semantic search: {str(e)}")


@router.post("/search/chunk", response_model=Dict[str, Any])
async def search_by_chunk(chunk: ContentChunk) -> Dict[str, Any]:
    """
    Perform semantic search using a content chunk as the query.

    Args:
        chunk: ContentChunk to use as search query

    Returns:
        Dictionary with search results
    """
    try:
        embedding_service = CohereEmbeddingService()

        # Perform semantic search using the chunk
        similar_results = await embedding_service.search_similar_embeddings_by_chunk(
            chunk=chunk,
            top_k=10
        )

        result = {
            "query_chunk_id": str(chunk.id),
            "results": similar_results,
            "total_results": len(similar_results)
        }

        return result

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error performing chunk-based search: {str(e)}")


@router.get("/search/health", response_model=Dict[str, Any])
async def search_health() -> Dict[str, Any]:
    """
    Health check for the search functionality.

    Returns:
        Dictionary with health status
    """
    try:
        embedding_service = CohereEmbeddingService()

        # Get service info to verify it's working
        info = await embedding_service.get_embedding_info()

        health_status = {
            "status": "healthy",
            "model": info.get("model"),
            "dimensions": info.get("dimensions"),
            "qdrant_collection": info.get("qdrant_collection"),
            "timestamp": __import__('datetime').datetime.now().isoformat()
        }

        return health_status

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Search health check failed: {str(e)}")