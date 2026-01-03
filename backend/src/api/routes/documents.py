"""
Document ingestion API routes for the RAG Agent Backend.

This module provides API endpoints for ingesting technical documentation
following the implementation plan requirements for User Story 2.
"""

from fastapi import APIRouter, Depends, HTTPException, status, UploadFile, File
from typing import List, Optional
import logging
from pydantic import BaseModel
from src.models.document import DocumentMetadata
from src.services.document_ingestion import DocumentIngestionService, get_document_ingestion_service
from src.services.document_indexing import DocumentIndexingService, get_document_indexing_service
from src.utils.errors import handle_rag_error, DocumentProcessingError
from src.api.middleware.auth import AuthenticationMiddleware


# Create the router
router = APIRouter()
logger = logging.getLogger(__name__)

# Initialize services


class IngestionRequest(BaseModel):
    """Request model for document ingestion"""
    file_path: str
    chunk_size: int = 1000
    overlap: int = 200
    metadata: Optional[DocumentMetadata] = None


class BatchIngestionRequest(BaseModel):
    """Request model for batch document ingestion"""
    file_paths: List[str]
    chunk_size: int = 1000
    overlap: int = 200


class IngestionResponse(BaseModel):
    """Response model for document ingestion"""
    success: bool
    document_id: Optional[str] = None
    message: str
    chunk_count: Optional[int] = None


class BatchIngestionResponse(BaseModel):
    """Response model for batch document ingestion"""
    results: dict
    success_count: int
    total_count: int


@router.post(
    "/documents/ingest",
    response_model=IngestionResponse,
    summary="Ingest a single document",
    description="Upload and ingest a single document into the RAG system",
    status_code=status.HTTP_200_OK
)
async def ingest_document(
    file: UploadFile = File(...),
    document_ingestion_service: DocumentIngestionService = Depends(get_document_ingestion_service),
    # auth_credentials: HTTPAuthorizationCredentials = Depends(auth_middleware.api_key_header)
) -> IngestionResponse:
    """
    Ingest a single document into the RAG system.

    Args:
        file: The document file to ingest
        document_ingestion_service: The document ingestion service

    Returns:
        IngestionResponse: Result of the ingestion process
    """
    try:
        logger.info(f"Received document ingestion request for file: {file.filename}")

        # Save the uploaded file temporarily
        import tempfile
        import os
        with tempfile.NamedTemporaryFile(delete=False, suffix=os.path.splitext(file.filename)[1]) as temp_file:
            content = await file.read()
            temp_file.write(content)
            temp_file_path = temp_file.name

        try:
            # Perform the ingestion
            success = await document_ingestion_service.ingest_document(
                file_path=temp_file_path,
                chunk_size=1000,  # Default chunk size
                overlap=200       # Default overlap
            )

            if success:
                # Extract document ID from the temporary file path
                # In a real implementation, this would come from the ingestion service
                import hashlib
                content_hash = hashlib.sha256(content).hexdigest()[:16]
                document_id = f"doc_{content_hash}"

                response = IngestionResponse(
                    success=True,
                    document_id=document_id,
                    message=f"Successfully ingested document: {file.filename}",
                    chunk_count=0  # Would be returned by the ingestion service in a real implementation
                )
                logger.info(f"Successfully ingested document: {file.filename}")
            else:
                response = IngestionResponse(
                    success=False,
                    message=f"Failed to ingest document: {file.filename}",
                    chunk_count=0
                )
                logger.error(f"Failed to ingest document: {file.filename}")

        finally:
            # Clean up the temporary file
            os.unlink(temp_file_path)

        return response

    except DocumentProcessingError as e:
        logger.error(f"Document processing error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "error": e.message,
                "error_code": e.error_code,
                "details": e.details
            }
        )
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Unexpected error ingesting document {file.filename}: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error",
                "error_code": "INTERNAL_ERROR",
                "details": {"message": f"An unexpected error occurred while ingesting the document: {str(e)}"}
            }
        )


@router.post(
    "/documents/ingest-batch",
    response_model=BatchIngestionResponse,
    summary="Ingest multiple documents",
    description="Upload and ingest multiple documents into the RAG system",
    status_code=status.HTTP_200_OK
)
async def ingest_batch_documents(
    request: BatchIngestionRequest,
    document_ingestion_service: DocumentIngestionService = Depends(get_document_ingestion_service),
    # auth_credentials: HTTPAuthorizationCredentials = Depends(auth_middleware.api_key_header)
) -> BatchIngestionResponse:
    """
    Ingest multiple documents into the RAG system.

    Args:
        request: Batch ingestion request containing file paths
        document_ingestion_service: The document ingestion service

    Returns:
        BatchIngestionResponse: Results of the batch ingestion process
    """
    try:
        logger.info(f"Received batch ingestion request for {len(request.file_paths)} files")

        # Perform batch ingestion
        results = await document_ingestion_service.ingest_multiple_documents(
            file_paths=request.file_paths,
            chunk_size=request.chunk_size,
            overlap=request.overlap
        )

        success_count = sum(1 for success in results.values() if success)
        total_count = len(results)

        response = BatchIngestionResponse(
            results=results,
            success_count=success_count,
            total_count=total_count
        )

        logger.info(f"Batch ingestion completed: {success_count}/{total_count} successful")
        return response

    except DocumentProcessingError as e:
        logger.error(f"Document processing error in batch ingestion: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "error": e.message,
                "error_code": e.error_code,
                "details": e.details
            }
        )
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Unexpected error in batch ingestion: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error",
                "error_code": "INTERNAL_ERROR",
                "details": {"message": f"An unexpected error occurred during batch ingestion: {str(e)}"}
            }
        )


@router.get(
    "/documents/{document_id}/status",
    response_model=dict,
    summary="Get ingestion status",
    description="Get the ingestion status for a specific document",
    status_code=status.HTTP_200_OK
)
async def get_ingestion_status(
    document_id: str,
    document_ingestion_service: DocumentIngestionService = Depends(get_document_ingestion_service),
    # auth_credentials: HTTPAuthorizationCredentials = Depends(auth_middleware.api_key_header)
) -> dict:
    """
    Get the ingestion status for a specific document.

    Args:
        document_id: ID of the document to check
        document_ingestion_service: The document ingestion service

    Returns:
        Dictionary containing the ingestion status
    """
    try:
        logger.info(f"Getting ingestion status for document: {document_id}")

        status_info = await document_ingestion_service.get_ingestion_status(document_id)

        logger.info(f"Retrieved status for document {document_id}")
        return status_info

    except Exception as e:
        logger.error(f"Error getting ingestion status for {document_id}: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error",
                "error_code": "INTERNAL_ERROR",
                "details": {"message": f"Error retrieving ingestion status: {str(e)}"}
            }
        )


@router.delete(
    "/documents/{document_id}",
    response_model=IngestionResponse,
    summary="Delete a document",
    description="Remove a document and its chunks from the RAG system",
    status_code=status.HTTP_200_OK
)
async def delete_document(
    document_id: str,
    document_ingestion_service: DocumentIngestionService = Depends(get_document_ingestion_service),
    # auth_credentials: HTTPAuthorizationCredentials = Depends(auth_middleware.api_key_header)
) -> IngestionResponse:
    """
    Delete a document and its chunks from the RAG system.

    Args:
        document_id: ID of the document to delete
        document_ingestion_service: The document ingestion service

    Returns:
        IngestionResponse: Result of the deletion process
    """
    try:
        logger.info(f"Received delete request for document: {document_id}")

        success = await document_ingestion_service.delete_document(document_id)

        if success:
            response = IngestionResponse(
                success=True,
                document_id=document_id,
                message=f"Successfully deleted document: {document_id}",
                chunk_count=None
            )
            logger.info(f"Successfully deleted document: {document_id}")
        else:
            response = IngestionResponse(
                success=False,
                document_id=document_id,
                message=f"Failed to delete document: {document_id}",
                chunk_count=None
            )
            logger.error(f"Failed to delete document: {document_id}")

        return response

    except Exception as e:
        logger.error(f"Error deleting document {document_id}: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error",
                "error_code": "INTERNAL_ERROR",
                "details": {"message": f"Error deleting document: {str(e)}"}
            }
        )


@router.get(
    "/documents/index/stats",
    response_model=dict,
    summary="Get index statistics",
    description="Get statistics about the document index",
    status_code=status.HTTP_200_OK
)
async def get_index_statistics(
    document_indexing_service: DocumentIndexingService = Depends(get_document_indexing_service),
    # auth_credentials: HTTPAuthorizationCredentials = Depends(auth_middleware.api_key_header)
) -> dict:
    """
    Get statistics about the document index.

    Args:
        document_indexing_service: The document indexing service

    Returns:
        Dictionary containing index statistics
    """
    try:
        logger.info("Getting document index statistics")

        stats = await document_indexing_service.get_index_statistics()

        logger.info("Retrieved index statistics")
        return stats

    except Exception as e:
        logger.error(f"Error getting index statistics: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error",
                "error_code": "INTERNAL_ERROR",
                "details": {"message": f"Error retrieving index statistics: {str(e)}"}
            }
        )


@router.post(
    "/documents/index/optimize",
    response_model=dict,
    summary="Optimize the document index",
    description="Optimize the document index for better search performance",
    status_code=status.HTTP_200_OK
)
async def optimize_index(
    document_indexing_service: DocumentIndexingService = Depends(get_document_indexing_service),
    # auth_credentials: HTTPAuthorizationCredentials = Depends(auth_middleware.api_key_header)
) -> dict:
    """
    Optimize the document index for better search performance.

    Args:
        document_indexing_service: The document indexing service

    Returns:
        Dictionary containing optimization result
    """
    try:
        logger.info("Optimizing document index")

        success = await document_indexing_service.optimize_index()

        result = {
            "success": success,
            "message": "Index optimization completed successfully" if success else "Index optimization failed",
            "optimized_at": __import__('datetime').datetime.now().isoformat()
        }

        logger.info(f"Index optimization completed: {success}")
        return result

    except Exception as e:
        logger.error(f"Error optimizing index: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error",
                "error_code": "INTERNAL_ERROR",
                "details": {"message": f"Error optimizing index: {str(e)}"}
            }
        )


@router.get(
    "/documents/health",
    summary="Health check for document ingestion service",
    description="Check if the document ingestion service is operational",
    status_code=status.HTTP_200_OK
)
async def document_ingestion_health_check(
    document_ingestion_service: DocumentIngestionService = Depends(get_document_ingestion_service)
) -> dict:
    """
    Health check endpoint for the document ingestion service.

    Returns:
        dict: Health status information
    """
    try:
        logger.info("Health check requested for document ingestion service")

        # Check if the document ingestion service is healthy
        is_healthy = await document_ingestion_service.health_check()

        health_status = {
            "status": "healthy" if is_healthy else "unhealthy",
            "service": "document_ingestion",
            "timestamp": __import__('datetime').datetime.now().isoformat()
        }

        if is_healthy:
            logger.info("Document ingestion service health check passed")
        else:
            logger.warning("Document ingestion service health check failed")

        return health_status

    except Exception as e:
        logger.error(f"Health check failed for document ingestion service: {str(e)}", exc_info=True)
        return {
            "status": "unhealthy",
            "service": "document_ingestion",
            "error": str(e),
            "timestamp": __import__('datetime').datetime.now().isoformat()
        }


# Function to include this router in the main app
def include_router(app):
    """
    Include the document ingestion router in the main application.

    Args:
        app: FastAPI application instance
    """
    app.include_router(router, prefix="/api/v1", tags=["documents"])
    logger.info("Document ingestion router included in application")