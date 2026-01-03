"""
Query API routes for the RAG Agent Backend.

This module provides the API endpoint for processing user queries
following the implementation plan requirements.
"""

from fastapi import APIRouter, Depends, HTTPException, status
from typing import List, Optional
import logging
from src.models.query import QueryRequest, QueryResponse
from src.services.query_handler import QueryHandlerService
from src.agents.rag_chatbot_agent import RAGChatbotAgent, get_rag_chatbot_agent
from src.utils.errors import handle_rag_error, RAGError, QueryProcessingError
from src.api.middleware.auth import AuthenticationMiddleware


# Create the router
router = APIRouter()
logger = logging.getLogger(__name__)

# Initialize services
query_handler = QueryHandlerService()


@router.post(
    "/query",
    response_model=QueryResponse,
    summary="Process a user query",
    description="Submit a natural language query to be processed by the RAG agent using Groq",
    status_code=status.HTTP_200_OK
)
async def process_query(
    query_request: QueryRequest,
    rag_chatbot_agent: RAGChatbotAgent = Depends(get_rag_chatbot_agent),
    # auth_credentials: HTTPAuthorizationCredentials = Depends(auth_middleware.api_key_header)
) -> QueryResponse:
    """
    Process a user query against the RAG system through the dedicated chatbot agent.

    Args:
        query_request: The query request containing the user's question and optional session info
        rag_chatbot_agent: The RAG Chatbot Agent instance

    Returns:
        QueryResponse: The response containing the answer and source references

    Raises:
        HTTPException: If there's an error processing the query
    """
    try:
        logger.info("RAG Pipeline Trace - Step 0: Query received by API route")
        logger.info(f"RAG Pipeline Trace - Step 0: Received query for RAG Chatbot Agent: {query_request.query[:50]}...")

        # Process the query through the RAG Chatbot Agent using the OpenAI Agents SDK
        response = await rag_chatbot_agent.process_query(query_request)

        logger.info(f"RAG Pipeline Trace - Step 4: Successfully processed query through RAG Chatbot Agent, response length: {len(response.response)}")
        logger.info("RAG Pipeline Trace - Step 4: Query processing completed, returning response")
        return response

    except RAGError as e:
        logger.error(f"RAG error processing query through agent: {str(e)}")
        # Convert RAG-specific error to HTTP exception
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
        logger.error(f"Unexpected error processing query through RAG Chatbot Agent: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error",
                "error_code": "INTERNAL_ERROR",
                "details": {"message": "An unexpected error occurred while processing the query through the RAG Chatbot Agent"}
            }
        )


@router.post(
    "/query/batch",
    response_model=List[Optional[QueryResponse]],
    summary="Process multiple queries in batch",
    description="Submit multiple queries to be processed in a batch",
    status_code=status.HTTP_200_OK
)
async def process_batch_queries(
    queries: List[QueryRequest],
    rag_chatbot_agent: RAGChatbotAgent = Depends(get_rag_chatbot_agent),
    # auth_credentials: HTTPAuthorizationCredentials = Depends(auth_middleware.api_key_header)
) -> List[Optional[QueryResponse]]:
    """
    Process multiple queries in a batch through the RAG Chatbot Agent.

    Args:
        queries: List of query requests to process
        rag_chatbot_agent: The RAG Chatbot Agent instance

    Returns:
        List of QueryResponse objects (None for failed queries)
    """
    try:
        logger.info(f"Received batch query request with {len(queries)} queries for RAG Chatbot Agent")

        # Process the batch of queries through the RAG Chatbot Agent
        responses = []
        for query_request in queries:
            try:
                response = await rag_chatbot_agent.process_query(query_request)
                responses.append(response)
            except Exception as e:
                logger.error(f"Error processing query in batch: {str(e)}")
                responses.append(None)  # Add None for failed queries

        success_count = sum(1 for r in responses if r is not None)
        logger.info(f"Successfully processed {success_count}/{len(queries)} queries in batch through RAG Chatbot Agent")
        return responses

    except Exception as e:
        logger.error(f"Error processing batch queries through RAG Chatbot Agent: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error",
                "error_code": "INTERNAL_ERROR",
                "details": {"message": f"Error processing batch queries through RAG Chatbot Agent: {str(e)}"}
            }
        )


@router.get(
    "/query/health",
    summary="Health check for query service",
    description="Check if the query service is operational",
    status_code=status.HTTP_200_OK
)
async def query_health_check(
    rag_chatbot_agent: RAGChatbotAgent = Depends(get_rag_chatbot_agent)
) -> dict:
    """
    Health check endpoint for the query service through the RAG Chatbot Agent.

    Returns:
        dict: Health status information
    """
    try:
        logger.info("Health check requested for RAG Chatbot Agent query service")

        # Check if the RAG Chatbot Agent is healthy
        is_healthy = await rag_chatbot_agent.health_check()

        health_status = {
            "status": "healthy" if is_healthy else "unhealthy",
            "service": "query",
            "agent": "RAG Chatbot Agent",
            "timestamp": __import__('datetime').datetime.now().isoformat()
        }

        if is_healthy:
            logger.info("RAG Chatbot Agent query service health check passed")
        else:
            logger.warning("RAG Chatbot Agent query service health check failed")

        return health_status

    except Exception as e:
        logger.error(f"Health check failed for RAG Chatbot Agent query service: {str(e)}", exc_info=True)
        return {
            "status": "unhealthy",
            "service": "query",
            "agent": "RAG Chatbot Agent",
            "error": str(e),
            "timestamp": __import__('datetime').datetime.now().isoformat()
        }


@router.get(
    "/query/stats",
    summary="Get query processing statistics",
    description="Retrieve statistics about query processing",
    status_code=status.HTTP_200_OK
)
async def get_query_stats(
    rag_chatbot_agent: RAGChatbotAgent = Depends(get_rag_chatbot_agent),
    # auth_credentials: HTTPAuthorizationCredentials = Depends(auth_middleware.api_key_header)
) -> dict:
    """
    Get statistics about query processing through the RAG Chatbot Agent.

    Returns:
        dict: Query processing statistics
    """
    try:
        logger.info("Query statistics requested for RAG Chatbot Agent")

        # Get statistics from the RAG Chatbot Agent
        stats = {
            "total_queries_processed": 0,  # Would need to track this in a real implementation
            "average_response_time_ms": 0.0,  # Would need to track this
            "success_rate": 1.0,  # Would need to track this
            "active_sessions": len(rag_chatbot_agent.conversation_sessions),  # Number of active conversation sessions
            "agent_model": getattr(rag_chatbot_agent, 'model', 'unknown'),  # Model being used
            "timestamp": __import__('datetime').datetime.now().isoformat()
        }

        logger.info("Query statistics retrieved successfully from RAG Chatbot Agent")
        return stats

    except Exception as e:
        logger.error(f"Error retrieving query statistics from RAG Chatbot Agent: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error",
                "error_code": "INTERNAL_ERROR",
                "details": {"message": f"Error retrieving query statistics from RAG Chatbot Agent: {str(e)}"}
            }
        )


# Function to include this router in the main app
def include_router(app):
    """
    Include the query router in the main application.

    Args:
        app: FastAPI application instance
    """
    app.include_router(router, prefix="/api/v1", tags=["query"])
    logger.info("Query router included in application")


