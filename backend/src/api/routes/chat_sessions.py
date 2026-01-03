"""
Chat session management API routes for the RAG Chatbot Agent.

This module provides API endpoints for managing conversation sessions
to support multi-turn conversations in the chatbot interface.
"""

from fastapi import APIRouter, Depends, HTTPException, status
from typing import List, Optional
import logging
from pydantic import BaseModel
from src.agents.rag_chatbot_agent import RAGChatbotAgent, get_rag_chatbot_agent
from src.api.middleware.auth import AuthenticationMiddleware


class StartSessionRequest(BaseModel):
    """Request model for starting a new chat session"""
    session_id: Optional[str] = None  # If not provided, will be generated


class StartSessionResponse(BaseModel):
    """Response model for starting a new chat session"""
    session_id: str
    success: bool
    message: str


class ClearSessionResponse(BaseModel):
    """Response model for clearing a chat session"""
    session_id: str
    success: bool
    message: str


class SessionStatsResponse(BaseModel):
    """Response model for session statistics"""
    session_id: str
    message_count: int
    last_accessed: Optional[str]
    active: bool
    success: bool


# Create the router
router = APIRouter()
logger = logging.getLogger(__name__)

# Initialize services


@router.post(
    "/chat/sessions",
    response_model=StartSessionResponse,
    summary="Start a new chat session",
    description="Start a new conversation session for multi-turn chat interactions",
    status_code=status.HTTP_200_OK
)
async def start_chat_session(
    request: StartSessionRequest,
    rag_chatbot_agent: RAGChatbotAgent = Depends(get_rag_chatbot_agent),
    # auth_credentials: HTTPAuthorizationCredentials = Depends(auth_middleware.api_key_header)
) -> StartSessionResponse:
    """
    Start a new chat session for multi-turn conversations.

    Args:
        request: Request containing session ID (optional)
        rag_chatbot_agent: The RAG Chatbot Agent instance

    Returns:
        StartSessionResponse: Result of the session creation
    """
    try:
        import uuid
        session_id = request.session_id or str(uuid.uuid4())

        logger.info(f"Starting new chat session: {session_id}")

        # Start a new conversation in the RAG Chatbot Agent
        success = await rag_chatbot_agent.start_new_conversation(session_id)

        if success:
            response = StartSessionResponse(
                session_id=session_id,
                success=True,
                message=f"Successfully started chat session: {session_id}"
            )
            logger.info(f"Successfully started chat session: {session_id}")
        else:
            response = StartSessionResponse(
                session_id=session_id,
                success=False,
                message=f"Failed to start chat session: {session_id}"
            )
            logger.error(f"Failed to start chat session: {session_id}")

        return response

    except Exception as e:
        logger.error(f"Error starting chat session: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error",
                "error_code": "INTERNAL_ERROR",
                "details": {"message": f"An unexpected error occurred while starting the chat session: {str(e)}"}
            }
        )


@router.delete(
    "/chat/sessions/{session_id}",
    response_model=ClearSessionResponse,
    summary="Clear a chat session",
    description="Clear the conversation history for a specific session",
    status_code=status.HTTP_200_OK
)
async def clear_chat_session(
    session_id: str,
    rag_chatbot_agent: RAGChatbotAgent = Depends(get_rag_chatbot_agent),
    # auth_credentials: HTTPAuthorizationCredentials = Depends(auth_middleware.api_key_header)
) -> ClearSessionResponse:
    """
    Clear the conversation history for a specific session.

    Args:
        session_id: ID of the session to clear
        rag_chatbot_agent: The RAG Chatbot Agent instance

    Returns:
        ClearSessionResponse: Result of the session clearing
    """
    try:
        logger.info(f"Clearing chat session: {session_id}")

        # Clear conversation history in the RAG Chatbot Agent
        success = await rag_chatbot_agent.clear_conversation_history(session_id)

        if success:
            response = ClearSessionResponse(
                session_id=session_id,
                success=True,
                message=f"Successfully cleared chat session: {session_id}"
            )
            logger.info(f"Successfully cleared chat session: {session_id}")
        else:
            response = ClearSessionResponse(
                session_id=session_id,
                success=False,
                message=f"Failed to clear chat session: {session_id}"
            )
            logger.error(f"Failed to clear chat session: {session_id}")

        return response

    except Exception as e:
        logger.error(f"Error clearing chat session {session_id}: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error",
                "error_code": "INTERNAL_ERROR",
                "details": {"message": f"An unexpected error occurred while clearing the chat session: {str(e)}"}
            }
        )


@router.get(
    "/chat/sessions/{session_id}/stats",
    response_model=SessionStatsResponse,
    summary="Get session statistics",
    description="Get statistics about a specific conversation session",
    status_code=status.HTTP_200_OK
)
async def get_session_stats(
    session_id: str,
    rag_chatbot_agent: RAGChatbotAgent = Depends(get_rag_chatbot_agent),
    # auth_credentials: HTTPAuthorizationCredentials = Depends(auth_middleware.api_key_header)
) -> SessionStatsResponse:
    """
    Get statistics about a specific conversation session.

    Args:
        session_id: ID of the session to get stats for
        rag_chatbot_agent: The RAG Chatbot Agent instance

    Returns:
        SessionStatsResponse: Statistics about the session
    """
    try:
        logger.info(f"Getting session stats for: {session_id}")

        # Get session statistics from the RAG Chatbot Agent
        stats = await rag_chatbot_agent.get_conversation_stats(session_id)

        response = SessionStatsResponse(
            session_id=session_id,
            message_count=stats.get("message_count", 0),
            last_accessed=stats.get("last_accessed"),
            active=stats.get("active", False),
            success=True
        )

        logger.info(f"Retrieved session stats for: {session_id}")
        return response

    except Exception as e:
        logger.error(f"Error getting session stats for {session_id}: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error",
                "error_code": "INTERNAL_ERROR",
                "details": {"message": f"An unexpected error occurred while getting session stats: {str(e)}"}
            }
        )


@router.get(
    "/chat/sessions",
    summary="Get all active sessions",
    description="Get a list of all active conversation sessions",
    status_code=status.HTTP_200_OK
)
async def get_all_sessions(
    rag_chatbot_agent: RAGChatbotAgent = Depends(get_rag_chatbot_agent),
    # auth_credentials: HTTPAuthorizationCredentials = Depends(auth_middleware.api_key_header)
) -> dict:
    """
    Get a list of all active conversation sessions.

    Args:
        rag_chatbot_agent: The RAG Chatbot Agent instance

    Returns:
        dict: List of active session IDs
    """
    try:
        logger.info("Getting all active chat sessions")

        # Get all active session IDs from the RAG Chatbot Agent
        active_sessions = list(rag_chatbot_agent.conversation_sessions.keys())

        response = {
            "active_sessions": active_sessions,
            "count": len(active_sessions),
            "success": True
        }

        logger.info(f"Retrieved {len(active_sessions)} active sessions")
        return response

    except Exception as e:
        logger.error(f"Error getting all active sessions: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "Internal server error",
                "error_code": "INTERNAL_ERROR",
                "details": {"message": f"An unexpected error occurred while getting active sessions: {str(e)}"}
            }
        )


@router.get(
    "/chat/health",
    summary="Health check for chat session service",
    description="Check if the chat session management service is operational",
    status_code=status.HTTP_200_OK
)
async def chat_health_check(
    rag_chatbot_agent: RAGChatbotAgent = Depends(get_rag_chatbot_agent)
) -> dict:
    """
    Health check endpoint for the chat session management service.

    Returns:
        dict: Health status information
    """
    try:
        logger.info("Health check requested for chat session management service")

        # Check if the RAG Chatbot Agent is healthy
        is_healthy = await rag_chatbot_agent.health_check()

        health_status = {
            "status": "healthy" if is_healthy else "unhealthy",
            "service": "chat_sessions",
            "agent": "RAG Chatbot Agent",
            "timestamp": __import__('datetime').datetime.now().isoformat()
        }

        if is_healthy:
            logger.info("Chat session management service health check passed")
        else:
            logger.warning("Chat session management service health check failed")

        return health_status

    except Exception as e:
        logger.error(f"Health check failed for chat session management service: {str(e)}", exc_info=True)
        return {
            "status": "unhealthy",
            "service": "chat_sessions",
            "agent": "RAG Chatbot Agent",
            "error": str(e),
            "timestamp": __import__('datetime').datetime.now().isoformat()
        }


# Function to include this router in the main app
def include_router(app):
    """
    Include the chat session router in the main application.

    Args:
        app: FastAPI application instance
    """
    app.include_router(router, prefix="/api/v1", tags=["chat"])
    logger.info("Chat session router included in application")