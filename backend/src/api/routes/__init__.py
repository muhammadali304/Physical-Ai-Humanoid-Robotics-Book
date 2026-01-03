"""
API routes initialization for the RAG Agent Backend.

This module initializes the API routes structure
following the implementation plan requirements.
"""

from fastapi import APIRouter
from . import query, documents, chat_sessions

# Main API router
api_router = APIRouter()

# Include sub-routers for different API versions
api_router.include_router(query.router, prefix="/v1", tags=["query"])
api_router.include_router(documents.router, prefix="/v1", tags=["documents"])
api_router.include_router(chat_sessions.router, prefix="/v1", tags=["chat_sessions"])

# Additional routes for other functionality
from . import alerts, backups
api_router.include_router(alerts.router, prefix="/v1", tags=["alerts"])
api_router.include_router(backups.router, prefix="/v1", tags=["backups"])

__all__ = ["api_router"]