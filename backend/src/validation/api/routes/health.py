from fastapi import APIRouter
from typing import Dict


router = APIRouter(tags=["health"])


@router.get("/health", response_model=Dict[str, str])
async def health_check():
    """
    Health check endpoint for the validation service
    """
    return {"status": "healthy", "service": "rag-validation"}