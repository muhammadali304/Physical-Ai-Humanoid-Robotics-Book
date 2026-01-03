"""
Backup API routes for the RAG Ingestion Pipeline.
Provides endpoints for managing backups and recovery operations.
"""

from typing import List, Dict, Any
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel

from src.services.backup_service import (
    BackupService, get_backup_service, BackupInfo
)
from src.api.auth.api_key_auth import api_key_auth


router = APIRouter(prefix="/backups", tags=["backups"])


class BackupResponse(BaseModel):
    """Response model for backup information."""
    id: str
    timestamp: str
    size_bytes: int
    status: str
    location: str
    collections_backed_up: List[str]
    metadata: Dict[str, Any]


class CreateBackupRequest(BaseModel):
    """Request model for creating a backup."""
    backup_id: str = None


class RestoreBackupRequest(BaseModel):
    """Request model for restoring a backup."""
    backup_id: str


@router.post("/", response_model=Dict[str, Any])
async def create_backup(
    request: CreateBackupRequest = None,
    service: BackupService = Depends(get_backup_service)
):
    """
    Create a backup of the system data.
    """
    try:
        backup_info = await service.create_backup(request.backup_id if request else None)
        return {
            "success": True,
            "message": "Backup created successfully",
            "backup_id": backup_info.id,
            "backup_info": {
                "id": backup_info.id,
                "timestamp": backup_info.timestamp.isoformat(),
                "size_bytes": backup_info.size_bytes,
                "status": backup_info.status,
                "location": backup_info.location,
                "collections_backed_up": backup_info.collections_backed_up
            }
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to create backup: {str(e)}")


@router.get("/", response_model=List[BackupResponse])
async def list_backups(
    service: BackupService = Depends(get_backup_service)
):
    """
    List all available backups.
    """
    try:
        backups = await service.list_backups()
        return [
            BackupResponse(
                id=backup.id,
                timestamp=backup.timestamp.isoformat(),
                size_bytes=backup.size_bytes,
                status=backup.status,
                location=backup.location,
                collections_backed_up=backup.collections_backed_up,
                metadata=backup.metadata
            )
            for backup in backups
        ]
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to list backups: {str(e)}")


@router.get("/{backup_id}", response_model=BackupResponse)
async def get_backup_info(
    backup_id: str,
    service: BackupService = Depends(get_backup_service)
):
    """
    Get information about a specific backup.
    """
    try:
        backup_info = await service.get_backup_info(backup_id)
        if not backup_info:
            raise HTTPException(status_code=404, detail=f"Backup {backup_id} not found")

        return BackupResponse(
            id=backup_info.id,
            timestamp=backup_info.timestamp.isoformat(),
            size_bytes=backup_info.size_bytes,
            status=backup_info.status,
            location=backup_info.location,
            collections_backed_up=backup_info.collections_backed_up,
            metadata=backup_info.metadata
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get backup info: {str(e)}")


@router.post("/restore", response_model=Dict[str, Any])
async def restore_backup(
    request: RestoreBackupRequest,
    service: BackupService = Depends(get_backup_service)
):
    """
    Restore from a backup.
    """
    try:
        success = await service.restore_backup(request.backup_id)
        if success:
            return {
                "success": True,
                "message": f"Restore from backup {request.backup_id} completed successfully"
            }
        else:
            raise HTTPException(status_code=500, detail=f"Failed to restore from backup {request.backup_id}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to restore backup: {str(e)}")


@router.delete("/{backup_id}", response_model=Dict[str, Any])
async def delete_backup(
    backup_id: str,
    service: BackupService = Depends(get_backup_service)
):
    """
    Delete a backup.
    """
    try:
        success = await service.delete_backup(backup_id)
        if success:
            return {
                "success": True,
                "message": f"Backup {backup_id} deleted successfully"
            }
        else:
            raise HTTPException(status_code=404, detail=f"Backup {backup_id} not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to delete backup: {str(e)}")


@router.post("/{backup_id}/verify", response_model=Dict[str, Any])
async def verify_backup(
    backup_id: str,
    service: BackupService = Depends(get_backup_service)
):
    """
    Verify the integrity of a backup.
    """
    try:
        verification_result = await service.verify_backup(backup_id)
        return {
            "success": True,
            "verification_result": verification_result
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to verify backup: {str(e)}")


# Include this router in the main app
def include_router(app):
    """
    Include the backups router in the main application.

    Args:
        app: FastAPI application instance
    """
    app.include_router(router, dependencies=[Depends(require_api_key)])