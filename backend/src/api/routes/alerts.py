"""
Alerting API routes for the RAG Ingestion Pipeline.
Provides endpoints for managing alerts and alerting configuration.
"""

from typing import List, Dict, Any
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel

from src.services.alerting_service import (
    AlertingService, get_alerting_service, Alert, AlertSeverity, AlertType
)
from src.api.auth.api_key_auth import api_key_auth


router = APIRouter(prefix="/alerts", tags=["alerts"])


class AlertRequest(BaseModel):
    """Request model for creating an alert."""
    title: str
    message: str
    severity: AlertSeverity
    alert_type: AlertType
    source: str
    details: Dict[str, Any] = {}
    recipients: List[str] = []


class AlertResponse(BaseModel):
    """Response model for alert information."""
    id: str
    title: str
    message: str
    severity: str
    alert_type: str
    timestamp: str
    source: str
    details: Dict[str, Any]
    recipients: List[str]


class PerformanceMetricsRequest(BaseModel):
    """Request model for performance metrics."""
    response_time_ms: float = None
    error_rate: float = None
    memory_usage_percent: float = None
    disk_usage_percent: float = None


@router.post("/", response_model=Dict[str, Any])
async def create_alert(
    alert_request: AlertRequest,
    service: AlertingService = Depends(get_alerting_service)
):
    """
    Create and send an alert.
    """
    try:
        alert = await service.create_alert(
            title=alert_request.title,
            message=alert_request.message,
            severity=alert_request.severity,
            alert_type=alert_request.alert_type,
            source=alert_request.source,
            details=alert_request.details,
            recipients=alert_request.recipients
        )

        return {
            "success": True,
            "message": "Alert created and sent successfully",
            "alert_id": alert.id
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to create alert: {str(e)}")


@router.get("/", response_model=List[AlertResponse])
async def get_recent_alerts(
    limit: int = 50,
    service: AlertingService = Depends(get_alerting_service)
):
    """
    Get recent alerts.
    """
    try:
        alerts = service.get_recent_alerts(limit=limit)
        return [
            AlertResponse(
                id=alert.id,
                title=alert.title,
                message=alert.message,
                severity=alert.severity.value,
                alert_type=alert.alert_type.value,
                timestamp=alert.timestamp.isoformat(),
                source=alert.source,
                details=alert.details,
                recipients=alert.recipients
            )
            for alert in alerts
        ]
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to retrieve alerts: {str(e)}")


@router.post("/check-performance")
async def check_performance_thresholds(
    metrics: PerformanceMetricsRequest,
    service: AlertingService = Depends(get_alerting_service)
):
    """
    Check performance metrics against thresholds and trigger alerts if needed.
    """
    try:
        metrics_dict = {}
        if metrics.response_time_ms is not None:
            metrics_dict["response_time_ms"] = metrics.response_time_ms
        if metrics.error_rate is not None:
            metrics_dict["error_rate"] = metrics.error_rate
        if metrics.memory_usage_percent is not None:
            metrics_dict["memory_usage_percent"] = metrics.memory_usage_percent
        if metrics.disk_usage_percent is not None:
            metrics_dict["disk_usage_percent"] = metrics.disk_usage_percent

        triggered_alerts = await service.check_performance_thresholds(metrics_dict)

        return {
            "success": True,
            "message": f"Performance check completed, {len(triggered_alerts)} alerts triggered",
            "alerts_triggered": len(triggered_alerts)
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to check performance: {str(e)}")


@router.post("/health-check")
async def perform_system_health_check(
    service: AlertingService = Depends(get_alerting_service)
):
    """
    Perform a comprehensive system health check and trigger alerts for issues.
    """
    try:
        alerts = await service.check_system_health()
        return {
            "success": True,
            "message": "System health check completed",
            "alerts_triggered": len(alerts)
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to perform health check: {str(e)}")


# Include this router in the main app
def include_router(app):
    """
    Include the alerts router in the main application.

    Args:
        app: FastAPI application instance
    """
    app.include_router(router, dependencies=[Depends(require_api_key)])