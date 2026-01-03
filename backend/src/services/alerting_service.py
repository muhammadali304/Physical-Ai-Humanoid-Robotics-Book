"""
Alerting service for the RAG Ingestion Pipeline.
Provides alerting functionality for critical system failures and important events.
"""

import asyncio
import smtplib
import json
from datetime import datetime
from typing import Dict, Any, List, Optional
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from dataclasses import dataclass
from enum import Enum

from src.config.settings import settings
from src.utils.logging import get_logger


class AlertSeverity(Enum):
    """Alert severity levels."""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class AlertType(Enum):
    """Types of alerts."""
    SYSTEM_FAILURE = "system_failure"
    PERFORMANCE_ISSUE = "performance_issue"
    SECURITY_EVENT = "security_event"
    DATA_ISSUE = "data_issue"
    EXTERNAL_SERVICE_FAILURE = "external_service_failure"
    RESOURCE_EXHAUSTION = "resource_exhaustion"


@dataclass
class Alert:
    """Data class for alert information."""
    id: str
    title: str
    message: str
    severity: AlertSeverity
    alert_type: AlertType
    timestamp: datetime
    source: str
    details: Dict[str, Any]
    recipients: List[str]


class AlertingService:
    """Service to handle alerting for critical system events."""

    def __init__(self):
        self.logger = get_logger("alerting")
        self.alerts = []
        self.webhook_urls = settings.get("alert_webhook_urls", [])
        self.email_config = {
            "smtp_server": settings.get("smtp_server", "localhost"),
            "smtp_port": settings.get("smtp_port", 587),
            "smtp_username": settings.get("smtp_username"),
            "smtp_password": settings.get("smtp_password"),
            "from_email": settings.get("from_email", "noreply@rag-pipeline.com")
        }
        self.alert_recipients = settings.get("alert_recipients", [])
        self.performance_thresholds = {
            "response_time_ms": settings.get("alert_response_time_threshold", 5000),  # 5 seconds
            "error_rate": settings.get("alert_error_rate_threshold", 0.05),  # 5%
            "memory_usage_percent": settings.get("alert_memory_threshold", 80.0),  # 80%
            "disk_usage_percent": settings.get("alert_disk_threshold", 90.0)  # 90%
        }

    async def create_alert(self, title: str, message: str, severity: AlertSeverity,
                          alert_type: AlertType, source: str, details: Dict[str, Any] = None,
                          recipients: List[str] = None) -> Alert:
        """
        Create and trigger an alert.

        Args:
            title: Alert title
            message: Alert message
            severity: Alert severity level
            alert_type: Type of alert
            source: Source of the alert
            details: Additional details about the alert
            recipients: List of recipient emails (optional)

        Returns:
            Created Alert object
        """
        from uuid import uuid4

        alert = Alert(
            id=str(uuid4()),
            title=title,
            message=message,
            severity=severity,
            alert_type=alert_type,
            timestamp=datetime.utcnow(),
            source=source,
            details=details or {},
            recipients=recipients or self.alert_recipients
        )

        self.logger.critical(
            f"ALERT: {alert.title}",
            alert_id=alert.id,
            severity=alert.severity.value,
            alert_type=alert.alert_type.value,
            source=alert.source,
            details=alert.details
        )

        # Store the alert
        self.alerts.append(alert)

        # Send the alert
        await self.send_alert(alert)

        return alert

    async def send_alert(self, alert: Alert):
        """
        Send an alert through all configured channels.

        Args:
            alert: Alert object to send
        """
        tasks = []

        # Send via webhook if configured
        if self.webhook_urls:
            tasks.append(self._send_webhook_alert(alert))

        # Send via email if recipients are configured
        if alert.recipients:
            tasks.append(self._send_email_alert(alert))

        # Execute all alert tasks concurrently
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    async def _send_webhook_alert(self, alert: Alert):
        """
        Send an alert via webhook.

        Args:
            alert: Alert object to send
        """
        import httpx

        alert_payload = {
            "id": alert.id,
            "title": alert.title,
            "message": alert.message,
            "severity": alert.severity.value,
            "type": alert.alert_type.value,
            "timestamp": alert.timestamp.isoformat(),
            "source": alert.source,
            "details": alert.details
        }

        for webhook_url in self.webhook_urls:
            try:
                async with httpx.AsyncClient() as client:
                    response = await client.post(
                        webhook_url,
                        json=alert_payload,
                        timeout=10.0
                    )
                    if response.status_code >= 200 and response.status_code < 300:
                        self.logger.info(
                            f"Webhook alert sent successfully to {webhook_url}",
                            alert_id=alert.id,
                            webhook_url=webhook_url
                        )
                    else:
                        self.logger.error(
                            f"Failed to send webhook alert to {webhook_url}",
                            alert_id=alert.id,
                            webhook_url=webhook_url,
                            status_code=response.status_code,
                            response_text=response.text
                        )
            except Exception as e:
                self.logger.error(
                    f"Error sending webhook alert to {webhook_url}: {str(e)}",
                    alert_id=alert.id,
                    webhook_url=webhook_url,
                    error=str(e)
                )

    async def _send_email_alert(self, alert: Alert):
        """
        Send an alert via email.

        Args:
            alert: Alert object to send
        """
        if not self.email_config["smtp_username"] or not self.email_config["smtp_password"]:
            self.logger.warning(
                "Email alert configuration missing, skipping email alert",
                alert_id=alert.id
            )
            return

        try:
            # Create message
            msg = MIMEMultipart()
            msg['From'] = self.email_config["from_email"]
            msg['To'] = ", ".join(alert.recipients)
            msg['Subject'] = f"[{alert.severity.value.upper()}] {alert.title}"

            # Create HTML body
            html_body = self._create_alert_email_body(alert)
            msg.attach(MIMEText(html_body, 'html'))

            # Connect to server and send email
            server = smtplib.SMTP(self.email_config["smtp_server"], self.email_config["smtp_port"])
            server.starttls()
            server.login(self.email_config["smtp_username"], self.email_config["smtp_password"])

            text = msg.as_string()
            server.sendmail(self.email_config["from_email"], alert.recipients, text)
            server.quit()

            self.logger.info(
                f"Email alert sent successfully to {len(alert.recipients)} recipients",
                alert_id=alert.id,
                recipients=alert.recipients
            )

        except Exception as e:
            self.logger.error(
                f"Error sending email alert: {str(e)}",
                alert_id=alert.id,
                recipients=alert.recipients,
                error=str(e)
            )

    def _create_alert_email_body(self, alert: Alert) -> str:
        """
        Create HTML body for alert email.

        Args:
            alert: Alert object

        Returns:
            HTML string for email body
        """
        return f"""
        <html>
        <head>
            <style>
                body {{ font-family: Arial, sans-serif; }}
                .alert {{
                    border: 2px solid #ff0000;
                    border-radius: 5px;
                    padding: 15px;
                    margin: 10px 0;
                    background-color: #ffe6e6;
                }}
                .header {{
                    font-size: 18px;
                    font-weight: bold;
                    color: #d00;
                    margin-bottom: 10px;
                }}
                .severity {{
                    display: inline-block;
                    padding: 3px 8px;
                    border-radius: 3px;
                    color: white;
                    font-weight: bold;
                    background-color: {self._get_severity_color(alert.severity)};
                }}
                .details {{
                    margin-top: 15px;
                    padding: 10px;
                    background-color: #f9f9f9;
                    border-radius: 3px;
                }}
                .timestamp {{ font-size: 12px; color: #666; }}
            </style>
        </head>
        <body>
            <div class="alert">
                <div class="header">
                    <span class="severity">{alert.severity.value.upper()}</span>
                    {alert.title}
                </div>
                <div class="timestamp">Time: {alert.timestamp.strftime('%Y-%m-%d %H:%M:%S UTC')}</div>
                <p>{alert.message}</p>

                <div class="details">
                    <h3>Details:</h3>
                    <p><strong>Source:</strong> {alert.source}</p>
                    <p><strong>Type:</strong> {alert.alert_type.value}</p>
                    {self._format_details_html(alert.details)}
                </div>
            </div>
        </body>
        </html>
        """

    def _get_severity_color(self, severity: AlertSeverity) -> str:
        """
        Get color for severity level.

        Args:
            severity: Alert severity

        Returns:
            CSS color code
        """
        colors = {
            AlertSeverity.LOW: "#007acc",
            AlertSeverity.MEDIUM: "#ff9900",
            AlertSeverity.HIGH: "#ff6600",
            AlertSeverity.CRITICAL: "#cc0000"
        }
        return colors.get(severity, "#000000")

    def _format_details_html(self, details: Dict[str, Any]) -> str:
        """
        Format alert details as HTML.

        Args:
            details: Alert details dictionary

        Returns:
            HTML string for details
        """
        if not details:
            return "<p>No additional details provided.</p>"

        html = "<ul>"
        for key, value in details.items():
            html += f"<li><strong>{key}:</strong> {value}</li>"
        html += "</ul>"
        return html

    async def check_performance_thresholds(self, metrics: Dict[str, Any]):
        """
        Check if performance metrics exceed thresholds and trigger alerts if needed.

        Args:
            metrics: Dictionary of performance metrics
        """
        alerts_triggered = []

        # Check response time
        if "response_time_ms" in metrics:
            if metrics["response_time_ms"] > self.performance_thresholds["response_time_ms"]:
                alert = await self.create_alert(
                    title="High Response Time",
                    message=f"Response time of {metrics['response_time_ms']}ms exceeds threshold of {self.performance_thresholds['response_time_ms']}ms",
                    severity=AlertSeverity.HIGH,
                    alert_type=AlertType.PERFORMANCE_ISSUE,
                    source="performance_monitor"
                )
                alerts_triggered.append(alert)

        # Check error rate
        if "error_rate" in metrics:
            if metrics["error_rate"] > self.performance_thresholds["error_rate"]:
                alert = await self.create_alert(
                    title="High Error Rate",
                    message=f"Error rate of {metrics['error_rate']:.2%} exceeds threshold of {self.performance_thresholds['error_rate']:.2%}",
                    severity=AlertSeverity.HIGH,
                    alert_type=AlertType.PERFORMANCE_ISSUE,
                    source="performance_monitor"
                )
                alerts_triggered.append(alert)

        # Check memory usage
        if "memory_usage_percent" in metrics:
            if metrics["memory_usage_percent"] > self.performance_thresholds["memory_usage_percent"]:
                alert = await self.create_alert(
                    title="High Memory Usage",
                    message=f"Memory usage of {metrics['memory_usage_percent']:.1f}% exceeds threshold of {self.performance_thresholds['memory_usage_percent']:.1f}%",
                    severity=AlertSeverity.HIGH,
                    alert_type=AlertType.RESOURCE_EXHAUSTION,
                    source="resource_monitor"
                )
                alerts_triggered.append(alert)

        # Check disk usage
        if "disk_usage_percent" in metrics:
            if metrics["disk_usage_percent"] > self.performance_thresholds["disk_usage_percent"]:
                alert = await self.create_alert(
                    title="High Disk Usage",
                    message=f"Disk usage of {metrics['disk_usage_percent']:.1f}% exceeds threshold of {self.performance_thresholds['disk_usage_percent']:.1f}%",
                    severity=AlertSeverity.HIGH,
                    alert_type=AlertType.RESOURCE_EXHAUSTION,
                    source="resource_monitor"
                )
                alerts_triggered.append(alert)

        return alerts_triggered

    async def check_system_health(self) -> List[Alert]:
        """
        Perform a comprehensive system health check and trigger alerts for issues.

        Returns:
            List of alerts triggered during health check
        """
        alerts_triggered = []

        try:
            # Check if external services are available
            await self._check_external_services()

            # Check database connectivity
            await self._check_database_health()

            # Check Redis connectivity
            await self._check_redis_health()

            # Check Qdrant connectivity
            await self._check_qdrant_health()

            self.logger.info("System health check completed successfully")

        except Exception as e:
            alert = await self.create_alert(
                title="System Health Check Failed",
                message=f"System health check failed with error: {str(e)}",
                severity=AlertSeverity.CRITICAL,
                alert_type=AlertType.SYSTEM_FAILURE,
                source="health_monitor",
                details={"error": str(e)}
            )
            alerts_triggered.append(alert)

        return alerts_triggered

    async def _check_external_services(self):
        """Check if external services (like Cohere API) are available."""
        import httpx

        try:
            # Test Cohere API
            async with httpx.AsyncClient(timeout=10.0) as client:
                response = await client.get(
                    "https://api.cohere.ai/health",
                    headers={"Authorization": f"Bearer {settings.cohere_api_key}"}
                )
                if response.status_code != 200:
                    await self.create_alert(
                        title="Cohere API Unavailable",
                        message="Cohere API health check failed",
                        severity=AlertSeverity.HIGH,
                        alert_type=AlertType.EXTERNAL_SERVICE_FAILURE,
                        source="external_service_monitor",
                        details={
                            "status_code": response.status_code,
                            "response_text": response.text
                        }
                    )
        except Exception as e:
            await self.create_alert(
                title="Cohere API Unavailable",
                message=f"Cohere API connection failed: {str(e)}",
                severity=AlertSeverity.HIGH,
                alert_type=AlertType.EXTERNAL_SERVICE_FAILURE,
                source="external_service_monitor",
                details={"error": str(e)}
            )

    async def _check_database_health(self):
        """Check database connectivity."""
        # This would check database connectivity in a real implementation
        pass

    async def _check_redis_health(self):
        """Check Redis connectivity."""
        # This would check Redis connectivity in a real implementation
        pass

    async def _check_qdrant_health(self):
        """Check Qdrant connectivity."""
        # This would check Qdrant connectivity in a real implementation
        pass

    def get_recent_alerts(self, limit: int = 50) -> List[Alert]:
        """
        Get recent alerts.

        Args:
            limit: Maximum number of alerts to return

        Returns:
            List of recent alerts
        """
        return self.alerts[-limit:]


# Global instance
_alerting_service = None


def get_alerting_service() -> AlertingService:
    """
    Get the global alerting service instance.

    Returns:
        AlertingService instance
    """
    global _alerting_service
    if _alerting_service is None:
        _alerting_service = AlertingService()
    return _alerting_service


async def trigger_system_failure_alert(title: str, message: str, details: Dict[str, Any] = None):
    """
    Convenience function to trigger a system failure alert.

    Args:
        title: Alert title
        message: Alert message
        details: Additional details
    """
    service = get_alerting_service()
    await service.create_alert(
        title=title,
        message=message,
        severity=AlertSeverity.CRITICAL,
        alert_type=AlertType.SYSTEM_FAILURE,
        source="system",
        details=details
    )


async def trigger_performance_alert(title: str, message: str, details: Dict[str, Any] = None):
    """
    Convenience function to trigger a performance alert.

    Args:
        title: Alert title
        message: Alert message
        details: Additional details
    """
    service = get_alerting_service()
    await service.create_alert(
        title=title,
        message=message,
        severity=AlertSeverity.HIGH,
        alert_type=AlertType.PERFORMANCE_ISSUE,
        source="performance_monitor",
        details=details
    )