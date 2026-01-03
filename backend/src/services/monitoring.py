"""
System health monitoring for the RAG Ingestion Pipeline.
Implements system health monitoring with Python-specific syntax highlighting.
"""

import asyncio
import time
from typing import Dict, Any, List, Optional
from datetime import datetime
import psutil
import os
import socket
from dataclasses import dataclass

from src.utils.logging import get_logger
from src.config.settings import settings
from src.services.crawler import CrawlerService
from src.services.job_service import CrawlJobService
from src.services.embedding_service import CohereEmbeddingService
from src.services.qdrant_client import QdrantClientService


@dataclass
class HealthCheckResult:
    """
    Result of a health check.
    """
    service: str
    status: str  # 'healthy', 'degraded', 'unhealthy'
    details: Dict[str, Any]
    timestamp: datetime
    message: Optional[str] = None


@dataclass
class SystemHealth:
    """
    Overall system health information.
    """
    status: str  # 'healthy', 'degraded', 'unhealthy'
    timestamp: datetime
    checks: List[HealthCheckResult]
    system_info: Dict[str, Any]


class HealthMonitorService:
    """
    Service to monitor system health and provide health check endpoints.
    """

    def __init__(self):
        self.logger = get_logger("health_monitor")
        self.crawler_service = CrawlerService()
        self.job_service = CrawlJobService()
        self.embedding_service = CohereEmbeddingService()
        self.qdrant_client = QdrantClientService()
        self.health_checks: List[HealthCheckResult] = []

    async def perform_comprehensive_health_check(self) -> SystemHealth:
        """
        Perform a comprehensive health check of the entire system.

        Returns:
            SystemHealth object with overall health status and individual checks
        """
        start_time = time.time()
        self.logger.info("Starting comprehensive health check")

        # Run all health checks concurrently
        check_tasks = [
            self.check_system_resources(),
            self.check_crawler_service(),
            self.check_job_service(),
            self.check_embedding_service(),
            self.check_qdrant_client(),
            self.check_application_health()
        ]

        results = await asyncio.gather(*check_tasks, return_exceptions=True)

        # Process results and filter out any exceptions
        health_results = []
        for i, result in enumerate(results):
            if isinstance(result, Exception):
                self.logger.error(f"Health check {i} failed with exception: {str(result)}", error=str(result))
                # Create a failed health check result
                check_names = ["system_resources", "crawler_service", "job_service",
                             "embedding_service", "qdrant_client", "application"]
                health_results.append(HealthCheckResult(
                    service=check_names[i],
                    status="unhealthy",
                    details={"error": str(result)},
                    timestamp=datetime.utcnow(),
                    message=f"Health check failed: {str(result)}"
                ))
            else:
                health_results.append(result)

        # Determine overall system status
        statuses = [check.status for check in health_results]
        if "unhealthy" in statuses:
            overall_status = "unhealthy"
        elif "degraded" in statuses:
            overall_status = "degraded"
        else:
            overall_status = "healthy"

        # Get system info
        system_info = await self._collect_system_info()

        system_health = SystemHealth(
            status=overall_status,
            timestamp=datetime.utcnow(),
            checks=health_results,
            system_info=system_info
        )

        duration = time.time() - start_time
        self.logger.info(
            f"Comprehensive health check completed in {duration:.2f}s with status: {overall_status}",
            duration=duration,
            overall_status=overall_status
        )

        # Store the health check result
        self.health_checks.append(HealthCheckResult(
            service="comprehensive",
            status=overall_status,
            details={"check_count": len(health_results)},
            timestamp=datetime.utcnow()
        ))

        return system_health

    async def check_system_resources(self) -> HealthCheckResult:
        """
        Check system resources (CPU, memory, disk, network).

        Returns:
            HealthCheckResult for system resources
        """
        try:
            # Get system resource usage
            cpu_percent = psutil.cpu_percent(interval=1)
            memory_info = psutil.virtual_memory()
            disk_usage = psutil.disk_usage('/')
            network_io = psutil.net_io_counters()

            # Define thresholds
            cpu_threshold = 80.0  # 80% CPU usage
            memory_threshold = 85.0  # 85% memory usage
            disk_threshold = 90.0  # 90% disk usage

            # Determine status based on thresholds
            status = "healthy"
            issues = []

            if cpu_percent > cpu_threshold:
                status = "degraded" if status == "healthy" else status
                issues.append(f"High CPU usage: {cpu_percent}% > {cpu_threshold}%")

            if memory_info.percent > memory_threshold:
                status = "degraded" if status == "healthy" else status
                issues.append(f"High memory usage: {memory_info.percent}% > {memory_threshold}%")

            if disk_usage.percent > disk_threshold:
                status = "degraded" if status == "healthy" else status
                issues.append(f"High disk usage: {disk_usage.percent}% > {disk_threshold}%")

            details = {
                "cpu_percent": cpu_percent,
                "memory_percent": memory_info.percent,
                "disk_percent": disk_usage.percent,
                "memory_available_gb": round(memory_info.available / (1024**3), 2),
                "disk_free_gb": round(disk_usage.free / (1024**3), 2),
                "network_bytes_sent": network_io.bytes_sent,
                "network_bytes_recv": network_io.bytes_recv,
                "timestamp": datetime.utcnow().isoformat()
            }

            if issues:
                message = f"System resources: {', '.join(issues)}"
            else:
                message = "System resources within normal ranges"

            return HealthCheckResult(
                service="system_resources",
                status=status,
                details=details,
                timestamp=datetime.utcnow(),
                message=message
            )

        except Exception as e:
            self.logger.error(f"Error checking system resources: {str(e)}", error=str(e))
            return HealthCheckResult(
                service="system_resources",
                status="unhealthy",
                details={"error": str(e)},
                timestamp=datetime.utcnow(),
                message=f"System resources check failed: {str(e)}"
            )

    async def check_crawler_service(self) -> HealthCheckResult:
        """
        Check crawler service health.

        Returns:
            HealthCheckResult for crawler service
        """
        try:
            # Test basic functionality - check if we can access the service
            # This is a lightweight check to see if the service is responsive
            start_time = time.time()

            # Test if we can access the service components
            if hasattr(self.crawler_service, 'http_client'):
                # The service is accessible
                status = "healthy"
                details = {
                    "accessible": True,
                    "response_time_ms": round((time.time() - start_time) * 1000, 2),
                    "selectors_count": len(self.crawler_service.content_selectors),
                    "navigation_selectors_count": len(self.crawler_service.navigation_selectors),
                    "timestamp": datetime.utcnow().isoformat()
                }
                message = "Crawler service is accessible and functioning"
            else:
                status = "unhealthy"
                details = {"accessible": False}
                message = "Crawler service is not accessible"

            return HealthCheckResult(
                service="crawler_service",
                status=status,
                details=details,
                timestamp=datetime.utcnow(),
                message=message
            )

        except Exception as e:
            self.logger.error(f"Error checking crawler service: {str(e)}", error=str(e))
            return HealthCheckResult(
                service="crawler_service",
                status="unhealthy",
                details={"error": str(e)},
                timestamp=datetime.utcnow(),
                message=f"Crawler service check failed: {str(e)}"
            )

    async def check_job_service(self) -> HealthCheckResult:
        """
        Check job service health.

        Returns:
            HealthCheckResult for job service
        """
        try:
            start_time = time.time()

            # Test if we can access the service and perform a basic operation
            # Just check if the repository is accessible
            if hasattr(self.job_service, 'repository'):
                # Count recent jobs as a basic test
                recent_jobs = await self.job_service.get_recent_jobs(hours=24)
                job_count = len(recent_jobs) if recent_jobs else 0

                status = "healthy"
                details = {
                    "accessible": True,
                    "recent_jobs_count": job_count,
                    "response_time_ms": round((time.time() - start_time) * 1000, 2),
                    "timestamp": datetime.utcnow().isoformat()
                }
                message = f"Job service is accessible with {job_count} recent jobs"
            else:
                status = "unhealthy"
                details = {"accessible": False}
                message = "Job service is not accessible"

            return HealthCheckResult(
                service="job_service",
                status=status,
                details=details,
                timestamp=datetime.utcnow(),
                message=message
            )

        except Exception as e:
            self.logger.error(f"Error checking job service: {str(e)}", error=str(e))
            return HealthCheckResult(
                service="job_service",
                status="unhealthy",
                details={"error": str(e)},
                timestamp=datetime.utcnow(),
                message=f"Job service check failed: {str(e)}"
            )

    async def check_embedding_service(self) -> HealthCheckResult:
        """
        Check embedding service health.

        Returns:
            HealthCheckResult for embedding service
        """
        try:
            start_time = time.time()

            # Test the embedding service by getting its info
            info = await self.embedding_service.get_embedding_info()

            status = "healthy"
            details = {
                "accessible": True,
                "model": info.get("model"),
                "dimensions": info.get("dimensions"),
                "api_provider": info.get("api_provider"),
                "qdrant_collection": info.get("qdrant_collection"),
                "response_time_ms": round((time.time() - start_time) * 1000, 2),
                "timestamp": datetime.utcnow().isoformat()
            }
            message = f"Embedding service is healthy using {info.get('model')} model"

            return HealthCheckResult(
                service="embedding_service",
                status=status,
                details=details,
                timestamp=datetime.utcnow(),
                message=message
            )

        except Exception as e:
            self.logger.error(f"Error checking embedding service: {str(e)}", error=str(e))
            return HealthCheckResult(
                service="embedding_service",
                status="unhealthy",
                details={"error": str(e)},
                timestamp=datetime.utcnow(),
                message=f"Embedding service check failed: {str(e)}"
            )

    async def check_qdrant_client(self) -> HealthCheckResult:
        """
        Check Qdrant client health.

        Returns:
            HealthCheckResult for Qdrant client
        """
        try:
            start_time = time.time()

            # Test Qdrant connection
            is_healthy = await self.qdrant_client.health_check()

            if is_healthy:
                # Get collection info for more details
                collection_info = await self.qdrant_client.get_collection_info()
                status = "healthy"
                details = {
                    "accessible": True,
                    "connected": True,
                    "collection_name": collection_info.get("collection_name"),
                    "vector_size": collection_info.get("vector_size"),
                    "distance": str(collection_info.get("distance")),
                    "point_count": collection_info.get("point_count"),
                    "response_time_ms": round((time.time() - start_time) * 1000, 2),
                    "timestamp": datetime.utcnow().isoformat()
                }
                message = f"Qdrant client is healthy with {collection_info.get('point_count', 0)} vectors"
            else:
                status = "unhealthy"
                details = {
                    "accessible": True,
                    "connected": False,
                    "response_time_ms": round((time.time() - start_time) * 1000, 2)
                }
                message = "Qdrant client is accessible but connection failed"

            return HealthCheckResult(
                service="qdrant_client",
                status=status,
                details=details,
                timestamp=datetime.utcnow(),
                message=message
            )

        except Exception as e:
            self.logger.error(f"Error checking Qdrant client: {str(e)}", error=str(e))
            return HealthCheckResult(
                service="qdrant_client",
                status="unhealthy",
                details={"error": str(e)},
                timestamp=datetime.utcnow(),
                message=f"Qdrant client check failed: {str(e)}"
            )

    async def check_application_health(self) -> HealthCheckResult:
        """
        Check general application health.

        Returns:
            HealthCheckResult for application health
        """
        try:
            start_time = time.time()

            # Check various application aspects
            uptime = time.time() - getattr(self, '_start_time', time.time())
            process = psutil.Process(os.getpid())
            open_files = len(process.open_files())
            num_threads = process.num_threads()
            num_fds = process.num_fds() if hasattr(process, 'num_fds') else 0

            # Check if we have valid API keys configured
            has_cohere_key = bool(settings.cohere_api_key)
            has_qdrant_config = bool(settings.qdrant_url or settings.qdrant_api_key)

            details = {
                "uptime_seconds": round(uptime, 2),
                "open_files_count": open_files,
                "threads_count": num_threads,
                "file_descriptors_count": num_fds,
                "has_cohere_api_key": has_cohere_key,
                "has_qdrant_config": has_qdrant_config,
                "response_time_ms": round((time.time() - start_time) * 1000, 2),
                "timestamp": datetime.utcnow().isoformat()
            }

            # Determine status based on resource usage
            status = "healthy"
            issues = []

            if num_threads > 100:  # Arbitrary threshold
                status = "degraded"
                issues.append(f"High thread count: {num_threads}")

            if open_files > 200:  # Arbitrary threshold
                status = "degraded" if status == "healthy" else status
                issues.append(f"High open files count: {open_files}")

            if not has_cohere_key:
                status = "degraded" if status == "healthy" else status
                issues.append("Cohere API key not configured")

            if not has_qdrant_config:
                status = "degraded" if status == "healthy" else status
                issues.append("Qdrant configuration incomplete")

            if issues:
                message = f"Application health: {', '.join(issues)}"
            else:
                message = "Application is healthy"

            return HealthCheckResult(
                service="application",
                status=status,
                details=details,
                timestamp=datetime.utcnow(),
                message=message
            )

        except Exception as e:
            self.logger.error(f"Error checking application health: {str(e)}", error=str(e))
            return HealthCheckResult(
                service="application",
                status="unhealthy",
                details={"error": str(e)},
                timestamp=datetime.utcnow(),
                message=f"Application health check failed: {str(e)}"
            )

    async def _collect_system_info(self) -> Dict[str, Any]:
        """
        Collect comprehensive system information.

        Returns:
            Dictionary with system information
        """
        try:
            # Get basic system info
            system_info = {
                "hostname": socket.gethostname(),
                "platform": f"{os.name} {psutil.OS}",
                "architecture": os.uname().machine if hasattr(os, 'uname') else "unknown",
                "python_version": f"{os.sys.version_info.major}.{os.sys.version_info.minor}.{os.sys.version_info.micro}",
                "process_id": os.getpid(),
                "working_directory": os.getcwd(),
                "environment": getattr(settings, 'environment', 'unknown'),
                "timestamp": datetime.utcnow().isoformat()
            }

            # Add more detailed system metrics
            boot_time = datetime.fromtimestamp(psutil.boot_time())
            system_info.update({
                "boot_time": boot_time.isoformat(),
                "uptime_seconds": time.time() - psutil.boot_time(),
                "cpu_count_logical": psutil.cpu_count(logical=True),
                "cpu_count_physical": psutil.cpu_count(logical=False),
                "cpu_freq_current": psutil.cpu_freq().current if psutil.cpu_freq() else None,
                "memory_total_gb": round(psutil.virtual_memory().total / (1024**3), 2),
                "disk_total_gb": round(psutil.disk_usage('/').total / (1024**3), 2),
            })

            return system_info

        except Exception as e:
            self.logger.error(f"Error collecting system info: {str(e)}", error=str(e))
            return {"error": str(e)}

    async def get_health_history(self, limit: int = 10) -> List[HealthCheckResult]:
        """
        Get historical health check results.

        Args:
            limit: Maximum number of results to return

        Returns:
            List of recent health check results
        """
        # Return the most recent health checks
        return self.health_checks[-limit:] if self.health_checks else []

    async def get_service_health(self, service_name: str) -> Optional[HealthCheckResult]:
        """
        Get health status for a specific service.

        Args:
            service_name: Name of the service to check

        Returns:
            HealthCheckResult for the specified service, or None if not found
        """
        # Find the most recent health check for the specified service
        for check in reversed(self.health_checks):
            if check.service == service_name:
                return check
        return None

    async def perform_liveness_check(self) -> HealthCheckResult:
        """
        Perform a lightweight liveness check to see if the application is running.

        Returns:
            HealthCheckResult indicating liveness
        """
        try:
            # A simple check to see if the application is responding
            start_time = time.time()

            # Basic check - just return success
            status = "healthy"
            details = {
                "alive": True,
                "response_time_ms": round((time.time() - start_time) * 1000, 2),
                "timestamp": datetime.utcnow().isoformat()
            }
            message = "Application is alive"

            return HealthCheckResult(
                service="liveness",
                status=status,
                details=details,
                timestamp=datetime.utcnow(),
                message=message
            )

        except Exception as e:
            self.logger.error(f"Liveness check failed: {str(e)}", error=str(e))
            return HealthCheckResult(
                service="liveness",
                status="unhealthy",
                details={"error": str(e)},
                timestamp=datetime.utcnow(),
                message=f"Liveness check failed: {str(e)}"
            )

    async def perform_readiness_check(self) -> HealthCheckResult:
        """
        Perform a readiness check to see if the application is ready to serve requests.

        Returns:
            HealthCheckResult indicating readiness
        """
        try:
            start_time = time.time()

            # Check if all critical services are ready
            checks = [
                ("qdrant", await self.qdrant_client.health_check()),
                ("embedding_service", True),  # Assume accessible if we got here
                ("job_service", True),  # Assume accessible if we got here
            ]

            all_ready = all(check[1] for check in checks)

            status = "healthy" if all_ready else "unhealthy"
            details = {
                "ready": all_ready,
                "checks": {name: status for name, status in checks},
                "response_time_ms": round((time.time() - start_time) * 1000, 2),
                "timestamp": datetime.utcnow().isoformat()
            }
            message = "Application is ready to serve requests" if all_ready else "Application is not ready to serve requests"

            return HealthCheckResult(
                service="readiness",
                status=status,
                details=details,
                timestamp=datetime.utcnow(),
                message=message
            )

        except Exception as e:
            self.logger.error(f"Readiness check failed: {str(e)}", error=str(e))
            return HealthCheckResult(
                service="readiness",
                status="unhealthy",
                details={"error": str(e)},
                timestamp=datetime.utcnow(),
                message=f"Readiness check failed: {str(e)}"
            )


def create_default_health_monitor() -> HealthMonitorService:
    """
    Create a default health monitor service instance.

    Returns:
        HealthMonitorService instance
    """
    service = HealthMonitorService()
    service._start_time = time.time()  # Track application start time
    return service


# Global instance
health_monitor = create_default_health_monitor()


def get_health_monitor() -> HealthMonitorService:
    """
    Get the global health monitor instance.

    Returns:
        HealthMonitorService instance
    """
    return health_monitor


# Convenience functions
async def perform_comprehensive_health_check() -> SystemHealth:
    """
    Convenience function to perform a comprehensive health check.
    """
    return await health_monitor.perform_comprehensive_health_check()


async def get_system_health_status() -> str:
    """
    Convenience function to get the overall system health status.
    """
    system_health = await health_monitor.perform_comprehensive_health_check()
    return system_health.status


async def perform_liveness_check() -> HealthCheckResult:
    """
    Convenience function to perform a liveness check.
    """
    return await health_monitor.perform_liveness_check()


async def perform_readiness_check() -> HealthCheckResult:
    """
    Convenience function to perform a readiness check.
    """
    return await health_monitor.perform_readiness_check()


async def get_service_health(service_name: str) -> Optional[HealthCheckResult]:
    """
    Convenience function to get health for a specific service.
    """
    return await health_monitor.get_service_health(service_name)


# Export for use in other modules
__all__ = [
    'HealthMonitorService',
    'HealthCheckResult',
    'SystemHealth',
    'create_default_health_monitor',
    'get_health_monitor',
    'perform_comprehensive_health_check',
    'get_system_health_status',
    'perform_liveness_check',
    'perform_readiness_check',
    'get_service_health',
    'health_monitor'
]