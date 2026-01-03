"""
Data retention service for the RAG Ingestion Pipeline.
Implements configurable data retention policies for cleaning up old data.
"""

import asyncio
from datetime import datetime, timedelta
from typing import List, Optional, Dict, Any
from uuid import UUID
from enum import Enum

from src.config.settings import settings
from src.config.constants import DEFAULT_JOB_RESULT_TTL
from src.utils.logging import get_logger
from src.models.job import CrawlJob, JobStatus
from src.models.chunk import ContentChunk
from src.models.log import ProcessingLog


class RetentionPolicyType(Enum):
    """Types of retention policies."""
    TIME_BASED = "time_based"
    COUNT_BASED = "count_based"
    SIZE_BASED = "size_based"


class RetentionRule:
    """Defines a single retention rule."""

    def __init__(
        self,
        name: str,
        policy_type: RetentionPolicyType,
        value: Any,
        description: str = ""
    ):
        self.name = name
        self.policy_type = policy_type
        self.value = value
        self.description = description

    def should_delete(self, item: Any, created_at: datetime) -> bool:
        """Determine if an item should be deleted based on this rule."""
        if self.policy_type == RetentionPolicyType.TIME_BASED:
            # Value is the number of days to retain
            retention_period = timedelta(days=self.value)
            return datetime.utcnow() - created_at > retention_period
        elif self.policy_type == RetentionPolicyType.COUNT_BASED:
            # This would be handled differently, typically in a batch process
            return False  # Placeholder for count-based logic
        elif self.policy_type == RetentionPolicyType.SIZE_BASED:
            # This would be handled differently, typically in a batch process
            return False  # Placeholder for size-based logic
        return False


class RetentionService:
    """Service to manage data retention policies."""

    def __init__(self):
        self.logger = get_logger("retention_service")
        self._rules: Dict[str, RetentionRule] = {}
        self._initialize_default_rules()

    def _initialize_default_rules(self):
        """Initialize default retention rules."""
        # Default rule: Keep crawl jobs for 30 days
        self.add_rule(
            "crawl_job_retention",
            RetentionPolicyType.TIME_BASED,
            30,  # days
            "Delete crawl jobs older than 30 days"
        )

        # Default rule: Keep content chunks for 30 days
        self.add_rule(
            "content_chunk_retention",
            RetentionPolicyType.TIME_BASED,
            30,  # days
            "Delete content chunks older than 30 days"
        )

        # Default rule: Keep processing logs for 7 days
        self.add_rule(
            "processing_log_retention",
            RetentionPolicyType.TIME_BASED,
            7,  # days
            "Delete processing logs older than 7 days"
        )

        # Default rule: Keep embedding vectors for 30 days
        self.add_rule(
            "embedding_retention",
            RetentionPolicyType.TIME_BASED,
            30,  # days
            "Delete embedding vectors older than 30 days"
        )

    def add_rule(self, name: str, policy_type: RetentionPolicyType, value: Any, description: str = ""):
        """Add a new retention rule."""
        rule = RetentionRule(name, policy_type, value, description)
        self._rules[name] = rule
        self.logger.info(f"Added retention rule: {name}", rule=rule.name)

    def get_rule(self, name: str) -> Optional[RetentionRule]:
        """Get a retention rule by name."""
        return self._rules.get(name)

    async def apply_retention_policy(self, rule_name: str, items: List[Any]) -> List[Any]:
        """Apply a retention policy to a list of items and return items to delete."""
        rule = self.get_rule(rule_name)
        if not rule:
            self.logger.warning(f"Retention rule not found: {rule_name}")
            return []

        items_to_delete = []
        for item in items:
            # Get the created_at field from the item
            if hasattr(item, 'created_at'):
                created_at = item.created_at
            else:
                # If item doesn't have created_at, skip it
                continue

            if rule.should_delete(item, created_at):
                items_to_delete.append(item)

        self.logger.info(
            f"Retention policy {rule_name} identified {len(items_to_delete)} items for deletion",
            rule_name=rule_name,
            items_to_delete=len(items_to_delete)
        )

        return items_to_delete

    async def cleanup_crawl_jobs(self, retention_days: Optional[int] = None) -> int:
        """
        Clean up old crawl jobs based on retention policy.
        This is a placeholder - actual implementation would need access to the data store.
        """
        days = retention_days or 30
        cutoff_date = datetime.utcnow() - timedelta(days=days)

        self.logger.info(f"Cleaning up crawl jobs older than {cutoff_date}")

        # This would actually query the database for jobs to delete
        # For now, we'll just log what would happen
        deleted_count = 0  # This would be the actual count from DB operations

        self.logger.info(
            f"Completed crawl job cleanup, {deleted_count} jobs deleted",
            deleted_count=deleted_count
        )

        return deleted_count

    async def cleanup_content_chunks(self, retention_days: Optional[int] = None) -> int:
        """
        Clean up old content chunks based on retention policy.
        This is a placeholder - actual implementation would need access to the data store.
        """
        days = retention_days or 30
        cutoff_date = datetime.utcnow() - timedelta(days=days)

        self.logger.info(f"Cleaning up content chunks older than {cutoff_date}")

        # This would actually query the database for chunks to delete
        # For now, we'll just log what would happen
        deleted_count = 0  # This would be the actual count from DB operations

        self.logger.info(
            f"Completed content chunk cleanup, {deleted_count} chunks deleted",
            deleted_count=deleted_count
        )

        return deleted_count

    async def cleanup_processing_logs(self, retention_days: Optional[int] = None) -> int:
        """
        Clean up old processing logs based on retention policy.
        This is a placeholder - actual implementation would need access to the data store.
        """
        days = retention_days or 7
        cutoff_date = datetime.utcnow() - timedelta(days=days)

        self.logger.info(f"Cleaning up processing logs older than {cutoff_date}")

        # This would actually query the database for logs to delete
        # For now, we'll just log what would happen
        deleted_count = 0  # This would be the actual count from DB operations

        self.logger.info(
            f"Completed processing log cleanup, {deleted_count} logs deleted",
            deleted_count=deleted_count
        )

        return deleted_count

    async def cleanup_embeddings(self, retention_days: Optional[int] = None) -> int:
        """
        Clean up old embeddings based on retention policy.
        This is a placeholder - actual implementation would need access to the vector store.
        """
        days = retention_days or 30
        cutoff_date = datetime.utcnow() - timedelta(days=days)

        self.logger.info(f"Cleaning up embeddings older than {cutoff_date}")

        # This would actually query the vector store for embeddings to delete
        # For now, we'll just log what would happen
        deleted_count = 0  # This would be the actual count from vector store operations

        self.logger.info(
            f"Completed embedding cleanup, {deleted_count} embeddings deleted",
            deleted_count=deleted_count
        )

        return deleted_count

    async def run_full_cleanup(self) -> Dict[str, int]:
        """Run cleanup for all data types."""
        self.logger.info("Starting full data retention cleanup")

        results = {
            "crawl_jobs": await self.cleanup_crawl_jobs(),
            "content_chunks": await self.cleanup_content_chunks(),
            "processing_logs": await self.cleanup_processing_logs(),
            "embeddings": await self.cleanup_embeddings()
        }

        total_deleted = sum(results.values())
        self.logger.info(
            f"Completed full data retention cleanup, {total_deleted} items deleted in total",
            total_deleted=total_deleted,
            breakdown=results
        )

        return results

    async def schedule_cleanup_job(self, interval_hours: int = 24):
        """
        Schedule a periodic cleanup job.
        This would typically run in a background worker or scheduled task.
        """
        self.logger.info(f"Scheduling cleanup job to run every {interval_hours} hours")

        while True:
            try:
                await self.run_full_cleanup()
                await asyncio.sleep(interval_hours * 3600)  # Convert hours to seconds
            except Exception as e:
                self.logger.error(f"Error in scheduled cleanup: {str(e)}")
                # Continue running even if one iteration fails
                await asyncio.sleep(interval_hours * 3600)

    def configure_retention_periods(self, periods: Dict[str, int]):
        """
        Configure retention periods for different data types.

        Args:
            periods: Dictionary mapping data type to retention days
                    e.g., {"crawl_jobs": 30, "content_chunks": 30, "logs": 7}
        """
        for data_type, days in periods.items():
            if days <= 0:
                self.logger.warning(f"Invalid retention period for {data_type}: {days} days")
                continue

            rule_name = f"{data_type}_retention"
            old_rule = self.get_rule(rule_name)

            if old_rule:
                # Update existing rule
                old_rule.value = days
                self.logger.info(
                    f"Updated retention period for {data_type} to {days} days",
                    data_type=data_type,
                    days=days
                )
            else:
                # Add new rule
                self.add_rule(
                    rule_name,
                    RetentionPolicyType.TIME_BASED,
                    days,
                    f"Delete {data_type} older than {days} days"
                )
                self.logger.info(
                    f"Added retention period for {data_type} to {days} days",
                    data_type=data_type,
                    days=days
                )


# Global retention service instance
_retention_service: Optional[RetentionService] = None


def get_retention_service() -> RetentionService:
    """Get the global retention service instance."""
    global _retention_service
    if _retention_service is None:
        _retention_service = RetentionService()
    return _retention_service


# Convenience functions for common operations
async def run_retention_cleanup() -> Dict[str, int]:
    """Convenience function to run full retention cleanup."""
    service = get_retention_service()
    return await service.run_full_cleanup()


def configure_data_retention(periods: Dict[str, int]):
    """Convenience function to configure retention periods."""
    service = get_retention_service()
    service.configure_retention_periods(periods)