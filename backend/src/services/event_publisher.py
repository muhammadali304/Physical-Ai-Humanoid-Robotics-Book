"""
Event publisher service for the RAG Ingestion Pipeline.
Implements event publishing functionality for job status updates and notifications.
"""

import json
from typing import Any, Dict, Optional, List
from datetime import datetime
from enum import Enum

import redis
from pydantic import BaseModel

from src.config.redis_config import create_default_redis_connection
from src.utils.logging import get_logger


class EventType(str, Enum):
    """
    Enum for different types of events in the RAG pipeline.
    """
    CRAWL_JOB_STARTED = "crawl_job_started"
    CRAWL_JOB_COMPLETED = "crawl_job_completed"
    CRAWL_JOB_FAILED = "crawl_job_failed"
    CRAWL_JOB_PROGRESS = "crawl_job_progress"
    CHUNKING_STARTED = "chunking_started"
    CHUNKING_COMPLETED = "chunking_completed"
    EMBEDDING_STARTED = "embedding_started"
    EMBEDDING_COMPLETED = "embedding_completed"
    SYSTEM_HEALTH = "system_health"
    CONTENT_PROCESSED = "content_processed"
    ERROR_OCCURRED = "error_occurred"


class EventPayload(BaseModel):
    """
    Base model for event payloads.
    """
    event_type: EventType
    timestamp: datetime = None
    source: str
    data: Dict[str, Any]
    job_id: Optional[str] = None

    def __init__(self, **data):
        super().__init__(**data)
        if self.timestamp is None:
            self.timestamp = datetime.utcnow()


class EventPublisher:
    """
    Service to publish events to Redis channels for real-time notifications.
    """

    def __init__(self, redis_conn: redis.Redis = None):
        self.logger = get_logger("event_publisher")
        self.redis_conn = redis_conn or create_default_redis_connection()
        self.default_channel = "rag_pipeline_events"
        self.event_channels = {
            EventType.CRAWL_JOB_STARTED: "crawl_events",
            EventType.CRAWL_JOB_COMPLETED: "crawl_events",
            EventType.CRAWL_JOB_FAILED: "crawl_events",
            EventType.CRAWL_JOB_PROGRESS: "crawl_events",
            EventType.CHUNKING_STARTED: "chunking_events",
            EventType.CHUNKING_COMPLETED: "chunking_events",
            EventType.EMBEDDING_STARTED: "embedding_events",
            EventType.EMBEDDING_COMPLETED: "embedding_events",
            EventType.SYSTEM_HEALTH: "health_events",
            EventType.CONTENT_PROCESSED: "content_events",
            EventType.ERROR_OCCURRED: "error_events"
        }

    def _serialize_payload(self, payload: EventPayload) -> str:
        """
        Serialize event payload to JSON string.

        Args:
            payload: Event payload to serialize

        Returns:
            JSON string representation of the payload
        """
        try:
            # Convert datetime to ISO string for JSON serialization
            payload_dict = payload.dict()
            payload_dict['timestamp'] = payload_dict['timestamp'].isoformat()
            return json.dumps(payload_dict)
        except Exception as e:
            self.logger.error(
                f"Error serializing event payload: {str(e)}",
                error=str(e),
                payload=payload.dict()
            )
            raise

    def _get_channel_for_event(self, event_type: EventType) -> str:
        """
        Get the appropriate channel for the given event type.

        Args:
            event_type: Type of event

        Returns:
            Redis channel name
        """
        return self.event_channels.get(event_type, self.default_channel)

    def publish_event(self, payload: EventPayload) -> bool:
        """
        Publish an event to the appropriate Redis channel.

        Args:
            payload: Event payload to publish

        Returns:
            True if published successfully, False otherwise
        """
        try:
            # Check if Redis connection is available
            self.redis_conn.ping()
        except Exception as e:
            self.logger.warning(
                f"Redis connection unavailable, skipping event {payload.event_type}: {str(e)}",
                error=str(e),
                event_type=payload.event_type,
                job_id=payload.job_id
            )
            return False

        try:
            # Serialize the payload
            serialized_payload = self._serialize_payload(payload)

            # Determine the appropriate channel
            channel = self._get_channel_for_event(payload.event_type)

            # Publish to Redis
            result = self.redis_conn.publish(channel, serialized_payload)

            self.logger.info(
                f"Published event {payload.event_type} to channel {channel}",
                event_type=payload.event_type,
                channel=channel,
                result=result,
                job_id=payload.job_id
            )

            return result > 0  # Return True if at least one subscriber received the message

        except Exception as e:
            self.logger.error(
                f"Error publishing event: {str(e)}",
                error=str(e),
                event_type=payload.event_type,
                job_id=payload.job_id
            )
            return False

    def publish_crawl_job_event(self, event_type: EventType, job_id: str, data: Dict[str, Any]) -> bool:
        """
        Publish a crawl job-related event.

        Args:
            event_type: Type of crawl job event
            job_id: ID of the crawl job
            data: Additional data to include in the event

        Returns:
            True if published successfully, False otherwise
        """
        # Validate event type is a crawl job event
        crawl_events = [
            EventType.CRAWL_JOB_STARTED,
            EventType.CRAWL_JOB_COMPLETED,
            EventType.CRAWL_JOB_FAILED,
            EventType.CRAWL_JOB_PROGRESS
        ]

        if event_type not in crawl_events:
            raise ValueError(f"Event type {event_type} is not a valid crawl job event")

        payload = EventPayload(
            event_type=event_type,
            source="crawl_service",
            job_id=job_id,
            data=data
        )

        return self.publish_event(payload)

    def publish_crawl_job_started(self, job_id: str, target_url: str, options: Dict[str, Any]) -> bool:
        """
        Publish an event when a crawl job starts.

        Args:
            job_id: ID of the crawl job
            target_url: Target URL being crawled
            options: Crawl options

        Returns:
            True if published successfully, False otherwise
        """
        data = {
            "target_url": target_url,
            "options": options,
            "start_time": datetime.utcnow().isoformat()
        }

        return self.publish_crawl_job_event(
            EventType.CRAWL_JOB_STARTED,
            job_id,
            data
        )

    def publish_crawl_job_completed(self, job_id: str, result: Dict[str, Any]) -> bool:
        """
        Publish an event when a crawl job completes successfully.

        Args:
            job_id: ID of the crawl job
            result: Result of the crawl job

        Returns:
            True if published successfully, False otherwise
        """
        data = {
            "result": result,
            "completion_time": datetime.utcnow().isoformat()
        }

        return self.publish_crawl_job_event(
            EventType.CRAWL_JOB_COMPLETED,
            job_id,
            data
        )

    def publish_crawl_job_failed(self, job_id: str, error_message: str, error_details: Dict[str, Any] = None) -> bool:
        """
        Publish an event when a crawl job fails.

        Args:
            job_id: ID of the crawl job
            error_message: Error message
            error_details: Additional error details (optional)

        Returns:
            True if published successfully, False otherwise
        """
        data = {
            "error_message": error_message,
            "error_details": error_details or {},
            "failure_time": datetime.utcnow().isoformat()
        }

        return self.publish_crawl_job_event(
            EventType.CRAWL_JOB_FAILED,
            job_id,
            data
        )

    def publish_crawl_job_progress(self, job_id: str, progress_data: Dict[str, Any]) -> bool:
        """
        Publish an event with crawl job progress information.

        Args:
            job_id: ID of the crawl job
            progress_data: Progress data including counts, percentages, etc.

        Returns:
            True if published successfully, False otherwise
        """
        data = {
            "progress_data": progress_data,
            "update_time": datetime.utcnow().isoformat()
        }

        return self.publish_crawl_job_event(
            EventType.CRAWL_JOB_PROGRESS,
            job_id,
            data
        )

    def publish_content_processing_event(self, event_type: EventType, job_id: str, chunk_id: str = None,
                                      data: Dict[str, Any] = None) -> bool:
        """
        Publish a content processing event (chunking, embedding, etc.).

        Args:
            event_type: Type of processing event
            job_id: ID of the parent job
            chunk_id: ID of the specific chunk being processed (optional)
            data: Additional data to include

        Returns:
            True if published successfully, False otherwise
        """
        # Validate event type is a content processing event
        content_events = [
            EventType.CHUNKING_STARTED,
            EventType.CHUNKING_COMPLETED,
            EventType.EMBEDDING_STARTED,
            EventType.EMBEDDING_COMPLETED,
            EventType.CONTENT_PROCESSED
        ]

        if event_type not in content_events:
            raise ValueError(f"Event type {event_type} is not a valid content processing event")

        payload = EventPayload(
            event_type=event_type,
            source="processing_service",
            job_id=job_id,
            data=data or {}
        )
        if chunk_id:
            payload.data["chunk_id"] = chunk_id

        return self.publish_event(payload)

    def publish_system_health_event(self, health_status: Dict[str, Any]) -> bool:
        """
        Publish a system health event.

        Args:
            health_status: Dictionary containing health status information

        Returns:
            True if published successfully, False otherwise
        """
        payload = EventPayload(
            event_type=EventType.SYSTEM_HEALTH,
            source="system_monitor",
            data=health_status
        )

        return self.publish_event(payload)

    def publish_error_event(self, error_context: str, error_message: str,
                          error_details: Dict[str, Any] = None) -> bool:
        """
        Publish an error event.

        Args:
            error_context: Context where the error occurred
            error_message: Error message
            error_details: Additional error details (optional)

        Returns:
            True if published successfully, False otherwise
        """
        data = {
            "context": error_context,
            "message": error_message,
            "details": error_details or {},
            "timestamp": datetime.utcnow().isoformat()
        }

        payload = EventPayload(
            event_type=EventType.ERROR_OCCURRED,
            source="error_monitor",
            data=data
        )

        return self.publish_event(payload)

    def get_subscriber_count(self, channel: str) -> int:
        """
        Get the number of subscribers for a channel.

        Args:
            channel: Name of the channel

        Returns:
            Number of subscribers
        """
        try:
            # Use PUBSUB NUMSUB to get subscriber count
            result = self.redis_conn.pubsub_numsub(channel)
            for channel_info in result:
                if channel_info[0].decode('utf-8') == channel:
                    return int(channel_info[1])
            return 0
        except Exception as e:
            self.logger.error(
                f"Error getting subscriber count for channel {channel}: {str(e)}",
                error=str(e),
                channel=channel
            )
            return 0

    def batch_publish_events(self, payloads: List[EventPayload]) -> List[bool]:
        """
        Publish multiple events in a batch.

        Args:
            payloads: List of event payloads to publish

        Returns:
            List of boolean results indicating success/failure for each event
        """
        results = []
        for payload in payloads:
            try:
                result = self.publish_event(payload)
                results.append(result)
            except Exception as e:
                self.logger.error(
                    f"Error in batch publishing event: {str(e)}",
                    error=str(e),
                    event_type=payload.event_type,
                    job_id=payload.job_id
                )
                results.append(False)

        success_count = sum(results)
        self.logger.info(
            f"Batch published {success_count}/{len(payloads)} events",
            total_events=len(payloads),
            successful_events=success_count,
            failed_events=len(payloads) - success_count
        )

        return results


def create_default_event_publisher() -> EventPublisher:
    """
    Create a default event publisher instance.

    Returns:
        EventPublisher instance
    """
    return EventPublisher()


# Convenience functions
def publish_crawl_job_started(job_id: str, target_url: str, options: Dict[str, Any]) -> bool:
    """Convenience function to publish a crawl job started event."""
    publisher = create_default_event_publisher()
    return publisher.publish_crawl_job_started(job_id, target_url, options)


def publish_crawl_job_completed(job_id: str, result: Dict[str, Any]) -> bool:
    """Convenience function to publish a crawl job completed event."""
    publisher = create_default_event_publisher()
    return publisher.publish_crawl_job_completed(job_id, result)


def publish_crawl_job_failed(job_id: str, error_message: str, error_details: Dict[str, Any] = None) -> bool:
    """Convenience function to publish a crawl job failed event."""
    publisher = create_default_event_publisher()
    return publisher.publish_crawl_job_failed(job_id, error_message, error_details)


def publish_crawl_job_progress(job_id: str, progress_data: Dict[str, Any]) -> bool:
    """Convenience function to publish a crawl job progress event."""
    publisher = create_default_event_publisher()
    return publisher.publish_crawl_job_progress(job_id, progress_data)


def publish_error_event(error_context: str, error_message: str, error_details: Dict[str, Any] = None) -> bool:
    """Convenience function to publish an error event."""
    publisher = create_default_event_publisher()
    return publisher.publish_error_event(error_context, error_message, error_details)