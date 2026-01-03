"""
Event listener service for the RAG Ingestion Pipeline.
Implements event listening functionality for job completion notifications.
"""

import json
import threading
import time
from typing import Callable, Dict, Any, List, Optional
from datetime import datetime

import redis
from pydantic import ValidationError

from src.config.redis_config import create_default_redis_connection
from src.services.event_publisher import EventPayload, EventType
from src.utils.logging import get_logger


class EventListener:
    """
    Service to listen for events from Redis channels and handle them with registered callbacks.
    """

    def __init__(self, redis_conn: redis.Redis = None):
        self.logger = get_logger("event_listener")
        self.redis_conn = redis_conn or create_default_redis_connection()
        self.pubsub = self.redis_conn.pubsub()
        self.event_handlers: Dict[EventType, List[Callable]] = {}
        self.channel_handlers: Dict[str, List[Callable]] = {}
        self.is_listening = False
        self.listener_thread = None
        self.default_channel = "rag_pipeline_events"

    def register_handler(self, event_type: EventType, handler: Callable):
        """
        Register a handler function for a specific event type.

        Args:
            event_type: Type of event to handle
            handler: Function to call when event is received
        """
        if event_type not in self.event_handlers:
            self.event_handlers[event_type] = []
        self.event_handlers[event_type].append(handler)

        self.logger.info(
            f"Registered handler for event type: {event_type}",
            event_type=event_type,
            handler_name=handler.__name__ if hasattr(handler, '__name__') else str(handler)
        )

    def register_channel_handler(self, channel: str, handler: Callable):
        """
        Register a handler function for a specific Redis channel.

        Args:
            channel: Name of the Redis channel
            handler: Function to call when message is received
        """
        if channel not in self.channel_handlers:
            self.channel_handlers[channel] = []
        self.channel_handlers[channel].append(handler)

        self.logger.info(
            f"Registered handler for channel: {channel}",
            channel=channel,
            handler_name=handler.__name__ if hasattr(handler, '__name__') else str(handler)
        )

    def _deserialize_payload(self, message_data: str) -> Optional[EventPayload]:
        """
        Deserialize event payload from JSON string.

        Args:
            message_data: JSON string representation of the payload

        Returns:
            EventPayload instance or None if deserialization fails
        """
        try:
            data_dict = json.loads(message_data)

            # Convert timestamp string back to datetime object
            if 'timestamp' in data_dict and isinstance(data_dict['timestamp'], str):
                data_dict['timestamp'] = datetime.fromisoformat(data_dict['timestamp'].replace('Z', '+00:00'))

            # Create EventPayload instance
            payload = EventPayload(**data_dict)
            return payload

        except ValidationError as e:
            self.logger.error(
                f"Validation error deserializing event payload: {str(e)}",
                error=str(e),
                message_data=message_data
            )
            return None
        except json.JSONDecodeError as e:
            self.logger.error(
                f"JSON decode error deserializing event payload: {str(e)}",
                error=str(e),
                message_data=message_data
            )
            return None
        except Exception as e:
            self.logger.error(
                f"Error deserializing event payload: {str(e)}",
                error=str(e),
                message_data=message_data
            )
            return None

    def _handle_message(self, message: Dict[str, Any]):
        """
        Handle an incoming message from Redis pubsub.

        Args:
            message: Dictionary containing message data from Redis
        """
        if message['type'] != 'message':
            return  # Ignore non-message events

        try:
            # Get the channel name
            channel = message['channel'].decode('utf-8')

            # Get the message data
            message_data = message['data'].decode('utf-8')

            self.logger.debug(
                f"Received message from channel {channel}",
                channel=channel,
                message_length=len(message_data)
            )

            # First, try to handle as a structured event
            payload = self._deserialize_payload(message_data)

            if payload:
                # Handle structured event
                self._handle_structured_event(payload)
            else:
                # Handle as raw channel message
                self._handle_channel_message(channel, message_data)

        except Exception as e:
            self.logger.error(
                f"Error handling message: {str(e)}",
                error=str(e)
            )

    def _handle_structured_event(self, payload: EventPayload):
        """
        Handle a structured event with EventPayload.

        Args:
            payload: EventPayload instance
        """
        event_type = payload.event_type

        # Call handlers for this specific event type
        if event_type in self.event_handlers:
            for handler in self.event_handlers[event_type]:
                try:
                    handler(payload)
                    self.logger.debug(
                        f"Called handler for event {event_type}",
                        event_type=event_type,
                        handler_name=handler.__name__ if hasattr(handler, '__name__') else str(handler)
                    )
                except Exception as e:
                    self.logger.error(
                        f"Error in event handler for {event_type}: {str(e)}",
                        error=str(e),
                        event_type=event_type,
                        handler_name=handler.__name__ if hasattr(handler, '__name__') else str(handler)
                    )

    def _handle_channel_message(self, channel: str, message_data: str):
        """
        Handle a raw channel message.

        Args:
            channel: Name of the channel
            message_data: Raw message data
        """
        # Call handlers for this specific channel
        if channel in self.channel_handlers:
            for handler in self.channel_handlers[channel]:
                try:
                    handler(channel, message_data)
                    self.logger.debug(
                        f"Called channel handler for {channel}",
                        channel=channel,
                        handler_name=handler.__name__ if hasattr(handler, '__name__') else str(handler)
                    )
                except Exception as e:
                    self.logger.error(
                        f"Error in channel handler for {channel}: {str(e)}",
                        error=str(e),
                        channel=channel,
                        handler_name=handler.__name__ if hasattr(handler, '__name__') else str(handler)
                    )

    def subscribe_to_channels(self, channels: List[str]):
        """
        Subscribe to specific Redis channels.

        Args:
            channels: List of channel names to subscribe to
        """
        self.pubsub.subscribe(*channels)
        self.logger.info(f"Subscribed to channels: {channels}", channels=channels)

    def unsubscribe_from_channels(self, channels: List[str]):
        """
        Unsubscribe from specific Redis channels.

        Args:
            channels: List of channel names to unsubscribe from
        """
        self.pubsub.unsubscribe(*channels)
        self.logger.info(f"Unsubscribed from channels: {channels}", channels=channels)

    def _listen_loop(self):
        """
        Internal method that runs the listening loop.
        """
        self.logger.info("Starting event listener loop")
        self.is_listening = True

        try:
            for message in self.pubsub.listen():
                if not self.is_listening:
                    break
                self._handle_message(message)
        except Exception as e:
            self.logger.error(f"Error in event listener loop: {str(e)}", error=str(e))
        finally:
            self.is_listening = False
            self.logger.info("Event listener loop stopped")

    def start_listening(self, channels: List[str] = None):
        """
        Start listening for events in a background thread.

        Args:
            channels: List of channels to listen to (if None, uses default channels)
        """
        if self.is_listening:
            self.logger.warning("Event listener is already running")
            return

        # Subscribe to default channels if none provided
        if channels is None:
            channels = [self.default_channel, "crawl_events", "chunking_events", "embedding_events", "health_events", "content_events", "error_events"]

        self.subscribe_to_channels(channels)

        # Start the listening thread
        self.listener_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listener_thread.start()

        self.logger.info("Started event listener in background thread", channels=channels)

    def stop_listening(self):
        """
        Stop the event listener.
        """
        self.logger.info("Stopping event listener")
        self.is_listening = False

        if self.listener_thread and self.listener_thread.is_alive():
            self.listener_thread.join(timeout=5.0)  # Wait up to 5 seconds for graceful shutdown

        # Unsubscribe from all channels
        self.pubsub.close()

        self.logger.info("Event listener stopped")

    def is_running(self) -> bool:
        """
        Check if the event listener is currently running.

        Returns:
            True if running, False otherwise
        """
        return self.is_listening and (self.listener_thread and self.listener_thread.is_alive())

    def get_subscribed_channels(self) -> List[str]:
        """
        Get list of currently subscribed channels.

        Returns:
            List of channel names
        """
        try:
            subscriptions = self.pubsub.channels
            return [channel.decode('utf-8') for channel in subscriptions.keys()]
        except Exception as e:
            self.logger.error(f"Error getting subscribed channels: {str(e)}", error=str(e))
            return []


class CrawlJobEventListener:
    """
    Specialized event listener for crawl job events with built-in handlers.
    """

    def __init__(self, event_listener: EventListener = None):
        self.logger = get_logger("crawl_job_event_listener")
        self.event_listener = event_listener or EventListener()

        # Register built-in handlers for crawl job events
        self._register_crawl_job_handlers()

    def _register_crawl_job_handlers(self):
        """
        Register built-in handlers for crawl job events.
        """
        self.event_listener.register_handler(EventType.CRAWL_JOB_STARTED, self._on_crawl_job_started)
        self.event_listener.register_handler(EventType.CRAWL_JOB_COMPLETED, self._on_crawl_job_completed)
        self.event_listener.register_handler(EventType.CRAWL_JOB_FAILED, self._on_crawl_job_failed)
        self.event_listener.register_handler(EventType.CRAWL_JOB_PROGRESS, self._on_crawl_job_progress)

    def _on_crawl_job_started(self, payload: EventPayload):
        """
        Handle crawl job started event.

        Args:
            payload: Event payload
        """
        job_id = payload.job_id
        data = payload.data
        target_url = data.get('target_url', 'unknown')

        self.logger.info(
            f"Crawl job {job_id} started for {target_url}",
            job_id=job_id,
            target_url=target_url,
            start_time=data.get('start_time')
        )

        # Additional processing can be added here
        # For example: update job status in database, send notifications, etc.

    def _on_crawl_job_completed(self, payload: EventPayload):
        """
        Handle crawl job completed event.

        Args:
            payload: Event payload
        """
        job_id = payload.job_id
        data = payload.data
        result = data.get('result', {})

        self.logger.info(
            f"Crawl job {job_id} completed",
            job_id=job_id,
            result=result,
            completion_time=data.get('completion_time')
        )

        # Additional processing can be added here
        # For example: update job status in database, send notifications, etc.

    def _on_crawl_job_failed(self, payload: EventPayload):
        """
        Handle crawl job failed event.

        Args:
            payload: Event payload
        """
        job_id = payload.job_id
        data = payload.data
        error_message = data.get('error_message', 'Unknown error')

        self.logger.error(
            f"Crawl job {job_id} failed: {error_message}",
            job_id=job_id,
            error_message=error_message,
            error_details=data.get('error_details'),
            failure_time=data.get('failure_time')
        )

        # Additional processing can be added here
        # For example: update job status in database, send alerts, etc.

    def _on_crawl_job_progress(self, payload: EventPayload):
        """
        Handle crawl job progress event.

        Args:
            payload: Event payload
        """
        job_id = payload.job_id
        data = payload.data
        progress_data = data.get('progress_data', {})

        self.logger.info(
            f"Crawl job {job_id} progress update",
            job_id=job_id,
            progress_data=progress_data,
            update_time=data.get('update_time')
        )

        # Additional processing can be added here
        # For example: update progress in database, send notifications, etc.

    def start_listening(self, channels: List[str] = None):
        """
        Start listening for crawl job events.

        Args:
            channels: List of channels to listen to
        """
        self.event_listener.start_listening(channels)

    def stop_listening(self):
        """
        Stop listening for crawl job events.
        """
        self.event_listener.stop_listening()

    def is_running(self) -> bool:
        """
        Check if the crawl job event listener is running.

        Returns:
            True if running, False otherwise
        """
        return self.event_listener.is_running()


def create_default_event_listener() -> EventListener:
    """
    Create a default event listener instance.

    Returns:
        EventListener instance
    """
    return EventListener()


def create_default_crawl_job_event_listener() -> CrawlJobEventListener:
    """
    Create a default crawl job event listener instance.

    Returns:
        CrawlJobEventListener instance
    """
    return CrawlJobEventListener()