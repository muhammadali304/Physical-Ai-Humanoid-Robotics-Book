"""
Query history storage service for the RAG Agent Backend.

This module provides functionality for storing and retrieving user query history
following the implementation plan requirements.
"""

import asyncio
import json
from typing import List, Optional, Dict, Any
from datetime import datetime
from uuid import UUID
import logging
from src.models.session import QueryRecord, UserSession
from src.models.query import QueryRequest, QueryResponse


class QueryHistoryService:
    """Service class for managing query history storage and retrieval"""

    def __init__(self):
        """Initialize the query history service"""
        self.logger = logging.getLogger(__name__)
        self.history_storage = {}  # In-memory storage (would be DB in production)
        self.max_history_length = 100  # Maximum number of queries per session
        self.retention_days = 30  # How long to retain history in days

    async def store_query(self, session_id: str, query_request: QueryRequest, query_response: QueryResponse) -> bool:
        """Store a query and its response in the history"""
        try:
            self.logger.info(f"Storing query for session: {session_id}")

            # Create a QueryRecord
            query_record = QueryRecord(
                query_id=str(UUID(int=hash(f"{session_id}_{datetime.now().isoformat()}"))),  # Generate unique ID
                query_text=query_request.query,
                response_id=str(UUID(int=hash(f"{session_id}_response_{datetime.now().isoformat()}"))),  # Generate unique response ID
                timestamp=datetime.now().isoformat(),
                metadata=query_request.metadata
            )

            # Add to session history
            if session_id not in self.history_storage:
                self.history_storage[session_id] = []

            # Add the record to the history
            self.history_storage[session_id].append(query_record)

            # Trim history if it exceeds the maximum length
            if len(self.history_storage[session_id]) > self.max_history_length:
                self.history_storage[session_id] = self.history_storage[session_id][-self.max_history_length:]

            self.logger.info(f"Successfully stored query in session {session_id}")
            return True

        except Exception as e:
            self.logger.error(f"Error storing query for session {session_id}: {str(e)}")
            return False

    async def get_session_history(self, session_id: str) -> List[QueryRecord]:
        """Retrieve the query history for a specific session"""
        try:
            self.logger.info(f"Retrieving query history for session: {session_id}")

            if session_id in self.history_storage:
                history = self.history_storage[session_id]
                self.logger.info(f"Retrieved {len(history)} queries for session {session_id}")
                return history
            else:
                self.logger.info(f"No history found for session: {session_id}")
                return []

        except Exception as e:
            self.logger.error(f"Error retrieving query history for session {session_id}: {str(e)}")
            return []

    async def get_recent_queries(self, session_id: str, limit: int = 10) -> List[QueryRecord]:
        """Get the most recent queries for a session, up to the limit"""
        try:
            self.logger.info(f"Retrieving {limit} most recent queries for session: {session_id}")

            history = await self.get_session_history(session_id)
            recent_queries = history[-limit:] if len(history) >= limit else history

            self.logger.info(f"Retrieved {len(recent_queries)} recent queries for session {session_id}")
            return recent_queries

        except Exception as e:
            self.logger.error(f"Error retrieving recent queries for session {session_id}: {str(e)}")
            return []

    async def clear_session_history(self, session_id: str) -> bool:
        """Clear the query history for a specific session"""
        try:
            self.logger.info(f"Clearing query history for session: {session_id}")

            if session_id in self.history_storage:
                del self.history_storage[session_id]
                self.logger.info(f"Successfully cleared history for session {session_id}")
                return True
            else:
                self.logger.info(f"No history found to clear for session {session_id}")
                return True  # Considered successful if nothing to clear

        except Exception as e:
            self.logger.error(f"Error clearing query history for session {session_id}: {str(e)}")
            return False

    async def get_all_sessions(self) -> List[str]:
        """Get a list of all session IDs that have query history"""
        try:
            self.logger.info("Retrieving all session IDs with query history")

            sessions = list(self.history_storage.keys())
            self.logger.info(f"Found {len(sessions)} sessions with query history")
            return sessions

        except Exception as e:
            self.logger.error(f"Error retrieving session list: {str(e)}")
            return []

    async def purge_expired_history(self) -> int:
        """Purge query history that has exceeded the retention period"""
        try:
            self.logger.info("Purging expired query history")

            current_time = datetime.now()
            expired_count = 0
            sessions_to_delete = []

            for session_id, records in self.history_storage.items():
                # Check each record's timestamp to see if it exceeds retention period
                records_to_keep = []
                for record in records:
                    # Parse the timestamp from the record
                    record_time = datetime.fromisoformat(record.timestamp.replace('Z', '+00:00'))
                    time_diff = (current_time - record_time).days

                    if time_diff <= self.retention_days:
                        # Keep records that are within the retention period
                        records_to_keep.append(record)
                    else:
                        # Count expired records
                        expired_count += 1

                # Update the session with only non-expired records
                if records_to_keep:
                    self.history_storage[session_id] = records_to_keep
                else:
                    # If no records remain, mark session for deletion
                    sessions_to_delete.append(session_id)

            # Remove sessions that have no remaining records
            for session_id in sessions_to_delete:
                if session_id in self.history_storage:
                    del self.history_storage[session_id]

            self.logger.info(f"Purged {expired_count} expired query history records")
            return expired_count

        except Exception as e:
            self.logger.error(f"Error purging expired query history: {str(e)}")
            return 0

    async def get_query_statistics(self, session_id: str) -> Dict[str, Any]:
        """Get statistics about queries for a specific session"""
        try:
            self.logger.info(f"Retrieving query statistics for session: {session_id}")

            history = await self.get_session_history(session_id)
            if not history:
                return {
                    "session_id": session_id,
                    "total_queries": 0,
                    "first_query": None,
                    "last_query": None,
                    "avg_query_length": 0
                }

            # Calculate statistics
            total_queries = len(history)
            first_query_time = history[0].timestamp if history else None
            last_query_time = history[-1].timestamp if history else None

            total_query_length = sum(len(record.query_text) for record in history)
            avg_query_length = total_query_length / total_queries if total_queries > 0 else 0

            stats = {
                "session_id": session_id,
                "total_queries": total_queries,
                "first_query": first_query_time,
                "last_query": last_query_time,
                "avg_query_length": round(avg_query_length, 2),
                "history_length": len(history)
            }

            self.logger.info(f"Retrieved statistics for session {session_id}")
            return stats

        except Exception as e:
            self.logger.error(f"Error retrieving query statistics for session {session_id}: {str(e)}")
            return {
                "session_id": session_id,
                "total_queries": 0,
                "first_query": None,
                "last_query": None,
                "avg_query_length": 0
            }

    async def search_in_history(self, session_id: str, search_term: str) -> List[QueryRecord]:
        """Search for queries in the history that contain a search term"""
        try:
            self.logger.info(f"Searching for '{search_term}' in session {session_id} history")

            history = await self.get_session_history(session_id)
            matching_records = []

            for record in history:
                if search_term.lower() in record.query_text.lower():
                    matching_records.append(record)

            self.logger.info(f"Found {len(matching_records)} matching queries for '{search_term}' in session {session_id}")
            return matching_records

        except Exception as e:
            self.logger.error(f"Error searching in query history for session {session_id}: {str(e)}")
            return []

    async def update_retention_policy(self, retention_days: int) -> bool:
        """Update the retention policy for query history"""
        try:
            self.logger.info(f"Updating retention policy to {retention_days} days")

            self.retention_days = retention_days
            self.logger.info(f"Retention policy updated to {retention_days} days")
            return True

        except Exception as e:
            self.logger.error(f"Error updating retention policy: {str(e)}")
            return False

    async def export_session_history(self, session_id: str) -> Optional[str]:
        """Export the session history as JSON string"""
        try:
            self.logger.info(f"Exporting history for session: {session_id}")

            history = await self.get_session_history(session_id)
            if not history:
                return None

            # Convert to serializable format
            history_data = []
            for record in history:
                history_data.append({
                    "query_id": record.query_id,
                    "query_text": record.query_text,
                    "response_id": record.response_id,
                    "timestamp": record.timestamp,
                    "metadata": record.metadata
                })

            export_data = {
                "session_id": session_id,
                "export_timestamp": datetime.now().isoformat(),
                "query_records": history_data
            }

            export_json = json.dumps(export_data, indent=2)
            self.logger.info(f"Exported {len(history)} queries for session {session_id}")
            return export_json

        except Exception as e:
            self.logger.error(f"Error exporting session history for {session_id}: {str(e)}")
            return None


# Global query history service instance
query_history_service = QueryHistoryService()


def get_query_history_service() -> QueryHistoryService:
    """Get the global query history service instance"""
    return query_history_service