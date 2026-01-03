"""
Session-related data models for the RAG Agent Backend.

This module contains Pydantic models for user sessions and query records
following the specifications from the data-model.md file.
"""

from pydantic import BaseModel, Field, validator
from typing import List, Optional, Dict, Any
from datetime import datetime
import uuid


class QueryRecord(BaseModel):
    """Record of a single query within a session"""
    query_id: str = Field(..., description="Unique identifier for this query")
    query_text: str = Field(..., min_length=1, description="The original query text")
    response_id: str = Field(..., description="Reference to the response")
    timestamp: str = Field(default_factory=lambda: datetime.now().isoformat(), description="When the query was made")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional metadata about the query")

    @validator('timestamp')
    def validate_timestamp(cls, v):
        try:
            datetime.fromisoformat(v.replace('Z', '+00:00'))
        except ValueError:
            raise ValueError('Timestamp must be in ISO 8601 format')
        return v


class UserSession(BaseModel):
    """Information about a user's session with the RAG system"""
    session_id: str = Field(..., description="Unique identifier for the session")
    user_id: Optional[str] = Field(None, description="Reference to the authenticated user")
    created_at: str = Field(default_factory=lambda: datetime.now().isoformat(), description="Timestamp of session creation")
    last_accessed_at: str = Field(default_factory=lambda: datetime.now().isoformat(), description="Timestamp of last access")
    query_history: List[QueryRecord] = Field(default_factory=list, description="History of queries in this session")

    @validator('last_accessed_at')
    def validate_timestamps(cls, v, values):
        if 'created_at' in values:
            created = datetime.fromisoformat(values['created_at'].replace('Z', '+00:00'))
            accessed = datetime.fromisoformat(v.replace('Z', '+00:00'))
            if accessed < created:
                raise ValueError('Last accessed time must be after creation time')
        return v

    @validator('session_id', 'user_id')
    def validate_identifiers(cls, v):
        if v is not None:
            # Validate that it's a properly formatted identifier
            try:
                uuid.UUID(v)
            except ValueError:
                raise ValueError('Identifier must be a valid UUID')
        return v


class SessionConfig(BaseModel):
    """Configuration for a user session"""
    session_id: str = Field(..., description="Session identifier")
    max_history_length: int = Field(default=10, ge=1, le=100, description="Maximum number of queries to keep in history")
    timeout_minutes: int = Field(default=30, ge=1, le=1440, description="Session timeout in minutes")
    enable_history: bool = Field(default=True, description="Whether to store query history")
    retention_days: int = Field(default=30, ge=1, le=365, description="How long to retain session data")