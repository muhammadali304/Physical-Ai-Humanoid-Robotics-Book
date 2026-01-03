"""
Document-related data models for the RAG Agent Backend.

This module contains Pydantic models for document chunks and related entities
following the specifications from the data-model.md file.
"""

from pydantic import BaseModel, Field, validator
from typing import List, Optional, Dict, Any
from datetime import datetime
import uuid


class DocumentChunk(BaseModel):
    """Individual chunks of ingested documents stored in the vector database"""
    id: str = Field(..., description="Unique identifier for this chunk")
    content: str = Field(..., min_length=1, description="The text content of this chunk")
    embedding: List[float] = Field(..., description="Vector representation for semantic search")
    document_id: str = Field(..., description="Reference to the original document")
    document_title: str = Field(..., description="Title of the original document")
    document_url: Optional[str] = Field(None, description="URL of the original document")
    chunk_index: int = Field(..., ge=0, description="Position of this chunk in the original document")
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Additional metadata about the chunk")
    created_at: str = Field(default_factory=lambda: datetime.now().isoformat(), description="Timestamp of creation")
    updated_at: str = Field(default_factory=lambda: datetime.now().isoformat(), description="Timestamp of last update")

    @validator('chunk_index')
    def validate_chunk_index(cls, v):
        if v < 0:
            raise ValueError('Chunk index must be non-negative')
        return v

    @validator('embedding')
    def validate_embedding(cls, v):
        # We'll validate that it has the expected dimensions later when we know the model
        if not isinstance(v, list) or len(v) == 0:
            raise ValueError('Embedding must be a non-empty list of floats')
        return v

    @validator('content')
    def validate_content(cls, v):
        if not v.strip():
            raise ValueError('Content must not be empty or just whitespace')
        return v


class DocumentMetadata(BaseModel):
    """Metadata for a document"""
    document_id: str = Field(..., description="Unique identifier for the document")
    title: str = Field(..., description="Title of the document")
    url: Optional[str] = Field(None, description="URL of the document")
    source_type: str = Field(..., description="Type of the source (e.g., pdf, html, text)")
    created_at: str = Field(default_factory=lambda: datetime.now().isoformat(), description="Timestamp of creation")
    updated_at: str = Field(default_factory=lambda: datetime.now().isoformat(), description="Timestamp of last update")
    tags: List[str] = Field(default_factory=list, description="Tags for categorization")
    access_permissions: List[str] = Field(default_factory=list, description="Access permissions for the document")