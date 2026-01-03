"""
Query-related data models for the RAG Agent Backend.

This module contains Pydantic models for query requests, responses, and related entities
following the specifications from the data-model.md file.
"""

from pydantic import BaseModel, Field
from pydantic.v1 import validator
from typing import List, Optional, Dict, Any
from datetime import datetime
import uuid


class SourceReference(BaseModel):
    """Reference to a source document used in the response"""
    document_id: str = Field(..., description="Unique identifier for the source document")
    title: str = Field(..., description="Title of the source document")
    url: Optional[str] = Field(None, description="URL to the source document")
    page: Optional[int] = Field(None, description="Page number (for multi-page documents)")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Relevance score (0.0 to 1.0)")
    text_snippet: Optional[str] = Field(None, description="Snippet of text from the source")

    @validator('relevance_score')
    def validate_relevance_score(cls, v):
        if not 0.0 <= v <= 1.0:
            raise ValueError('Relevance score must be between 0.0 and 1.0')
        return v


class TokenUsage(BaseModel):
    """Information about token consumption during query processing"""
    input_tokens: int = Field(..., ge=0, description="Number of tokens in the input query")
    output_tokens: int = Field(..., ge=0, description="Number of tokens in the generated response")
    total_tokens: int = Field(..., ge=0, description="Total tokens consumed")

    @validator('input_tokens', 'output_tokens', 'total_tokens')
    def validate_non_negative(cls, v):
        if v < 0:
            raise ValueError('Token counts must be non-negative')
        return v

    # Removed strict validation that requires total_tokens to equal input_tokens + output_tokens
    # Some LLM APIs report total tokens that include additional tokens beyond just input+output
    # such as cached tokens, system tokens, or other overhead


class RetrievalInfo(BaseModel):
    """Information about the document retrieval process"""
    retrieved_chunks: int = Field(..., ge=0, description="Number of document chunks retrieved")
    search_time_ms: float = Field(..., gt=0, description="Time taken for retrieval in milliseconds")
    top_k: int = Field(..., gt=0, description="Number of top results requested")
    relevance_threshold: Optional[float] = Field(None, ge=0.0, le=1.0, description="Minimum relevance score")


class QueryRequest(BaseModel):
    """Input model for user queries to the RAG system"""
    query: str = Field(..., min_length=1, max_length=2000, description="The user's natural language question")
    session_id: Optional[str] = Field(None, description="Identifier for conversation context")
    user_id: Optional[str] = Field(None, description="Identifier for authenticated user")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional context or parameters")
    selected_text: Optional[str] = Field(None, description="Selected text from the page that provides additional context")

    class Config:
        extra = "allow"  # Allow additional fields that are not explicitly defined

    @validator('session_id', 'user_id')
    def validate_identifiers(cls, v):
        if v is not None:
            # Validate that it's a properly formatted identifier
            try:
                uuid.UUID(v)
            except ValueError:
                raise ValueError('Identifier must be a valid UUID')
        return v


class QueryResponse(BaseModel):
    """Output model for responses from the RAG system"""
    response: str = Field(..., min_length=1, description="The generated answer to the user's query")
    sources: List[SourceReference] = Field(default_factory=list, description="List of documents used in generating the response")
    session_id: Optional[str] = Field(None, description="Identifier for conversation context")
    tokens_used: Optional[TokenUsage] = Field(None, description="Information about token consumption")
    retrieval_info: Optional[RetrievalInfo] = Field(None, description="Details about the retrieval process")

    @validator('session_id')
    def validate_session_id(cls, v):
        if v is not None:
            try:
                uuid.UUID(v)
            except ValueError:
                raise ValueError('Session ID must be a valid UUID')
        return v