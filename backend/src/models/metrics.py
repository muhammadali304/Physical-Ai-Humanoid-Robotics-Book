"""
Metrics-related data models for the RAG Agent Backend.

This module contains Pydantic models for token usage, retrieval info, and other metrics
following the specifications from the data-model.md file.
"""

from pydantic import BaseModel, Field, validator
from typing import List, Optional, Dict, Any
from datetime import datetime
import uuid


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

    @validator('total_tokens')
    def validate_total_tokens(cls, v, values):
        if 'input_tokens' in values and 'output_tokens' in values:
            expected_total = values['input_tokens'] + values['output_tokens']
            if v != expected_total:
                raise ValueError(f'total_tokens must equal input_tokens + output_tokens ({expected_total})')
        return v


class RetrievalInfo(BaseModel):
    """Information about the document retrieval process"""
    retrieved_chunks: int = Field(..., ge=0, description="Number of document chunks retrieved")
    search_time_ms: float = Field(..., gt=0, description="Time taken for retrieval in milliseconds")
    top_k: int = Field(..., gt=0, description="Number of top results requested")
    relevance_threshold: Optional[float] = Field(None, ge=0.0, le=1.0, description="Minimum relevance score for inclusion")

    @validator('retrieved_chunks')
    def validate_retrieved_chunks(cls, v):
        if v < 0:
            raise ValueError('Retrieved chunks must be non-negative')
        return v

    @validator('search_time_ms')
    def validate_search_time(cls, v):
        if v <= 0:
            raise ValueError('Search time must be positive')
        return v

    @validator('top_k')
    def validate_top_k(cls, v):
        if v <= 0:
            raise ValueError('Top-k value must be positive')
        return v

    @validator('relevance_threshold')
    def validate_relevance_threshold(cls, v):
        if v is not None and not 0.0 <= v <= 1.0:
            raise ValueError('Relevance threshold must be between 0.0 and 1.0')
        return v


class QueryMetrics(BaseModel):
    """Comprehensive metrics for a query operation"""
    query_id: str = Field(..., description="Unique identifier for the query")
    response_time_ms: float = Field(..., gt=0, description="Total time to process the query in milliseconds")
    tokens_used: TokenUsage = Field(..., description="Token usage information")
    retrieval_info: RetrievalInfo = Field(..., description="Retrieval information")
    success: bool = Field(..., description="Whether the query was successful")
    timestamp: str = Field(default_factory=lambda: datetime.now().isoformat(), description="Timestamp of the query")
    error_message: Optional[str] = Field(None, description="Error message if the query failed")


class SystemMetrics(BaseModel):
    """System-wide metrics for monitoring"""
    active_sessions: int = Field(default=0, ge=0, description="Number of active user sessions")
    queries_per_minute: float = Field(default=0.0, ge=0.0, description="Average queries per minute")
    average_response_time: float = Field(default=0.0, ge=0.0, description="Average response time in milliseconds")
    success_rate: float = Field(default=1.0, ge=0.0, le=1.0, description="Success rate (0.0 to 1.0)")
    timestamp: str = Field(default_factory=lambda: datetime.now().isoformat(), description="Timestamp of metrics collection")