from pydantic import BaseModel, Field, validator
from typing import Optional, Dict, Any
from datetime import datetime
from uuid import UUID, uuid4


class ContentChunk(BaseModel):
    """
    Represents a semantically meaningful segment of documentation text that has been
    processed and prepared for vector storage.
    """
    id: UUID = Field(default_factory=uuid4)
    source_url: str = Field(..., description="Original URL from which the content was extracted")
    page_title: str = Field(..., max_length=500, description="Title of the source page")
    section_heading: Optional[str] = Field(None, max_length=500, description="Heading under which this content appears")
    chunk_index: int = Field(..., description="Sequential position of this chunk within the source document")
    content: str = Field(..., description="The raw text content of this chunk")
    token_count: int = Field(..., description="Number of tokens in the content (for size validation)")
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Additional metadata including extraction information")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Timestamp when the chunk was created")
    updated_at: datetime = Field(default_factory=datetime.utcnow, description="Timestamp when the chunk was last updated")

    @validator('source_url')
    def validate_source_url(cls, v):
        """Validate that the source URL is properly formatted."""
        if not v or not v.startswith(('http://', 'https://')):
            raise ValueError('source_url must be a valid URL starting with http:// or https://')
        return v

    @validator('content')
    def validate_content_length(cls, v):
        """Validate that content meets minimum length requirements."""
        if len(v) < 50:
            raise ValueError('Content must be at least 50 characters long')
        return v

    @validator('token_count')
    def validate_token_count(cls, v):
        """Validate that token count is within acceptable range."""
        if v < 50 or v > 1000:
            raise ValueError('Token count must be between 50 and 1000 tokens')
        return v

    def __init__(self, **data):
        super().__init__(**data)
        self.updated_at = datetime.utcnow()