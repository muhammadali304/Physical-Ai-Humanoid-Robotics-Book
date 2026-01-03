from pydantic import BaseModel, Field, validator
from typing import List, Optional
from datetime import datetime
from uuid import UUID, uuid4


class EmbeddingVector(BaseModel):
    """
    Represents the semantic embedding vector for a content chunk, stored in the vector database.
    """
    id: UUID = Field(default_factory=uuid4)
    content_chunk_id: UUID = Field(..., description="Reference to the associated ContentChunk")
    vector_data: List[float] = Field(..., description="The embedding vector values (1024-dimensional for Cohere v3)")
    model_used: str = Field(..., description="Identifier of the embedding model used (e.g., 'cohere/embed-multilingual-v3.0')")
    model_version: str = Field(default="v3", description="Version of the embedding model")
    dimensions: int = Field(default=1024, description="Number of dimensions in the vector")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Timestamp when the embedding was generated")

    @validator('vector_data')
    def validate_vector_data(cls, v):
        """Validate that vector data has the correct dimensions."""
        if len(v) != 1024:  # Expected dimension for Cohere v3 model
            raise ValueError('Vector data must have exactly 1024 dimensions for Cohere v3 model')
        return v

    @validator('model_used')
    def validate_model_used(cls, v):
        """Validate that model used is a supported embedding model."""
        supported_models = [
            "cohere/embed-multilingual-v3.0",
            "cohere/embed-english-v3.0",
            "cohere/embed-multilingual-light-v3.0",
            "cohere/embed-english-light-v3.0"
        ]
        if v not in supported_models:
            raise ValueError(f'Model must be one of: {supported_models}')
        return v

    @validator('dimensions')
    def validate_dimensions(cls, v):
        """Validate that dimensions match the expected size for the model."""
        if v != 1024:
            raise ValueError('Dimensions must be 1024 for Cohere v3 model')
        return v

    def __init__(self, **data):
        super().__init__(**data)
        # Ensure dimensions match vector length
        if 'vector_data' in data and len(data['vector_data']) != data.get('dimensions', 1024):
            raise ValueError('Vector data length must match dimensions')