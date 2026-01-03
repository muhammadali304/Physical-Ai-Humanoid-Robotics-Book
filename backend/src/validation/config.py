from pydantic_settings import BaseSettings
from pydantic import Field
from typing import Optional


class ValidationSettings(BaseSettings):
    # Qdrant Configuration
    qdrant_url: str = Field(default="http://localhost:6333", alias="QDRANT_URL")
    qdrant_api_key: Optional[str] = Field(default=None, alias="QDRANT_API_KEY")
    qdrant_collection_name: str = Field(default="embedded_content", alias="QDRANT_COLLECTION_NAME")

    # Cohere Configuration
    cohere_api_key: str = Field(default="your_cohere_api_key_here", alias="COHERE_API_KEY")  # Default for testing
    cohere_model: str = Field(default="embed-multilingual-v3.0", alias="COHERE_MODEL")

    # Validation Configuration
    validation_min_score_threshold: float = 0.3
    validation_top_k_default: int = 5
    validation_batch_size: int = 10
    validation_concurrency: int = 5
    validation_timeout: float = 30.0
    validation_max_execution_time: float = 300.0  # 5 minutes

    # API Configuration
    validation_api_key: str = Field(default="test_api_key", alias="VALIDATION_API_KEY")  # Default for testing
    validation_rate_limit: str = "100/minute"

    class Config:
        env_file = ".env"
        case_sensitive = False  # Allow both cases
        extra = "ignore"  # Ignore extra fields that don't match


settings = ValidationSettings()