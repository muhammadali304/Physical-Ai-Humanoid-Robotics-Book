from pydantic_settings import BaseSettings
from pydantic import Field
from typing import Optional


class Settings(BaseSettings):
    # API Keys and URLs
    cohere_api_key: str = Field(..., alias="COHERE_API_KEY")
    groq_api_key: str = Field(..., alias="GROQ_API_KEY")
    qdrant_url: str = Field(..., alias="QDRANT_URL")
    qdrant_api_key: str = Field(..., alias="QDRANT_API_KEY")

    # Crawler settings
    max_depth: int = Field(default=3, alias="MAX_DEPTH")
    rate_limit_delay: float = Field(default=1.0, alias="RATE_LIMIT_DELAY")  # seconds between requests
    user_agent: str = Field(default="RAG-Ingestion-Bot/1.0", alias="USER_AGENT")
    max_workers: int = Field(default=4, alias="MAX_WORKERS")
    max_retries: int = 3  # maximum number of retries for failed requests

    # Processing settings
    max_chunk_size: int = Field(default=512, alias="MAX_CHUNK_SIZE")  # tokens
    chunk_overlap: float = Field(default=0.2, alias="CHUNK_OVERLAP")  # 20% overlap
    min_content_length: int = 50  # minimum characters for valid content

    # Qdrant settings
    qdrant_collection_name: str = Field(default="doc_embeddings", alias="QDRANT_COLLECTION_NAME")
    vector_dimensions: int = Field(default=1024, alias="VECTOR_DIMENSIONS")  # Cohere v3 model
    search_top_k: int = 5

    # Redis settings for queues
    redis_host: str = "localhost"
    redis_port: int = 6379
    redis_db: int = 0
    redis_password: Optional[str] = None

    # Logging
    log_level: str = Field(default="INFO", alias="LOG_LEVEL")
    log_format: str = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    log_json_format: bool = True
    log_file: Optional[str] = None
    log_max_bytes: int = 10485760  # 10MB
    log_backup_count: int = 5
    log_include_traceback: bool = True

    # Alerting
    alert_webhook_urls: list = []
    smtp_server: str = "localhost"
    smtp_port: int = 587
    smtp_username: Optional[str] = None
    smtp_password: Optional[str] = None
    from_email: str = "noreply@rag-pipeline.com"
    alert_recipients: list = []
    alert_response_time_threshold: float = 5000.0  # 5 seconds in ms
    alert_error_rate_threshold: float = 0.05  # 5%
    alert_memory_threshold: float = 80.0  # 80%
    alert_disk_threshold: float = 90.0  # 90%

    # Backup and Recovery
    backup_directory: str = "./backups"
    max_backup_age_days: int = 30
    backup_compression_enabled: bool = True
    backup_schedule_cron: str = "0 2 * * *"  # Daily at 2 AM

    # Application
    app_name: str = "RAG Ingestion Pipeline"
    debug: bool = False

    class Config:
        env_file = ".env"
        case_sensitive = False  # Allow both cases
        extra = "ignore"  # Ignore extra fields that don't match


# Create a singleton instance of settings
settings = Settings()