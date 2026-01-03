"""
Configuration constants for the RAG Ingestion Pipeline.
These are fixed values that don't change based on environment.
"""

# Crawler Constants
DEFAULT_MAX_DEPTH = 3
DEFAULT_RATE_LIMIT_DELAY = 1.0  # seconds
DEFAULT_USER_AGENT = "RAG-Ingestion-Bot/1.0"
DEFAULT_MAX_WORKERS = 4

# Content Processing Constants
DEFAULT_MAX_CHUNK_SIZE = 512  # tokens
DEFAULT_CHUNK_OVERLAP = 0.2  # 20% overlap
DEFAULT_MIN_CONTENT_LENGTH = 50  # minimum characters for valid content
MAX_TOKEN_COUNT = 1000  # maximum tokens per chunk

# Qdrant Constants
DEFAULT_QDRANT_COLLECTION_NAME = "doc_embeddings"
DEFAULT_VECTOR_DIMENSIONS = 1024  # Cohere v3 model
DEFAULT_SEARCH_TOP_K = 5
COSINE_SIMILARITY = "Cosine"

# Embedding Model Constants
COHERE_EMBED_MULTILINGUAL_V3 = "cohere/embed-multilingual-v3.0"
COHERE_EMBED_ENGLISH_V3 = "cohere/embed-english-v3.0"
SUPPORTED_EMBEDDING_MODELS = [
    COHERE_EMBED_MULTILINGUAL_V3,
    COHERE_EMBED_ENGLISH_V3,
    "cohere/embed-multilingual-light-v3.0",
    "cohere/embed-english-light-v3.0"
]

# API and Network Constants
DEFAULT_TIMEOUT = 30  # seconds
DEFAULT_MAX_RETRIES = 3
DEFAULT_RETRY_DELAY = 1  # seconds
DEFAULT_BACKOFF_FACTOR = 2

# Queue and Job Processing Constants
DEFAULT_QUEUE_NAME = "default"
DEFAULT_JOB_TIMEOUT = 300  # seconds (5 minutes)
DEFAULT_JOB_RESULT_TTL = 86400  # seconds (24 hours)

# Logging Constants
DEFAULT_LOG_FORMAT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
DEFAULT_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"

# Validation Constants
MIN_URL_LENGTH = 10
MAX_URL_LENGTH = 2048
MIN_CONTENT_LENGTH = 50
MAX_CONTENT_LENGTH = 100000  # 100KB max content per chunk

# Status and Progress Constants
MIN_PROGRESS_VALUE = 0
MAX_PROGRESS_VALUE = 100

# File and Path Constants
DEFAULT_FILE_ENCODING = "utf-8"
SUPPORTED_FILE_EXTENSIONS = [".html", ".htm", ".md", ".txt"]