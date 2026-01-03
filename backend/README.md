# RAG Ingestion Pipeline

The RAG Ingestion Pipeline is a comprehensive system for crawling documentation sites, extracting clean textual content, chunking it semantically, generating embeddings using Cohere models, and storing in Qdrant vector database for downstream retrieval. This system enables semantic search capabilities for documentation and knowledge bases.

## Features

### Core Functionality
- **Docusaurus Documentation Crawling**: Automatically crawls Docusaurus-based documentation sites with respect to robots.txt and proper rate limiting
- **Semantic Content Extraction**: Extracts clean textual content while preserving document structure and headings
- **Intelligent Chunking**: Splits content into semantically meaningful chunks with configurable size (50-1000 tokens) and overlap
- **Cohere Embeddings**: Generates semantic embeddings using Cohere's embed-multilingual-v3.0 model with 1024-dimensional vectors
- **Qdrant Vector Storage**: Stores embeddings in Qdrant vector database with cosine similarity for semantic search
- **Event-Driven Architecture**: Uses Redis/RQ for background processing of crawl jobs
- **API Integration**: Provides RESTful API endpoints for crawling, searching, and management

### Advanced Features
- **Idempotent Operations**: Safe to re-run without duplication
- **Rate Limiting**: Configurable rate limiting with sliding window algorithm
- **Caching**: Multi-level caching for embeddings to avoid redundant API calls
- **Fallback Strategies**: Graceful degradation when external services are unavailable
- **Structured Logging**: Comprehensive JSON-formatted logging with context
- **Metrics Collection**: Prometheus-style metrics for monitoring and alerting
- **Health Monitoring**: Comprehensive health checks for all system components
- **Data Retention**: Configurable retention policies with time-based cleanup
- **Security Validation**: Input validation and sanitization middleware
- **Alerting System**: Critical system failure alerts via webhooks and email
- **Backup & Recovery**: Automated backup and recovery procedures for data safety

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   API Layer     │    │  Queue Workers   │    │  Data Storage   │
│  (FastAPI)      │    │   (RQ/Redis)     │    │                 │
│                 │    │                  │    │  ┌─────────────┐ │
│  ┌───────────┐  │    │  ┌─────────────┐ │    │  │   Qdrant    │ │
│  │  /crawl   │  │───▶│  │Crawl Jobs   │ │───▶│  │  (Vector   │ │
│  │  /search  │  │    │  │Chunk Jobs   │ │    │  │   DB)      │ │
│  │  /health  │  │    │  │Embed Jobs   │ │    │  └─────────────┘ │
│  └───────────┘  │    │  └─────────────┘ │    │                 │
└─────────────────┘    └──────────────────┘    │  ┌─────────────┐ │
                                              │  │   Redis     │ │
                                              │  │ (Message    │ │
                                              │  │   Queue)    │ │
                                              │  └─────────────┘ │
                                              └─────────────────┘
```

## Setup and Installation

### Prerequisites
- Python 3.10+
- Docker and Docker Compose
- Redis (for message queues)
- Qdrant (vector database)

### Environment Configuration
Create a `.env` file with the following variables:
```bash
# API Keys
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here

# API Configuration
API_KEY=your_secure_api_key_here

# Crawler Settings
MAX_DEPTH=3
RATE_LIMIT_DELAY=1.0
MAX_WORKERS=4

# Processing Settings
MAX_CHUNK_SIZE=512
CHUNK_OVERLAP=0.2

# Redis Settings
REDIS_HOST=localhost
REDIS_PORT=6379

# Logging Settings
LOG_LEVEL=INFO
LOG_JSON_FORMAT=true

# Alerting Settings
ALERT_WEBHOOK_URLS=[]
SMTP_SERVER=localhost
SMTP_PORT=587
ALERT_RECIPIENTS=[]

# Backup Settings
BACKUP_DIRECTORY=./backups
MAX_BACKUP_AGE_DAYS=30
```

### Installation
1. Clone the repository
2. Install dependencies: `poetry install`
3. Set up environment variables in `.env`
4. Start the services: `docker-compose up -d`
5. Run the application: `python main.py`

## Usage

### Crawling Documentation
```bash
curl -X POST http://localhost:8000/crawl \
  -H "Authorization: Bearer your_api_key" \
  -H "Content-Type: application/json" \
  -d '{
    "url": "https://example-docs.com/docs",
    "max_depth": 2,
    "include_patterns": ["/docs/*"],
    "exclude_patterns": ["/docs/api/*"]
  }'
```

### Semantic Search
```bash
curl -X POST http://localhost:8000/search \
  -H "Authorization: Bearer your_api_key" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How to configure authentication?",
    "top_k": 5
  }'
```

### Check System Health
```bash
curl -X GET http://localhost:8000/health
```

## API Documentation

Complete API documentation is available at:
- [docs/api.md](docs/api.md) - Detailed API endpoints and examples
- [docs/user-guide.md](docs/user-guide.md) - User guide for the RAG ingestion pipeline
- [docs/performance-optimization.md](docs/performance-optimization.md) - Performance optimization guide

## Monitoring and Observability

The system includes comprehensive monitoring capabilities:

- **Structured Logging**: All logs are in JSON format with contextual information
- **Metrics Collection**: Prometheus-style metrics for performance monitoring
- **Health Checks**: Detailed health status for all system components
- **Alerting**: Configurable alerts for critical system failures
- **Performance Monitoring**: Real-time performance metrics and thresholds

## Security

- **API Authentication**: Bearer token authentication for all endpoints
- **Rate Limiting**: Protection against abuse with configurable limits
- **Input Validation**: Comprehensive validation and sanitization of all inputs
- **Security Scanning**: Built-in security validation for requests and responses

## Backup and Recovery

The system includes automated backup and recovery procedures:
- Automatic backup scheduling
- Compressed backup storage
- Verification of backup integrity
- One-click recovery from backups

## Performance Targets

- **Availability**: 99.9% uptime
- **Response Time**: < 500ms for search queries (p95)
- **Throughput**: 100+ requests per second for search operations
- **Scalability**: Horizontal scaling support for increased load

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## License

This project is licensed under the MIT License.