# Quickstart Guide: RAG Agent Backend with OpenAI Agents SDK using Gemini

**Feature**: 003-rag-agent-gemini
**Created**: 2025-12-24

## Overview

This guide provides instructions to quickly set up and run the RAG Agent Backend with OpenAI Agents SDK using Google's Gemini model. The system allows users to query technical documentation and receive contextually relevant responses powered by Gemini.

## Prerequisites

- Python 3.10 or higher
- UV package manager
- Access to Google's Gemini API
- Qdrant vector database (either local or cloud instance)

## Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Navigate to Backend Directory
```bash
cd backend
```

### 3. Install Dependencies with UV
```bash
# If UV is not installed, install it first
pip install uv

# Install project dependencies
uv pip install -r requirements.txt
# or if using pyproject.toml
uv sync
```

### 4. Set Up Environment Variables

Create a `.env` file in the backend directory with the following variables:

```env
# Gemini API Configuration
GEMINI_API_KEY=your_gemini_api_key_here
GEMINI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/

# Qdrant Configuration
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=doc_embeddings

# Application Settings
LOG_LEVEL=INFO
HOST=0.0.0.0
PORT=8000
MAX_WORKERS=4
RATE_LIMIT_DELAY=1.0
MAX_CHUNK_SIZE=512
CHUNK_OVERLAP=0.2
VECTOR_DIMENSIONS=1024  # For Gemini-compatible embeddings
```

## Running the Application

### 1. Start the Development Server
```bash
# Using uvicorn directly
uvicorn main:app --reload --host 0.0.0.0 --port 8000

# Or using a startup script if available
python -m src.main
```

### 2. Verify the Service is Running
```bash
curl http://localhost:8000/health
```

Expected response:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-24T10:00:00Z"
}
```

## Making Your First Query

### 1. Send a Query to the API
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your-auth-token" \
  -d '{
    "query": "What is this system about?",
    "session_id": "test-session-123"
  }'
```

### 2. Expected Response
```json
{
  "response": "This system is a RAG (Retrieval-Augmented Generation) agent that uses Google's Gemini model...",
  "sources": [
    {
      "document_id": "doc-intro-1",
      "title": "System Introduction",
      "url": "https://example.com/docs/intro",
      "page": 1,
      "relevance_score": 0.92,
      "text_snippet": "This system combines retrieval and generation to provide contextually relevant answers..."
    }
  ],
  "session_id": "test-session-123",
  "tokens_used": {
    "input_tokens": 8,
    "output_tokens": 45,
    "total_tokens": 53
  },
  "retrieval_info": {
    "retrieved_chunks": 2,
    "search_time_ms": 125.3,
    "top_k": 5
  }
}
```

## Configuration Options

### Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `GEMINI_API_KEY` | Google Gemini API key | required |
| `GEMINI_BASE_URL` | Gemini OpenAI-compatible API endpoint | `https://generativelanguage.googleapis.com/v1beta/openai/` |
| `QDRANT_URL` | Qdrant database URL | required |
| `QDRANT_API_KEY` | Qdrant database API key | required |
| `QDRANT_COLLECTION_NAME` | Name of the collection for document embeddings | `doc_embeddings` |
| `HOST` | Host to bind the server to | `0.0.0.0` |
| `PORT` | Port to run the server on | `8000` |
| `LOG_LEVEL` | Logging level | `INFO` |
| `MAX_WORKERS` | Maximum number of worker threads | `4` |
| `VECTOR_DIMENSIONS` | Dimension of embeddings (for Gemini) | `1024` |

## Testing the API

### 1. Unit Tests
```bash
# Run all tests
python -m pytest tests/

# Run specific test file
python -m pytest tests/test_query_api.py

# Run with coverage
python -m pytest --cov=src tests/
```

### 2. API Contract Tests
```bash
# Test API compliance with OpenAPI spec
python -m pytest tests/test_api_contracts.py
```

## Troubleshooting

### Common Issues

1. **Authentication Errors**
   - Verify `GEMINI_API_KEY` is correctly set
   - Check that the API key has appropriate permissions

2. **Qdrant Connection Issues**
   - Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct
   - Check that Qdrant service is running and accessible

3. **Slow Response Times**
   - Check that the Qdrant instance has sufficient resources
   - Verify network connectivity to external APIs

### Health Checks
- `/health` - Basic service health
- `/health/external` - Check external dependencies (Gemini, Qdrant)

## Next Steps

1. **Ingest Documents**: Use the ingestion pipeline to add documents to your Qdrant collection
2. **Configure Authentication**: Set up proper authentication for production use
3. **Scale Deployment**: Consider containerization and orchestration for production deployment
4. **Monitor Performance**: Set up logging and monitoring for production environments