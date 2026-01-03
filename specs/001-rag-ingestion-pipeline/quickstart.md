# Quick Start: RAG Ingestion Pipeline

## Overview
This guide will help you set up and run the RAG ingestion pipeline to crawl documentation sites, generate embeddings, and store them in Qdrant for retrieval.

## Prerequisites
- Python 3.9 or higher
- UV package manager (install with `pip install uv`)
- Cohere API key
- Qdrant Cloud cluster credentials (URL and API key)

## Setup

### 1. Create Backend Directory
```bash
mkdir backend && cd backend
```

### 2. Initialize Python Project with UV
```bash
uv init
uv add fastapi httpx beautifulsoup4 lxml cohere qdrant-client python-dotenv typer
uv add --dev pytest black mypy
```

### 3. Create Environment File
Create a `.env` file in the backend directory with your credentials:

```bash
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
LOG_LEVEL=INFO
MAX_WORKERS=4
```

### 4. Project Structure
Create the following directory structure:

```
backend/
├── .env
├── pyproject.toml
├── main.py
├── .gitignore
├── docs/
├── scripts/
│   ├── __init__.py
│   ├── crawl_and_embed.py
│   └── validate_ingestion.py
├── src/
│   ├── __init__.py
│   ├── config/
│   │   ├── __init__.py
│   │   └── settings.py
│   ├── models/
│   │   ├── __init__.py
│   │   ├── chunk.py
│   │   └── job.py
│   ├── services/
│   │   ├── __init__.py
│   │   ├── crawler.py
│   │   ├── chunker.py
│   │   ├── embedding.py
│   │   └── storage.py
│   └── api/
│       ├── __init__.py
│       └── routes/
│           ├── __init__.py
│           └── crawl.py
```

### 5. Configuration Settings
Create `src/config/settings.py`:

```python
from pydantic_settings import BaseSettings
from typing import List, Optional


class Settings(BaseSettings):
    # API Keys and URLs
    cohere_api_key: str
    qdrant_url: str
    qdrant_api_key: str

    # Crawler settings
    max_depth: int = 3
    rate_limit_delay: float = 1.0  # seconds between requests
    user_agent: str = "RAG-Ingestion-Bot/1.0"

    # Processing settings
    max_chunk_size: int = 512  # tokens
    chunk_overlap: float = 0.2  # 20% overlap
    max_workers: int = 4

    # Qdrant settings
    qdrant_collection_name: str = "doc_embeddings"
    vector_dimensions: int = 1024  # Cohere v3 model

    # Logging
    log_level: str = "INFO"

    class Config:
        env_file = ".env"


settings = Settings()
```

## Running the Pipeline

### 1. Basic Crawl and Embed
```bash
# Run directly with Python
python -m scripts.crawl_and_embed --url "https://docs.example.com"

# Or using the CLI tool
python main.py crawl --url "https://docs.example.com" --max-depth 2
```

### 2. Start the API Server
```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### 3. Submit a Crawl Job via API
```bash
curl -X POST http://localhost:8000/api/v1/crawl-jobs \
  -H "Content-Type: application/json" \
  -d '{
    "target_url": "https://docs.example.com",
    "options": {
      "max_depth": 2,
      "rate_limit": 1
    }
  }'
```

## Validation

### 1. Verify Ingestion
Run the validation script to check if content was properly ingested:

```bash
python -m scripts.validate_ingestion --collection-name "doc_embeddings"
```

### 2. Test Search
```bash
curl -X POST http://localhost:8000/api/v1/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "your search query here",
    "top_k": 5
  }'
```

## Development

### Running Tests
```bash
python -m pytest
```

### Code Formatting
```bash
black .
```

### Type Checking
```bash
mypy .
```

## Next Steps
1. Customize the CSS selectors in the crawler for your specific Docusaurus site
2. Adjust chunking parameters based on your content requirements
3. Set up monitoring and alerting for production deployments
4. Implement additional preprocessing steps if needed for your content