# API Contracts: RAG Ingestion Pipeline

## Crawl Management API

### Create Crawl Job
```
POST /api/v1/crawl-jobs
```

#### Request
```json
{
  "target_url": "https://docs.example.com",
  "options": {
    "include_patterns": ["/docs/**"],
    "exclude_patterns": ["/blog/**", "/api/**"],
    "selectors": {
      "content": [".main-wrapper .markdown", ".theme-doc-markdown", ".doc-content"],
      "title": ["h1", ".hero__title"],
      "navigation": [".menu", ".nav", ".sidebar"]
    },
    "max_depth": 3,
    "rate_limit": 1
  }
}
```

#### Response (201 Created)
```json
{
  "id": "a1b2c3d4-e5f6-7890-1234-567890abcdef",
  "target_url": "https://docs.example.com",
  "status": "pending",
  "progress": 0,
  "total_pages": 0,
  "processed_pages": 0,
  "failed_pages": 0,
  "options": { ... },
  "created_at": "2025-12-22T10:00:00Z",
  "updated_at": "2025-12-22T10:00:00Z"
}
```

#### Errors
- 400: Invalid URL format
- 422: Validation error in request parameters
- 429: Rate limit exceeded

---

### Get Crawl Job Status
```
GET /api/v1/crawl-jobs/{id}
```

#### Response (200 OK)
```json
{
  "id": "a1b2c3d4-e5f6-7890-1234-567890abcdef",
  "target_url": "https://docs.example.com",
  "status": "processing",
  "progress": 45,
  "total_pages": 22,
  "processed_pages": 10,
  "failed_pages": 0,
  "options": { ... },
  "error_log": null,
  "created_at": "2025-12-22T10:00:00Z",
  "updated_at": "2025-12-22T10:15:30Z",
  "completed_at": null
}
```

#### Errors
- 404: Crawl job not found

---

### List Crawl Jobs
```
GET /api/v1/crawl-jobs
```

#### Query Parameters
- `status` (optional): Filter by status (pending, processing, completed, failed)
- `limit` (optional): Number of results to return (default: 20, max: 100)
- `offset` (optional): Number of results to skip (default: 0)

#### Response (200 OK)
```json
{
  "jobs": [
    {
      "id": "a1b2c3d4-e5f6-7890-1234-567890abcdef",
      "target_url": "https://docs.example.com",
      "status": "completed",
      "progress": 100,
      "total_pages": 22,
      "processed_pages": 22,
      "created_at": "2025-12-22T10:00:00Z",
      "updated_at": "2025-12-22T10:45:22Z"
    }
  ],
  "total": 1,
  "limit": 20,
  "offset": 0
}
```

---

## Content Management API

### Search Content Chunks
```
POST /api/v1/search
```

#### Request
```json
{
  "query": "How to configure authentication?",
  "top_k": 5,
  "filters": {
    "source_url": "https://docs.example.com/docs/auth"
  }
}
```

#### Response (200 OK)
```json
{
  "results": [
    {
      "content_chunk": {
        "id": "f1e2d3c4-a5b6-7890-1234-567890fedcba",
        "source_url": "https://docs.example.com/docs/auth/configuration",
        "page_title": "Authentication Configuration",
        "section_heading": "Setting up OAuth",
        "content": "To configure authentication, first set up your OAuth provider...",
        "chunk_index": 2,
        "metadata": { ... }
      },
      "similarity": 0.92
    }
  ]
}
```

---

### Get Content Chunk
```
GET /api/v1/content-chunks/{id}
```

#### Response (200 OK)
```json
{
  "id": "f1e2d3c4-a5b6-7890-1234-567890fedcba",
  "source_url": "https://docs.example.com/docs/auth/configuration",
  "page_title": "Authentication Configuration",
  "section_heading": "Setting up OAuth",
  "content": "To configure authentication, first set up your OAuth provider...",
  "chunk_index": 2,
  "token_count": 128,
  "metadata": {
    "extracted_at": "2025-12-22T10:15:22Z",
    "extraction_method": "docusaurus-scraper-v1"
  },
  "created_at": "2025-12-22T10:15:22Z",
  "updated_at": "2025-12-22T10:15:22Z"
}
```

---

### List Content Chunks
```
GET /api/v1/content-chunks
```

#### Query Parameters
- `source_url` (optional): Filter by source URL (supports partial matching)
- `page_title` (optional): Filter by page title (supports partial matching)
- `limit` (optional): Number of results to return (default: 20, max: 100)
- `offset` (optional): Number of results to skip (default: 0)

#### Response (200 OK)
```json
{
  "chunks": [
    {
      "id": "f1e2d3c4-a5b6-7890-1234-567890fedcba",
      "source_url": "https://docs.example.com/docs/auth/configuration",
      "page_title": "Authentication Configuration",
      "section_heading": "Setting up OAuth",
      "chunk_index": 2,
      "token_count": 128,
      "created_at": "2025-12-22T10:15:22Z"
    }
  ],
  "total": 1,
  "limit": 20,
  "offset": 0
}
```

---

## System Health API

### Health Check
```
GET /health
```

#### Response (200 OK)
```json
{
  "status": "healthy",
  "timestamp": "2025-12-22T10:30:45Z",
  "services": {
    "database": "connected",
    "qdrant": "connected",
    "cohere_api": "reachable"
  }
}
```

---

## Error Response Format

All error responses follow this format:

```json
{
  "error": {
    "type": "ValidationError",
    "message": "The request contains invalid parameters",
    "details": [
      {
        "field": "target_url",
        "issue": "Invalid URL format"
      }
    ]
  }
}
```

### Common Error Types
- `ValidationError`: Request parameters are invalid
- `NotFoundError`: Requested resource doesn't exist
- `ServiceUnavailableError`: External service is unavailable
- `RateLimitError`: Rate limit has been exceeded
- `ProcessingError`: Error occurred during processing