# RAG Ingestion Pipeline API Documentation

This document provides comprehensive documentation for the RAG Ingestion Pipeline API, including endpoints, request/response formats, and usage examples.

## Table of Contents

1. [Authentication](#authentication)
2. [API Endpoints](#api-endpoints)
   - [Crawling Endpoints](#crawling-endpoints)
   - [Search Endpoints](#search-endpoints)
   - [Management Endpoints](#management-endpoints)
   - [Health Endpoints](#health-endpoints)
3. [Response Format](#response-format)
4. [Error Handling](#error-handling)
5. [Rate Limiting](#rate-limiting)

## Authentication

All API endpoints require authentication using an API key. Include the API key in the `Authorization` header:

```
Authorization: Bearer YOUR_API_KEY
```

Alternatively, you can use the `X-API-Key` header:

```
X-API-Key: YOUR_API_KEY
```

## API Endpoints

### Crawling Endpoints

#### POST /crawl

Initiates a crawling job for a given URL. The job is processed asynchronously using background workers.

**Request Body:**
```json
{
  "url": "https://example.com/docs",
  "max_depth": 2,
  "include_patterns": ["/docs/*"],
  "exclude_patterns": ["/docs/api/*"],
  "concurrency": 5
}
```

**Parameters:**
- `url` (string, required): The base URL to crawl
- `max_depth` (integer, optional): Maximum depth to crawl (default: 1)
- `include_patterns` (array, optional): URL patterns to include in crawling
- `exclude_patterns` (array, optional): URL patterns to exclude from crawling
- `concurrency` (integer, optional): Number of concurrent requests (default: 5)

**Example Request:**
```bash
curl -X POST http://localhost:8000/crawl \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -d '{
    "url": "https://docusaurus.io/docs",
    "max_depth": 2,
    "include_patterns": ["/docs/*"],
    "exclude_patterns": ["/docs/api/*"],
    "concurrency": 3
  }'
```

**Response:**
```json
{
  "job_id": "crawling_12345",
  "status": "queued",
  "url": "https://docusaurus.io/docs",
  "queued_at": "2025-12-22T10:30:00Z"
}
```

#### GET /crawl/{job_id}

Gets the status of a crawling job.

**Path Parameters:**
- `job_id` (string, required): The ID of the crawling job

**Example Request:**
```bash
curl -X GET http://localhost:8000/crawl/crawling_12345 \
  -H "Authorization: Bearer YOUR_API_KEY"
```

**Response:**
```json
{
  "job_id": "crawling_12345",
  "status": "completed",
  "url": "https://docusaurus.io/docs",
  "pages_processed": 42,
  "pages_failed": 0,
  "started_at": "2025-12-22T10:30:00Z",
  "completed_at": "2025-12-22T10:35:00Z",
  "progress": 100
}
```

#### GET /crawl/status

Gets the status of all active crawling jobs.

**Example Request:**
```bash
curl -X GET http://localhost:8000/crawl/status \
  -H "Authorization: Bearer YOUR_API_KEY"
```

**Response:**
```json
{
  "active_jobs": [
    {
      "job_id": "crawling_12345",
      "status": "processing",
      "url": "https://docusaurus.io/docs",
      "progress": 65,
      "pages_processed": 27,
      "pages_total": 42
    }
  ],
  "total_active_jobs": 1
}
```

### Search Endpoints

#### POST /search

Performs semantic search against the vector database.

**Request Body:**
```json
{
  "query": "How to configure authentication?",
  "top_k": 5,
  "filters": {
    "source_domain": "docusaurus.io"
  }
}
```

**Parameters:**
- `query` (string, required): The search query
- `top_k` (integer, optional): Number of results to return (default: 5, max: 20)
- `filters` (object, optional): Filters to apply to the search results

**Example Request:**
```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -d '{
    "query": "How to configure authentication?",
    "top_k": 5,
    "filters": {
      "source_domain": "docusaurus.io"
    }
  }'
```

**Response:**
```json
{
  "query": "How to configure authentication?",
  "results": [
    {
      "id": "chunk_abc123",
      "score": 0.85,
      "content": "To configure authentication in Docusaurus, you need to set up the authentication plugin...",
      "metadata": {
        "url": "https://docusaurus.io/docs/authentication",
        "title": "Authentication Configuration",
        "chunk_index": 0,
        "source_domain": "docusaurus.io"
      }
    }
  ],
  "search_time_ms": 42.5
}
```

#### POST /search/advanced

Performs advanced semantic search with additional options.

**Request Body:**
```json
{
  "query": "How to customize the navbar?",
  "top_k": 10,
  "min_score": 0.5,
  "filters": {
    "tags": ["configuration", "ui"],
    "published_after": "2024-01-01"
  },
  "return_metadata": true,
  "highlight": true
}
```

**Parameters:**
- `query` (string, required): The search query
- `top_k` (integer, optional): Number of results to return (default: 5, max: 50)
- `min_score` (number, optional): Minimum similarity score (0.0-1.0)
- `filters` (object, optional): Filters to apply to the search results
- `return_metadata` (boolean, optional): Whether to return full metadata (default: true)
- `highlight` (boolean, optional): Whether to highlight query terms in results (default: false)

**Example Request:**
```bash
curl -X POST http://localhost:8000/search/advanced \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -d '{
    "query": "How to customize the navbar?",
    "top_k": 10,
    "min_score": 0.5,
    "filters": {
      "tags": ["configuration", "ui"]
    },
    "highlight": true
  }'
```

### Management Endpoints

#### GET /documents

Lists stored documents with metadata.

**Query Parameters:**
- `page` (integer, optional): Page number (default: 1)
- `limit` (integer, optional): Number of items per page (default: 20, max: 100)
- `source_url` (string, optional): Filter by source URL
- `sort_by` (string, optional): Sort field (default: "created_at")
- `sort_order` (string, optional): Sort order ("asc" or "desc", default: "desc")

**Example Request:**
```bash
curl -X GET "http://localhost:8000/documents?page=1&limit=10&source_url=https://docusaurus.io/docs" \
  -H "Authorization: Bearer YOUR_API_KEY"
```

**Response:**
```json
{
  "documents": [
    {
      "id": "doc_123",
      "url": "https://docusaurus.io/docs/introduction",
      "title": "Introduction to Docusaurus",
      "chunk_count": 3,
      "created_at": "2025-12-22T09:00:00Z",
      "updated_at": "2025-12-22T09:00:00Z"
    }
  ],
  "total": 150,
  "page": 1,
  "limit": 10,
  "has_more": true
}
```

#### DELETE /documents/{document_id}

Deletes a document and its associated chunks from the vector database.

**Path Parameters:**
- `document_id` (string, required): The ID of the document to delete

**Example Request:**
```bash
curl -X DELETE http://localhost:8000/documents/doc_123 \
  -H "Authorization: Bearer YOUR_API_KEY"
```

**Response:**
```json
{
  "message": "Document deleted successfully",
  "document_id": "doc_123",
  "chunks_deleted": 3
}
```

#### POST /documents/bulk-delete

Deletes multiple documents based on filters.

**Request Body:**
```json
{
  "filters": {
    "source_domain": "example.com",
    "created_before": "2024-01-01"
  }
}
```

**Example Request:**
```bash
curl -X POST http://localhost:8000/documents/bulk-delete \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -d '{
    "filters": {
      "source_domain": "example.com",
      "created_before": "2024-01-01"
    }
  }'
```

**Response:**
```json
{
  "message": "Documents deleted successfully",
  "documents_deleted": 25,
  "chunks_deleted": 150
}
```

### Health Endpoints

#### GET /health

Checks the health status of the application and its dependencies.

**Example Request:**
```bash
curl -X GET http://localhost:8000/health
```

**Response:**
```json
{
  "status": "healthy",
  "timestamp": "2025-12-22T11:00:00Z",
  "services": {
    "application": "healthy",
    "database": "healthy",
    "qdrant": "healthy",
    "redis": "healthy"
  },
  "version": "1.0.0"
}
```

#### GET /health/details

Provides detailed health information for all system components.

**Example Request:**
```bash
curl -X GET http://localhost:8000/health/details
```

**Response:**
```json
{
  "status": "healthy",
  "checks": {
    "database_connection": {
      "status": "healthy",
      "response_time_ms": 2.5
    },
    "qdrant_connection": {
      "status": "healthy",
      "response_time_ms": 15.2,
      "collection_status": "ready"
    },
    "redis_connection": {
      "status": "healthy",
      "response_time_ms": 1.8
    },
    "cohere_api": {
      "status": "healthy",
      "response_time_ms": 250.3
    }
  }
}
```

## Response Format

All API responses follow a consistent format:

```json
{
  "success": true,
  "data": { ... },
  "message": "Operation completed successfully",
  "timestamp": "2025-12-22T11:00:00Z"
}
```

For error responses:

```json
{
  "success": false,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid request parameters",
    "details": { ... }
  },
  "timestamp": "2025-12-22T11:00:00Z"
}
```

## Error Handling

The API returns standard HTTP status codes:

- `200 OK`: Request completed successfully
- `201 Created`: Resource created successfully
- `400 Bad Request`: Invalid request parameters
- `401 Unauthorized`: Missing or invalid authentication
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Requested resource not found
- `422 Unprocessable Entity`: Validation error
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error

Common error responses:

**Validation Error:**
```json
{
  "success": false,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Request validation failed",
    "details": [
      {
        "field": "url",
        "message": "URL must be a valid HTTPS URL"
      }
    ]
  }
}
```

**Authentication Error:**
```json
{
  "success": false,
  "error": {
    "code": "AUTHENTICATION_ERROR",
    "message": "Invalid or missing API key"
  }
}
```

**Rate Limit Error:**
```json
{
  "success": false,
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Too many requests. Maximum 60 per minute."
  }
}
```

## Rate Limiting

The API implements rate limiting to prevent abuse:

- Default limit: 60 requests per minute per IP
- Authenticated requests: 1000 requests per minute per API key
- Search endpoints: Additional rate limiting of 10 requests per minute per IP

Rate limit information is included in response headers:
- `X-RateLimit-Limit`: Maximum requests allowed
- `X-RateLimit-Remaining`: Remaining requests
- `X-RateLimit-Reset`: Time when the rate limit resets (Unix timestamp)