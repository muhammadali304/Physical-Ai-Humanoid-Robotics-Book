# RAG Ingestion Pipeline User Guide

This guide provides comprehensive instructions on how to use the RAG Ingestion Pipeline to crawl documentation sites, generate embeddings, and perform semantic search.

## Table of Contents

1. [Getting Started](#getting-started)
2. [Prerequisites](#prerequisites)
3. [Configuration](#configuration)
4. [Crawling Documentation Sites](#crawling-documentation-sites)
5. [Searching Content](#searching-content)
6. [Managing Documents](#managing-documents)
7. [Monitoring and Troubleshooting](#monitoring-and-troubleshooting)
8. [Best Practices](#best-practices)

## Getting Started

The RAG Ingestion Pipeline is a comprehensive system for ingesting documentation content, generating semantic embeddings, and enabling semantic search. This guide will walk you through setting up, configuring, and using the pipeline effectively.

### What You'll Learn

- How to configure the pipeline with your API keys
- How to crawl documentation sites efficiently
- How to perform semantic searches on your content
- How to manage and maintain your document collection
- How to monitor system health and performance

## Prerequisites

Before using the RAG Ingestion Pipeline, you'll need:

1. **Cohere API Key**: Sign up at [Cohere](https://cohere.ai) to get an API key for embedding generation
2. **Qdrant Cloud Account**: Create an account at [Qdrant Cloud](https://cloud-qdrant.qdrant.io) for vector storage
3. **Docker and Docker Compose**: For running the pipeline in containers
4. **Python 3.10+**: If running locally without Docker
5. **Redis**: For message queuing (automatically included in Docker setup)

## Configuration

### Environment Variables

Create a `.env` file in the project root with the following variables:

```bash
# Cohere API configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant configuration
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=content_embeddings

# Database configuration (if using external database)
DATABASE_URL=postgresql://user:password@localhost/dbname

# Redis configuration (for message queues)
REDIS_URL=redis://localhost:6379/0

# API configuration
API_KEY=your_secure_api_key_here
ENVIRONMENT=production  # or development
```

### API Key Security

- Use a strong, unique API key for authentication
- Never commit API keys to version control
- Rotate API keys regularly for security
- Use different keys for different environments

## Crawling Documentation Sites

### Initiating a Crawl

To start crawling a documentation site, use the `/crawl` endpoint:

```bash
curl -X POST http://localhost:8000/crawl \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your_api_key" \
  -d '{
    "url": "https://docusaurus.io/docs",
    "max_depth": 2,
    "include_patterns": ["/docs/*"],
    "exclude_patterns": ["/docs/api/*", "/docs/next/*"],
    "concurrency": 5
  }'
```

### Crawl Parameters

- `url`: The base URL of the documentation site to crawl
- `max_depth`: How deep to follow links (1 = only the specified URL, 2 = URL + one level of links, etc.)
- `include_patterns`: URL patterns to include in the crawl (supports wildcards)
- `exclude_patterns`: URL patterns to exclude from the crawl (supports wildcards)
- `concurrency`: Number of concurrent requests (be respectful to the target site)

### Monitoring Crawl Progress

Check the status of a crawl job using its job ID:

```bash
curl -X GET http://localhost:8000/crawl/crawling_12345 \
  -H "Authorization: Bearer your_api_key"
```

### Best Practices for Crawling

1. **Respect robots.txt**: Always check the site's robots.txt file
2. **Set appropriate concurrency**: Start with low concurrency (3-5) and increase gradually
3. **Use patterns wisely**: Use include/exclude patterns to focus on relevant content
4. **Monitor rate limits**: Be aware of any rate limits imposed by the target site
5. **Verify content quality**: Review crawled content to ensure it meets your needs

### Example Crawl Scenarios

#### Crawl Entire Documentation Site

```bash
curl -X POST http://localhost:8000/crawl \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your_api_key" \
  -d '{
    "url": "https://example-docs.com/docs",
    "max_depth": 3,
    "include_patterns": ["/docs/*"],
    "exclude_patterns": ["/docs/api/*", "/docs/changelog/*"],
    "concurrency": 3
  }'
```

#### Crawl Specific Section

```bash
curl -X POST http://localhost:8000/crawl \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your_api_key" \
  -d '{
    "url": "https://example-docs.com/docs/guides",
    "max_depth": 2,
    "include_patterns": ["/docs/guides/*"],
    "concurrency": 2
  }'
```

## Searching Content

### Basic Semantic Search

Perform a semantic search using the `/search` endpoint:

```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your_api_key" \
  -d '{
    "query": "How to configure authentication in the application?",
    "top_k": 5
  }'
```

### Advanced Search with Filters

Use the `/search/advanced` endpoint for more control:

```bash
curl -X POST http://localhost:8000/search/advanced \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your_api_key" \
  -d '{
    "query": "authentication configuration",
    "top_k": 10,
    "min_score": 0.6,
    "filters": {
      "source_domain": "docusaurus.io",
      "tags": ["security", "configuration"]
    },
    "highlight": true
  }'
```

### Search Response Format

The search response includes:

- `results`: Array of matching content chunks with scores
- `score`: Similarity score (0.0-1.0, where 1.0 is most similar)
- `content`: The text content of the matching chunk
- `metadata`: Additional information including URL, title, and chunk index

### Search Tips

1. **Use specific queries**: More specific queries often yield better results
2. **Adjust top_k**: Increase for more results, decrease for focus
3. **Set min_score**: Filter out low-quality matches
4. **Use filters**: Narrow results by source, date, or tags
5. **Enable highlighting**: Helps identify relevant parts of results

## Managing Documents

### Listing Documents

View all stored documents:

```bash
curl -X GET "http://localhost:8000/documents?page=1&limit=20" \
  -H "Authorization: Bearer your_api_key"
```

### Deleting Documents

Remove a specific document:

```bash
curl -X DELETE http://localhost:8000/documents/doc_123 \
  -H "Authorization: Bearer your_api_key"
```

### Bulk Deletion

Delete multiple documents based on criteria:

```bash
curl -X POST http://localhost:8000/documents/bulk-delete \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your_api_key" \
  -d '{
    "filters": {
      "source_domain": "example.com",
      "created_before": "2024-01-01"
    }
  }'
```

### Data Retention

The system implements automatic data retention policies:

- Default retention: 30 days (configurable)
- Automatic cleanup of old documents
- Configurable retention periods per document type

## Monitoring and Troubleshooting

### Health Checks

Check system health:

```bash
curl -X GET http://localhost:8000/health
```

For detailed health information:

```bash
curl -X GET http://localhost:8000/health/details
```

### Common Issues and Solutions

#### Crawl Jobs Stuck in Queue

**Symptoms**: Crawl jobs remain in "queued" status for a long time

**Solutions**:
1. Check if worker processes are running
2. Verify Redis connection
3. Check system resources (CPU, memory)
4. Review worker logs for errors

#### Poor Search Results

**Symptoms**: Search returns irrelevant or low-quality results

**Solutions**:
1. Verify embedding quality and dimensionality
2. Check if content was properly chunked
3. Adjust search parameters (top_k, min_score)
4. Review crawled content quality

#### API Rate Limiting

**Symptoms**: Requests return 429 errors

**Solutions**:
1. Check your request rate against limits
2. Implement exponential backoff in your client
3. Use authentication for higher rate limits
4. Consider batch processing for high-volume operations

#### Embedding Generation Failures

**Symptoms**: Errors during embedding generation

**Solutions**:
1. Verify Cohere API key validity
2. Check internet connectivity to Cohere
3. Review rate limits on Cohere API
4. Check for malformed text content

### Log Analysis

The system generates structured logs that can be analyzed for:

- Request/response patterns
- Performance bottlenecks
- Error trends
- Resource usage

## Best Practices

### Performance Optimization

1. **Batch Operations**: Use batch endpoints when processing multiple items
2. **Caching**: Implement client-side caching for frequently accessed data
3. **Connection Pooling**: Reuse connections for multiple requests
4. **Asynchronous Processing**: Use asynchronous endpoints for long-running operations

### Security

1. **API Key Management**: Rotate keys regularly and use environment variables
2. **Network Security**: Use HTTPS for all API communications
3. **Input Validation**: Always validate and sanitize inputs
4. **Access Control**: Implement proper authentication and authorization

### Content Quality

1. **Selective Crawling**: Focus on high-quality, relevant content
2. **Content Cleaning**: Remove boilerplate, navigation, and ads
3. **Proper Chunking**: Maintain semantic coherence in chunks
4. **Metadata Preservation**: Keep important context with content

### System Maintenance

1. **Regular Monitoring**: Set up alerts for system health
2. **Backup Strategy**: Regularly backup critical data
3. **Performance Tuning**: Monitor and optimize system performance
4. **Documentation Updates**: Keep documentation current with system changes

### Cost Management

1. **API Usage Monitoring**: Track embedding API usage
2. **Storage Optimization**: Implement appropriate retention policies
3. **Resource Scaling**: Scale resources based on actual usage
4. **Efficient Processing**: Minimize redundant operations

## Integration Examples

### Python Client

```python
import requests
import json

class RAGClient:
    def __init__(self, base_url, api_key):
        self.base_url = base_url
        self.headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        }

    def search(self, query, top_k=5):
        response = requests.post(
            f"{self.base_url}/search",
            headers=self.headers,
            json={"query": query, "top_k": top_k}
        )
        return response.json()

    def start_crawl(self, url, max_depth=1):
        response = requests.post(
            f"{self.base_url}/crawl",
            headers=self.headers,
            json={"url": url, "max_depth": max_depth}
        )
        return response.json()

# Usage
client = RAGClient("http://localhost:8000", "your_api_key")
results = client.search("authentication configuration")
print(results)
```

### JavaScript Client

```javascript
class RAGClient {
  constructor(baseUrl, apiKey) {
    this.baseUrl = baseUrl;
    this.headers = {
      'Authorization': `Bearer ${apiKey}`,
      'Content-Type': 'application/json'
    };
  }

  async search(query, topK = 5) {
    const response = await fetch(`${this.baseUrl}/search`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify({ query, top_k: topK })
    });
    return await response.json();
  }

  async startCrawl(url, maxDepth = 1) {
    const response = await fetch(`${this.baseUrl}/crawl`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify({ url, max_depth: maxDepth })
    });
    return await response.json();
  }
}

// Usage
const client = new RAGClient('http://localhost:8000', 'your_api_key');
const results = await client.search('authentication configuration');
console.log(results);
```

## Troubleshooting Checklist

Before contacting support, verify:

- [ ] Environment variables are properly set
- [ ] API keys are valid and have appropriate permissions
- [ ] Network connectivity to external services (Cohere, Qdrant)
- [ ] Sufficient system resources (memory, disk space)
- [ ] Docker containers are running (if using Docker)
- [ ] Rate limits are not being exceeded
- [ ] Request formats match API documentation
- [ ] Target websites are accessible and not blocking requests

## Support and Resources

- **API Documentation**: [docs/api.md](api.md) - Complete API reference
- **GitHub Repository**: Check the project repository for updates and issues
- **Community**: Join our community for support and discussions
- **Issue Tracker**: Report bugs or request features through the issue tracker