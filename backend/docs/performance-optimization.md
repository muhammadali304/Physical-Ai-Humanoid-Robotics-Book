# Performance Optimization Guide for RAG Ingestion Pipeline

This guide provides comprehensive information on optimizing the performance of the RAG Ingestion Pipeline, including configuration, monitoring, and best practices.

## Table of Contents

1. [Performance Targets](#performance-targets)
2. [System Architecture for Performance](#system-architecture-for-performance)
3. [Configuration Optimization](#configuration-optimization)
4. [Database and Storage Optimization](#database-and-storage-optimization)
5. [Caching Strategies](#caching-strategies)
6. [Monitoring and Metrics](#monitoring-and-metrics)
7. [Load Testing](#load-testing)
8. [Performance Tuning](#performance-tuning)
9. [Troubleshooting Performance Issues](#troubleshooting-performance-issues)

## Performance Targets

The RAG Ingestion Pipeline is designed to meet the following performance targets:

- **Availability**: 99.9% uptime
- **Response Time**:
  - Health checks: < 50ms (p95)
  - Search queries: < 500ms (p95)
  - Crawl initiation: < 100ms (p95)
- **Throughput**:
  - Search queries: 100+ requests per second
  - Health checks: 1000+ requests per second
- **Scalability**: Horizontal scaling support for increased load

## System Architecture for Performance

### Service Architecture
```
[API Gateway] -> [Load Balancer] -> [Multiple API Instances]
                                    -> [Multiple Worker Instances]
                                    -> [Redis Queue]
                                    -> [Qdrant Vector DB]
                                    -> [External APIs (Cohere)]
```

### Performance-Critical Components
1. **API Layer**: FastAPI with Gunicorn workers
2. **Message Queue**: Redis for background processing
3. **Vector Database**: Qdrant for semantic search
4. **Caching Layer**: In-memory and Redis caching
5. **External Services**: Cohere API for embeddings

## Configuration Optimization

### API Server Configuration
```bash
# Gunicorn configuration (gunicorn.conf.py)
workers = multiprocessing.cpu_count() * 2 + 1
worker_class = "uvicorn.workers.UvicornWorker"
worker_connections = 1000
timeout = 300
keepalive = 5
max_requests = 1000
max_requests_jitter = 100
```

### Environment Variables for Performance
```bash
# Performance-related settings
WORKERS=4  # Number of Gunicorn workers
WORKER_CONNECTIONS=1000
TIMEOUT=300
KEEPALIVE=5

# Crawler settings
MAX_WORKERS=4  # Concurrent crawling requests
RATE_LIMIT_DELAY=1.0  # Seconds between requests

# Chunking settings
MAX_CHUNK_SIZE=512  # Maximum tokens per chunk
CHUNK_OVERLAP=0.2  # 20% overlap between chunks

# Redis settings
REDIS_MAX_CONNECTIONS=20
REDIS_CONNECTION_TIMEOUT=30

# Qdrant settings
QDRANT_TIMEOUT=30
QDRANT_RETRIES=3
```

## Database and Storage Optimization

### Qdrant Vector Database Optimization

#### Collection Configuration
- **Vector Dimensions**: 1024 (for Cohere multilingual v3 model)
- **Distance Metric**: Cosine similarity
- **Sharding**: Enable sharding for large datasets
- **Replication**: Configure for high availability

#### Indexing Strategy
- **HNSW Index**: For fast approximate nearest neighbor search
- **Quantization**: Enable scalar quantization for memory optimization
- **Payload Indexing**: Create indexes on frequently queried fields

#### Performance Settings
```python
# Qdrant client configuration
client = QdrantClient(
    url=settings.qdrant_url,
    api_key=settings.qdrant_api_key,
    timeout=30.0,
    # Enable HTTP2 for better performance
    https=True
)
```

### Redis Optimization

#### Connection Pool Configuration
```python
# Redis connection pool settings
redis_client = redis.Redis(
    host=settings.redis_host,
    port=settings.redis_port,
    db=settings.redis_db,
    password=settings.redis_password,
    max_connections=20,
    socket_timeout=30,
    socket_connect_timeout=5,
    health_check_interval=30
)
```

## Caching Strategies

### Multi-Level Caching Architecture

#### Level 1: In-Memory Caching
- **Purpose**: Cache frequently accessed embeddings and search results
- **Implementation**: Python dict with TTL
- **TTL**: 1 hour for embeddings, 5 minutes for search results

#### Level 2: Redis Caching
- **Purpose**: Distributed caching across multiple instances
- **Implementation**: Redis with expiration
- **TTL**: 2 hours for embeddings, 10 minutes for search results

#### Caching Implementation
```python
# Embedding caching example
class EmbeddingCache:
    def __init__(self):
        self._cache = {}
        self._ttl = 3600  # 1 hour

    async def get(self, key: str) -> Optional[EmbeddingVector]:
        if key in self._cache:
            data, timestamp = self._cache[key]
            if time.time() - timestamp < self._ttl:
                return data
            else:
                del self._cache[key]
        return None

    async def set(self, key: str, value: EmbeddingVector):
        self._cache[key] = (value, time.time())
```

## Monitoring and Metrics

### Key Performance Indicators (KPIs)

#### Response Time Metrics
- **P50, P95, P99 response times** for all endpoints
- **API endpoint latency** by method and path
- **Database query times** for Qdrant operations
- **External API call times** for Cohere services

#### Throughput Metrics
- **Requests per second** by endpoint
- **Successful vs failed request rates**
- **Queue processing rates** for background jobs

#### Resource Metrics
- **CPU and memory usage** of application instances
- **Database connection pool** utilization
- **Cache hit/miss ratios**

### Monitoring Implementation

#### Application Metrics
```python
# Example metrics collection
from src.monitoring.metrics import (
    REQUEST_COUNT, REQUEST_DURATION, CACHE_HIT_COUNT, CACHE_MISS_COUNT
)

async def search_endpoint(query: str):
    start_time = time.time()

    try:
        # Track cache hit/miss
        cache_result = await cache.get(query)
        if cache_result:
            CACHE_HIT_COUNT.inc()
        else:
            CACHE_MISS_COUNT.inc()
            # Perform actual search
            result = await perform_search(query)
            await cache.set(query, result)

        # Track request duration
        duration = time.time() - start_time
        REQUEST_DURATION.observe(duration)

        return result
    finally:
        REQUEST_COUNT.inc()
```

## Load Testing

### Load Testing Strategy

#### Test Scenarios
1. **Health Check Load**: 1000 requests at 50 concurrency
2. **Search Load**: 500 requests at 20 concurrency
3. **Crawl Load**: 50 requests at 5 concurrency (intensive operation)
4. **Mixed Load**: Combination of all operations

#### Performance Targets
- **Health Check**: > 99.9% success rate, < 50ms p95
- **Search**: > 99% success rate, < 500ms p95
- **Crawl**: > 95% success rate, < 2000ms p95

### Load Testing Script Usage
```bash
# Run the load test script
python scripts/load_test.py

# Example configuration for different scenarios
concurrency_levels = [10, 20, 50, 100]  # Different concurrency levels
request_counts = [100, 500, 1000]  # Different request volumes
```

## Performance Tuning

### API Performance Tuning

#### FastAPI Optimizations
- Use Pydantic models for request/response validation
- Implement async/await for I/O operations
- Use connection pooling for database connections
- Enable compression for large responses

#### Response Optimization
```python
# Optimize response size
class OptimizedSearchResponse(BaseModel):
    results: List[OptimizedSearchResult]
    search_time_ms: float
    # Exclude verbose metadata unless specifically requested
    model_config = ConfigDict(extra="ignore")
```

### Background Processing Optimization

#### Worker Configuration
```python
# Worker optimization settings
WORKER_BATCH_SIZE = 10  # Process items in batches
WORKER_TIMEOUT = 300  # 5 minute timeout per job
WORKER_RETRY_ATTEMPTS = 3  # Retry failed jobs
WORKER_CONCURRENCY = 4  # Concurrent jobs per worker
```

### Memory Management

#### Memory Optimization Techniques
- Use generators for large data processing
- Implement proper garbage collection
- Monitor memory usage and set appropriate limits
- Use memory-efficient data structures

## Troubleshooting Performance Issues

### Common Performance Issues

#### High Response Times
**Symptoms**: Response times > 1000ms
**Causes**:
- Database connection pool exhaustion
- External API rate limiting
- Memory leaks
- Inefficient queries

**Solutions**:
- Increase database connection pool size
- Implement exponential backoff for external APIs
- Profile and fix memory leaks
- Optimize database queries

#### High Error Rates
**Symptoms**: Error rate > 5%
**Causes**:
- Resource exhaustion
- External service failures
- Rate limiting
- Invalid input data

**Solutions**:
- Scale resources appropriately
- Implement circuit breakers
- Add retry mechanisms with exponential backoff
- Validate input data early

#### Low Throughput
**Symptoms**: Requests per second < target
**Causes**:
- Bottlenecked components
- Insufficient worker processes
- Blocking I/O operations

**Solutions**:
- Identify and optimize bottlenecks
- Increase worker processes
- Use async/await for I/O operations

### Performance Monitoring Commands

#### System Resource Monitoring
```bash
# Monitor system resources
htop
iostat -x 1
vmstat 1
netstat -i
```

#### Application Monitoring
```bash
# Monitor application logs
tail -f logs/app.log | grep -E "(ERROR|WARNING|SLOW)"

# Monitor Redis
redis-cli --stat

# Monitor Qdrant
curl http://localhost:6333/dashboard
```

### Performance Profiling

#### Python Profiling
```bash
# Profile CPU usage
python -m cProfile -o profile.stats your_script.py

# Profile memory usage
pip install memory-profiler
python -m memory_profiler your_script.py
```

## Best Practices

### Development Best Practices
1. **Use async/await**: For I/O-bound operations
2. **Implement caching**: At multiple levels
3. **Optimize queries**: Use appropriate indexes
4. **Monitor continuously**: Set up alerts for performance degradation
5. **Test regularly**: Run performance tests with each release

### Production Best Practices
1. **Scale horizontally**: Add more instances under load
2. **Use CDN**: For static assets if applicable
3. **Implement circuit breakers**: For external service calls
4. **Set up proper monitoring**: With alerting thresholds
5. **Regular maintenance**: Clean up old data and optimize databases

## Scaling Guidelines

### Horizontal Scaling
- **API Instances**: Scale based on request volume and CPU usage
- **Worker Instances**: Scale based on queue depth and processing time
- **Database**: Scale Qdrant with sharding for large datasets

### Vertical Scaling
- **CPU**: Increase for compute-intensive operations
- **Memory**: Increase for caching and large data processing
- **Storage**: Increase for larger datasets

### Auto-Scaling Configuration
```yaml
# Example Kubernetes auto-scaling configuration
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: rag-api-hpa
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: rag-api
  minReplicas: 2
  maxReplicas: 10
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
  - type: Resource
    resource:
      name: memory
      target:
        type: Utilization
        averageUtilization: 80
```

This performance optimization guide provides a comprehensive approach to maintaining and improving the performance of the RAG Ingestion Pipeline. Regular monitoring, testing, and optimization are essential for maintaining the target performance levels.