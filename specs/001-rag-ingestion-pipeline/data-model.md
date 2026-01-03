# Data Model: RAG Ingestion Pipeline

## Entity: ContentChunk

Represents a semantically meaningful segment of documentation text that has been processed and prepared for vector storage.

### Fields
- `id`: UUID (Primary Key) - Unique identifier for the content chunk
- `source_url`: String (URL) - Original URL from which the content was extracted
- `page_title`: String (Text, max 500 chars) - Title of the source page
- `section_heading`: String (Text, nullable, max 500 chars) - Heading under which this content appears
- `chunk_index`: Integer - Sequential position of this chunk within the source document
- `content`: Text - The raw text content of this chunk
- `token_count`: Integer - Number of tokens in the content (for size validation)
- `metadata`: JSON - Additional metadata including extraction information
- `created_at`: DateTime - Timestamp when the chunk was created
- `updated_at`: DateTime - Timestamp when the chunk was last updated

### Relationships
- `embedding_vector` (1-to-1): Reference to the associated EmbeddingVector
- `crawl_job` (Many-to-1): Reference to the CrawlJob that created this chunk

### Validation Rules
- Content must be at least 50 characters long
- Token count must be between 50 and 1000 tokens
- Source URL must be a valid URL format

## Entity: CrawlJob

Represents a crawling operation that processes a target website and creates content chunks.

### Fields
- `id`: UUID (Primary Key) - Unique identifier for the crawl job
- `target_url`: String (URL) - Root URL to crawl
- `status`: Enum (pending, processing, completed, failed, cancelled) - Current status of the job
- `progress`: Integer (0-100) - Percentage completion of the job
- `total_pages`: Integer - Total number of pages identified to process
- `processed_pages`: Integer - Number of pages processed so far
- `failed_pages`: Integer - Number of pages that failed processing
- `options`: JSON - Configuration options for the crawl (selectors, filters, etc.)
- `error_log`: Text (nullable) - Details about errors encountered during the crawl
- `created_at`: DateTime - Timestamp when the job was created
- `updated_at`: DateTime - Timestamp when the job was last updated
- `completed_at`: DateTime (nullable) - Timestamp when the job was completed

### Validation Rules
- Target URL must be a valid URL format
- Status must be one of the defined enum values
- Progress must be between 0 and 100

## Entity: EmbeddingVector

Represents the semantic embedding vector for a content chunk, stored in the vector database.

### Fields
- `id`: UUID (Primary Key) - Unique identifier for the embedding
- `content_chunk_id`: UUID (Foreign Key) - Reference to the associated ContentChunk
- `vector_data`: Array of Float - The embedding vector values (1024-dimensional for Cohere v3)
- `model_used`: String - Identifier of the embedding model used (e.g., "cohere/embed-multilingual-v3.0")
- `model_version`: String - Version of the embedding model
- `dimensions`: Integer - Number of dimensions in the vector
- `created_at`: DateTime - Timestamp when the embedding was generated

### Relationships
- `content_chunk` (1-to-1): Reference to the associated ContentChunk

### Validation Rules
- Vector data must have exactly 1024 dimensions (for Cohere v3 model)
- Model used must be a supported embedding model
- Dimensions must match the expected size for the model

## Entity: ProcessingLog

Represents logs of processing operations for debugging and monitoring purposes.

### Fields
- `id`: UUID (Primary Key) - Unique identifier for the log entry
- `job_id`: UUID (Foreign Key) - Reference to the associated CrawlJob
- `chunk_id`: UUID (Foreign Key, nullable) - Reference to the associated ContentChunk if applicable
- `level`: Enum (info, warning, error, debug) - Severity level of the log
- `message`: Text - Log message content
- `context`: JSON - Additional context information about the log event
- `created_at`: DateTime - Timestamp when the log was created

### Relationships
- `crawl_job` (Many-to-1): Reference to the associated CrawlJob
- `content_chunk` (Many-to-1, nullable): Reference to the associated ContentChunk

## Relationships Overview

```
CrawlJob (1) <---> (Many) ContentChunk (1) <---> (1) EmbeddingVector
     |
     |
     -----> (Many) ProcessingLog
```

- A CrawlJob creates multiple ContentChunks
- Each ContentChunk has exactly one EmbeddingVector
- Both CrawlJob and ContentChunk can have multiple ProcessingLog entries