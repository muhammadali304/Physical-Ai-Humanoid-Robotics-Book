# Implementation Plan: Website Content Ingestion and Vectorization Pipeline for RAG Chatbot

**Feature**: 001-rag-ingestion-pipeline
**Created**: 2025-12-22
**Status**: Draft
**Spec Reference**: [spec.md](./spec.md)

## Technical Context

- **Primary Language**: Python
- **Package Manager**: UV
- **Framework**: FastAPI (for any API components)
- **Data Storage**: Qdrant Cloud vector database
- **External APIs**: Cohere embedding API
- **Target Deployment**: Cloud-based pipeline
- **Architecture**: Event-driven with message queues
- **Backend Directory**: `backend/` (to be created)

### Technology Decisions

- **Web Scraping**: BeautifulSoup4/lxml for HTML parsing
- **Embeddings**: Cohere API for semantic embeddings
- **Vector Storage**: Qdrant Cloud for vector similarity search
- **Message Queues**: Redis/RQ or Celery for job processing
- **Environment Management**: Python-dotenv for configuration
- **HTTP Client**: httpx for async API calls

### Known Unknowns

- Specific Docusaurus site structure patterns for scraping [RESOLVED in research.md]
- Cohere embedding model selection and pricing [RESOLVED in research.md]
- Qdrant Cloud tier and performance characteristics [RESOLVED in research.md]
- Rate limits for external APIs [RESOLVED in research.md]
- Target documentation site URLs to process [RESOLVED in research.md]

## Constitution Check

### Code Quality Standards
- Follow PEP 8 style guidelines
- Use type hints throughout the codebase
- Write comprehensive unit tests (target 80%+ coverage)
- Document all public interfaces

### Security Requirements
- Secure handling of API keys and credentials
- Input validation for all external content
- Rate limiting to prevent abuse
- Data encryption for stored content

### Performance Requirements
- Efficient memory usage during processing
- Asynchronous processing where possible
- Proper error handling and retry mechanisms
- Monitoring and observability implementation

### Architecture Principles
- Separate concerns with clean architecture
- Use dependency injection for testability
- Implement proper logging
- Design for horizontal scaling

## Gates

### Pre-Implementation Checks
- [x] All [NEEDS CLARIFICATION] items resolved
- [ ] Architecture decisions documented in ADRs if significant
- [x] Security requirements validated
- [x] Performance requirements validated

### Compliance Verification
- [x] No hardcoded secrets in source code (using environment variables)
- [x] Proper error handling for all external dependencies
- [x] Input validation for all user-provided content
- [x] Rate limiting implemented for external API calls

---

## Phase 0: Research & Discovery

### research.md

#### Research Task: Docusaurus Site Structure Analysis
- **Decision**: Use BeautifulSoup with specific selectors for Docusaurus sites
- **Rationale**: Docusaurus sites have consistent class names and structure patterns
- **Alternatives considered**: Selenium (heavier), custom parsers

#### Research Task: Cohere Embedding Model Selection
- **Decision**: Use Cohere's embed-multilingual-v3.0 model
- **Rationale**: Good performance for documentation content, handles multiple languages
- **Alternatives considered**: OpenAI embeddings, Hugging Face models

#### Research Task: Qdrant Cloud Configuration
- **Decision**: Use Qdrant Cloud managed service with appropriate tier
- **Rationale**: Managed service reduces operational overhead
- **Alternatives considered**: Self-hosted Qdrant, other vector databases

#### Research Task: Content Chunking Strategy
- **Decision**: Semantic chunking based on document structure
- **Rationale**: Better for RAG quality than fixed-size chunking
- **Alternatives considered**: Fixed-size chunking, sentence-based chunking

#### Research Task: Target Documentation Sites
- **Decision**: Use Claude documentation site as primary target: https://docs.anthropic.com/en/docs
- **Rationale**: Publicly accessible Docusaurus site appropriate for testing
- **Alternatives considered**: Generic approach without specific targets

---

## Phase 1: Design & Architecture

### data-model.md

#### Entity: ContentChunk
- **Fields**:
  - id: UUID (primary key)
  - source_url: String (URL of source document)
  - page_title: String (title of source page)
  - chunk_index: Integer (position in document)
  - content: Text (raw text content)
  - embedding: Vector (semantic embedding)
  - metadata: JSON (additional metadata)
  - created_at: DateTime
  - updated_at: DateTime

#### Entity: CrawlJob
- **Fields**:
  - id: UUID (primary key)
  - target_url: String (URL to crawl)
  - status: Enum (pending, processing, completed, failed)
  - progress: Integer (percentage complete)
  - total_pages: Integer (total pages to process)
  - processed_pages: Integer (pages processed)
  - created_at: DateTime
  - updated_at: DateTime

#### Entity: EmbeddingVector
- **Fields**:
  - id: UUID (primary key)
  - content_chunk_id: UUID (foreign key to ContentChunk)
  - vector_data: Vector (the embedding vector)
  - model_used: String (embedding model identifier)
  - created_at: DateTime

#### Relationships:
- CrawlJob 1-* ContentChunk (one crawl job creates many content chunks)
- ContentChunk 1-1 EmbeddingVector (one content chunk has one embedding vector)

### API Contracts

#### Crawl Management API
```
POST /api/v1/crawl-jobs
- Request: {target_url: string, options?: object}
- Response: {id: uuid, status: string, created_at: datetime}

GET /api/v1/crawl-jobs/{id}
- Response: {id: uuid, target_url: string, status: string, progress: number, ...}

GET /api/v1/crawl-jobs
- Response: [{id: uuid, target_url: string, status: string, ...}]
```

#### Content Retrieval API
```
GET /api/v1/content-chunks
- Query: {source_url?: string, page_title?: string}
- Response: [{id: uuid, source_url: string, content: string, ...}]

GET /api/v1/content-chunks/{id}
- Response: {id: uuid, source_url: string, content: string, metadata: object}
```

#### Vector Search API
```
POST /api/v1/search
- Request: {query: string, top_k?: number}
- Response: [{content_chunk: object, similarity: number}]
```

### quickstart.md

# Quick Start: RAG Ingestion Pipeline

## Prerequisites
- Python 3.9+
- UV package manager
- Cohere API key
- Qdrant Cloud cluster credentials

## Setup

1. Create backend directory:
```bash
mkdir backend && cd backend
```

2. Initialize project with UV:
```bash
uv init
uv add fastapi httpx beautifulsoup4 lxml cohere qdrant-client python-dotenv
```

3. Set environment variables:
```bash
# Create .env file
echo "COHERE_API_KEY=your_cohere_key" >> .env
echo "QDRANT_URL=your_qdrant_url" >> .env
echo "QDRANT_API_KEY=your_qdrant_key" >> .env
```

4. Run the pipeline:
```bash
python -m scripts.crawl_and_embed --url "https://target-docs.example.com"
```

## Development

```bash
# Install development dependencies
uv add --dev pytest black mypy

# Run tests
python -m pytest

# Format code
black .

# Type check
mypy .
```

### Agent Context Update

The following technologies will be added to the agent context:
- Python (3.9+)
- FastAPI
- Cohere API
- Qdrant vector database
- BeautifulSoup4
- UV package manager
- AsyncIO patterns

---

## Phase 2: Implementation Strategy

### Implementation Order
1. Backend project setup and configuration
2. Web crawling and content extraction module
3. Content chunking and preprocessing
4. Embedding generation with Cohere
5. Vector storage in Qdrant
6. API endpoints and job management
7. Testing and validation
8. Monitoring and observability

### Risk Mitigation
- Implement with external API mocks for testing
- Use configuration flags for development vs production
- Build in rate limiting and retry mechanisms
- Include comprehensive error handling