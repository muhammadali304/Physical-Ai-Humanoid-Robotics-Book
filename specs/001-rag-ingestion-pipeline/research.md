# Research Summary: RAG Ingestion Pipeline

## Target Documentation Sites

### Decision
Use Claude documentation site as the primary target for initial development and testing: https://docs.anthropic.com/en/docs

### Rationale
The Claude documentation site is a Docusaurus-based documentation site that is publicly accessible and appropriate for testing the ingestion pipeline. It provides a good example of the type of content the pipeline is designed to process.

### Alternatives Considered
- Creating a dummy Docusaurus site (wouldn't reflect real-world complexity)
- Using other public documentation sites (would need verification of permissibility)
- Using the Physical AI Humanoid Robotics Book site (already exists in this repo)

## Docusaurus Site Structure Analysis

### Decision
Use CSS selectors targeting `.main-wrapper .markdown`, `.theme-doc-markdown`, and `.doc-content` classes for content extraction, with fallbacks for different Docusaurus versions.

### Rationale
These are common class names used in Docusaurus sites for main content areas. This approach allows for extracting clean text while avoiding navigation, headers, and other non-content elements.

### Alternatives Considered
- Selenium for JavaScript-heavy sites (more resource intensive)
- Custom parsing rules per site (not scalable)

## Cohere Embedding Model Selection

### Decision
Use Cohere's `embed-multilingual-v3.0` model with 1024-dimensional embeddings.

### Rationale
This model provides good performance for documentation content, handles multiple languages, and offers a good balance between cost and quality. The multilingual capability is useful for diverse documentation sets.

### Alternatives Considered
- OpenAI embeddings (higher cost, vendor lock-in)
- Hugging Face open models (higher computational requirements, self-hosting needs)

## Qdrant Cloud Configuration

### Decision
Use Qdrant Cloud's free tier initially, with option to upgrade as needed. Configure collection with 1024-dimensional vectors and cosine similarity.

### Rationale
The free tier provides sufficient capacity for development and initial testing while allowing easy scaling. Qdrant is specifically designed for vector similarity search which is ideal for RAG applications.

### Alternatives Considered
- Self-hosted Qdrant (requires infrastructure management)
- Other vector databases like Pinecone or Weaviate (different pricing/models)

## Content Chunking Strategy

### Decision
Use semantic chunking based on document structure (headings, paragraphs) with maximum 512 tokens per chunk and 20% overlap between chunks.

### Rationale
Semantic chunking preserves context and meaning better than fixed-size chunking, which is important for RAG quality. The overlap helps maintain context across chunk boundaries.

### Alternatives Considered
- Fixed-size token chunking (might break semantic boundaries)
- Sentence-based chunking (might create too many small chunks)