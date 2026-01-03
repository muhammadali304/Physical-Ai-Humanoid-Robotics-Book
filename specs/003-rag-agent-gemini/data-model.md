# Data Model: RAG Agent Backend with OpenAI Agents SDK using Gemini

**Feature**: 003-rag-agent-gemini
**Created**: 2025-12-24
**Status**: Draft

## Core Entities

### QueryRequest
**Description**: Input model for user queries to the RAG system

**Fields**:
- `query`: str (required) - The user's natural language question
- `session_id`: Optional[str] - Identifier for conversation context (for multi-turn conversations)
- `user_id`: Optional[str] - Identifier for authenticated user (for access control)
- `metadata`: Optional[dict] - Additional context or parameters for the query

**Validation Rules**:
- `query` must be between 1 and 1000 characters
- `session_id` must be a valid UUID if provided
- `user_id` must be a valid identifier if provided

### QueryResponse
**Description**: Output model for responses from the RAG system

**Fields**:
- `response`: str (required) - The generated answer to the user's query
- `sources`: List[SourceReference] (required) - List of documents used in generating the response
- `session_id`: Optional[str] - Identifier for conversation context
- `tokens_used`: Optional[TokenUsage] - Information about token consumption
- `retrieval_info`: Optional[RetrievalInfo] - Details about the retrieval process

**Validation Rules**:
- `response` must not be empty
- `sources` must contain at least one reference if context was used
- `session_id` must be a valid UUID if provided

### SourceReference
**Description**: Reference to a source document used in the response

**Fields**:
- `document_id`: str (required) - Unique identifier for the source document
- `title`: str (required) - Title of the source document
- `url`: Optional[str] - URL to the source document (if applicable)
- `page`: Optional[int] - Page number (for multi-page documents)
- `relevance_score`: float - Score indicating how relevant this source was to the query (0.0 to 1.0)
- `text_snippet`: Optional[str] - Snippet of text from the source (for verification)

**Validation Rules**:
- `document_id` must be a valid identifier
- `relevance_score` must be between 0.0 and 1.0
- `page` must be positive if provided

### DocumentChunk
**Description**: Individual chunks of ingested documents stored in the vector database

**Fields**:
- `id`: str (required) - Unique identifier for this chunk
- `content`: str (required) - The text content of this chunk
- `embedding`: List[float] (required) - Vector representation for semantic search
- `document_id`: str (required) - Reference to the original document
- `document_title`: str (required) - Title of the original document
- `document_url`: Optional[str] - URL of the original document
- `chunk_index`: int (required) - Position of this chunk in the original document
- `metadata`: dict - Additional metadata about the chunk
- `created_at`: str - Timestamp of when this chunk was created
- `updated_at`: str - Timestamp of last update

**Validation Rules**:
- `id` must be unique across all chunks
- `embedding` must have the correct dimension for the configured model
- `chunk_index` must be non-negative
- `content` must not be empty

### TokenUsage
**Description**: Information about token consumption during query processing

**Fields**:
- `input_tokens`: int (required) - Number of tokens in the input query
- `output_tokens`: int (required) - Number of tokens in the generated response
- `total_tokens`: int (required) - Total tokens consumed

**Validation Rules**:
- All values must be non-negative integers
- `total_tokens` must equal `input_tokens + output_tokens`

### RetrievalInfo
**Description**: Information about the document retrieval process

**Fields**:
- `retrieved_chunks`: int (required) - Number of document chunks retrieved
- `search_time_ms`: float (required) - Time taken for the retrieval process in milliseconds
- `top_k`: int (required) - Number of top results requested
- `relevance_threshold`: Optional[float] - Minimum relevance score for inclusion

**Validation Rules**:
- `retrieved_chunks` must be non-negative
- `search_time_ms` must be positive
- `top_k` must be positive

### UserSession
**Description**: Information about a user's session with the RAG system

**Fields**:
- `session_id`: str (required) - Unique identifier for the session
- `user_id`: Optional[str] - Reference to the authenticated user
- `created_at`: str (required) - Timestamp of session creation
- `last_accessed_at`: str (required) - Timestamp of last access
- `query_history`: List[QueryRecord] - History of queries in this session

**Validation Rules**:
- `session_id` must be unique
- `last_accessed_at` must be after `created_at`

### QueryRecord
**Description**: Record of a single query within a session

**Fields**:
- `query_id`: str (required) - Unique identifier for this query
- `query_text`: str (required) - The original query text
- `response_id`: str (required) - Reference to the response
- `timestamp`: str (required) - When the query was made
- `metadata`: Optional[dict] - Additional metadata about the query

**Validation Rules**:
- `query_id` must be unique
- `timestamp` must be a valid ISO 8601 string

## Relationships

### DocumentChunk → SourceReference
- Each DocumentChunk can be referenced by multiple SourceReference instances
- When a query retrieves relevant chunks, SourceReference objects are created to represent the connection

### QueryRequest → UserSession
- Each QueryRequest optionally belongs to a UserSession
- UserSession maintains history of QueryRecord objects

### QueryResponse → SourceReference
- Each QueryResponse contains multiple SourceReference objects
- SourceReference objects point to the DocumentChunk objects that informed the response

## State Transitions

### Query Processing States
1. **Received**: QueryRequest received by the system
2. **Retrieving**: System is searching for relevant documents
3. **Generating**: System is generating response using retrieved context
4. **Completed**: Final response ready for user
5. **Failed**: Error occurred during processing

## Constraints

### Access Control
- Only authenticated users can access documents they have permissions for
- Document access is governed by user roles and document-level permissions

### Performance
- Query responses must be returned within 10 seconds
- Embedding generation must be cached to avoid redundant computation

### Data Integrity
- Document chunks must maintain their original content without modification
- Embeddings must be regenerated when source content changes