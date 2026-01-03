# Implementation Plan: RAG Agent Backend with OpenAI Agents SDK using Gemini

**Feature**: 003-rag-agent-gemini
**Created**: 2025-12-24
**Status**: Draft
**Input**: (Technical Plan)
- Initialize `backend/` project with UV, set up FastAPI app structure and environment config
- Configure OpenAI Agents SDK to use **Gemini via OpenAI-compatible API** with system instructions
- Implement Qdrant client and semantic retrieval (top-k documents per query)
- Inject retrieved context into agent prompt to enforce grounded responses
- Expose and test a clean POST `/query` API endpoint with error handling

## Technical Context

### Architecture Overview
- **Backend Framework**: FastAPI for high-performance async API
- **Dependency Management**: UV (Python package manager)
- **AI Integration**: OpenAI Agents SDK configured for Google Gemini via OpenAI-compatible API
- **Vector Database**: Qdrant for semantic search and retrieval
- **API Design**: Clean REST API with POST `/query` endpoint
- **Environment**: Configurable via environment variables

### Core Components
- **Agent Service**: OpenAI Agents SDK wrapper for Gemini integration
- **Retrieval Service**: Qdrant client for semantic document retrieval
- **Query Handler**: Orchestrates retrieval and generation process
- **API Layer**: FastAPI endpoints with proper error handling

### Technology Stack
- **Language**: Python 3.10+
- **Framework**: FastAPI
- **AI SDK**: OpenAI Agents SDK (configured for Gemini)
- **Vector DB**: Qdrant
- **Package Manager**: UV
- **API Standard**: OpenAPI 3.0

### Known Unknowns
- Specific Gemini model name to use (NEEDS CLARIFICATION)
- Exact Qdrant configuration parameters (NEEDS CLARIFICATION)
- System instructions prompt template (NEEDS CLARIFICATION)
- Top-k retrieval count (NEEDS CLARIFICATION)

## Constitution Check

### Alignment with Project Principles
- **Technical accuracy**: Using established frameworks (FastAPI, Qdrant) and official SDKs
- **Educational clarity**: Clean architecture with well-defined components
- **Practical applicability**: Real-world RAG implementation with proper error handling
- **Progressive complexity**: Modular design allowing incremental development

### Compliance Verification
- All code will follow Python PEP 8 standards
- Dependencies will be properly specified and versioned
- API will follow REST best practices
- Error handling will be comprehensive
- Configuration will be environment-based

### Gate Evaluation
- ✅ Technical accuracy: Using official SDKs and established frameworks
- ✅ Educational clarity: Well-structured, modular components
- ✅ Practical applicability: Production-ready architecture
- ✅ Progressive complexity: Component-based design

## Phase 0: Research

### Research Tasks

#### 0.1 Gemini API Configuration Research
**Task**: Research how to configure OpenAI Agents SDK to work with Google's Gemini via OpenAI-compatible API
- Decision: Use Google's OpenAI-compatible endpoint for Gemini
- Rationale: Allows leveraging existing OpenAI SDK patterns while using Gemini's capabilities
- Alternatives considered: Direct Gemini API vs OpenAI-compatible API

#### 0.2 Qdrant Integration Patterns
**Task**: Research best practices for Qdrant integration with FastAPI
- Decision: Use async Qdrant client for non-blocking operations
- Rationale: Maintains FastAPI's async performance characteristics
- Alternatives considered: Sync vs async clients, connection pooling strategies

#### 0.3 Retrieval-Augmented Generation Patterns
**Task**: Research RAG implementation patterns for context injection
- Decision: Use context injection with system instructions for grounded responses
- Rationale: Ensures generated responses are based on retrieved documents
- Alternatives considered: Different context injection strategies

#### 0.4 API Error Handling Standards
**Task**: Research best practices for API error handling in FastAPI
- Decision: Use FastAPI's exception handlers with standardized error responses
- Rationale: Provides consistent error reporting to clients
- Alternatives considered: Different error response formats

## Phase 1: Design

### 1.1 Data Model

#### Core Entities
- **QueryRequest**: Input from user containing the question
  - query: str (the user's question)
  - session_id: Optional[str] (for conversation context)
  - metadata: Optional[dict] (additional context)

- **QueryResponse**: Output to user containing answer and sources
  - response: str (the generated answer)
  - sources: List[dict] (citations to source documents)
  - session_id: Optional[str] (conversation context)
  - tokens_used: dict (token usage metrics)

- **DocumentChunk**: Individual chunks of ingested documents
  - id: str (unique identifier)
  - content: str (the text content)
  - embedding: List[float] (vector representation)
  - metadata: dict (source, page, etc.)

### 1.2 API Contracts

#### Query Endpoint
```
POST /query
Content-Type: application/json
Authorization: Bearer {token}

Request Body:
{
  "query": "What is the recommended approach for robot kinematics?",
  "session_id": "optional-session-identifier"
}

Response (200):
{
  "response": "The recommended approach for robot kinematics...",
  "sources": [
    {
      "document_id": "doc-123",
      "title": "Robotics Kinematics Guide",
      "page": 15,
      "relevance_score": 0.87
    }
  ],
  "session_id": "session-identifier",
  "tokens_used": {
    "input": 45,
    "output": 128
  }
}

Error Response (400, 401, 500):
{
  "error": "Error message",
  "error_code": "ERROR_CODE",
  "details": "Additional error details"
}
```

### 1.3 Quickstart Guide

#### Development Setup
1. Navigate to the backend directory
2. Install dependencies with UV: `uv pip install -r requirements.txt`
3. Set environment variables (API keys, URLs)
4. Start the development server: `uv run main.py`

#### Required Environment Variables
- `GEMINI_API_KEY`: Google Gemini API key
- `QDRANT_URL`: Qdrant database URL
- `QDRANT_API_KEY`: Qdrant database API key
- `QDRANT_COLLECTION_NAME`: Name of the collection for document embeddings

#### Testing the API
1. Start the server: `uvicorn main:app --reload`
2. Send a test query:
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is this system about?"}'
```

## Phase 2: Implementation Strategy

### 2.1 Component Development Order
1. **Infrastructure**: Project setup, dependencies, configuration
2. **Data Layer**: Qdrant client, document models
3. **AI Integration**: OpenAI SDK configuration for Gemini
4. **Services**: Retrieval and generation services
5. **API Layer**: FastAPI endpoints
6. **Testing**: Integration and unit tests
7. **Documentation**: API docs and usage guides

### 2.2 Risk Mitigation
- **API Compatibility**: Thorough testing of OpenAI SDK with Gemini endpoint
- **Performance**: Async implementation to handle concurrent requests
- **Error Handling**: Comprehensive error cases for all services
- **Security**: Proper authentication and input validation

### 2.3 Success Criteria
- API endpoint successfully processes queries
- Context injection produces grounded responses
- Semantic retrieval returns relevant documents
- System handles errors gracefully
- Performance meets expected SLAs