# Quickstart: RAG Chatbot Agent Backend

## Prerequisites

- Python 3.14 or higher
- pip package manager
- Google Gemini API key
- Access to Qdrant vector database (already configured in the system)

## Environment Setup

1. **Set up environment variables**:
   ```bash
   export GEMINI_API_KEY="your-gemini-api-key-here"
   export GEMINI_BASE_URL="https://generativelanguage.googleapis.com/v1beta/openai/"
   export GEMINI_MODEL="gemini-1.5-pro"
   ```

2. **Install dependencies** (if not already installed):
   ```bash
   pip install -r requirements.txt
   ```

## Running the Service

1. **Start the backend service**:
   ```bash
   cd backend
   uvicorn main:app --host 0.0.0.0 --port 8000 --reload
   ```

2. **Verify the service is running**:
   - Navigate to `http://localhost:8000/docs` to access the API documentation
   - The `/query` endpoint should be available for RAG chatbot queries

## Making Your First Query

1. **Using curl**:
   ```bash
   curl -X POST "http://localhost:8000/api/v1/query" \
     -H "Content-Type: application/json" \
     -d '{
       "query": "What is the main purpose of this system?",
       "session_id": "123e4567-e89b-12d3-a456-426614174000"
     }'
   ```

2. **Using Python requests**:
   ```python
   import requests
   import uuid

   response = requests.post(
       "http://localhost:8000/api/v1/query",
       json={
           "query": "What is the main purpose of this system?",
           "session_id": str(uuid.uuid4())
       }
   )
   print(response.json())
   ```

## Multi-turn Conversation Example

1. **Start a conversation**:
   ```bash
   # First query (creates session)
   curl -X POST "http://localhost:8000/api/v1/query" \
     -H "Content-Type: application/json" \
     -d '{
       "query": "Explain how RAG works",
       "session_id": "123e4567-e89b-12d3-a456-426614174000"
     }'
   ```

2. **Follow-up in the same session**:
   ```bash
   # Follow-up query (uses same session)
   curl -X POST "http://localhost:8000/api/v1/query" \
     -H "Content-Type: application/json" \
     -d '{
       "query": "Can you give me a practical example?",
       "session_id": "123e4567-e89b-12d3-a456-426614174000"
     }'
   ```

## API Endpoints

- `POST /api/v1/query` - Process a query through the RAG chatbot agent
- `POST /api/v1/query/batch` - Process multiple queries in batch
- `GET /api/v1/query/health` - Health check for the query service
- `GET /api/v1/query/stats` - Get query processing statistics

## Configuration Options

- **Session timeout**: 30 minutes of inactivity (configurable)
- **Maximum response tokens**: 1024 (configurable per request)
- **LLM temperature**: 0.7 (configurable per request)

## Troubleshooting

1. **API Key Issues**: Ensure `GEMINI_API_KEY` is properly set
2. **Qdrant Connection**: Verify Qdrant service is running and accessible
3. **Response Quality**: If responses seem irrelevant, check that documents are properly indexed in Qdrant

## Next Steps

1. Integrate with your frontend chatbot interface
2. Configure production-level security and monitoring
3. Set up proper session persistence if needed for production
4. Monitor token usage and API costs