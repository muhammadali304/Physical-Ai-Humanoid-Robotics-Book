# Data Model: RAG Chatbot Agent Backend

## Entity: ChatSession
**Description**: Represents a multi-turn conversation with context history and user interactions

**Fields**:
- `session_id`: UUID (primary key) - Unique identifier for the conversation session
- `created_at`: DateTime - Timestamp when session was created
- `last_activity`: DateTime - Timestamp of last interaction
- `conversation_history`: List[Message] - List of message pairs in the conversation
- `metadata`: Dict[str, Any] - Additional session-specific data

**Relationships**:
- One-to-many with Message entities (contained in conversation_history)

**Validation Rules**:
- session_id must be a valid UUID
- created_at must be in the past
- last_activity must be >= created_at
- conversation_history length must not exceed 50 messages (configurable)

## Entity: Message
**Description**: A single message in a conversation, either from user or agent

**Fields**:
- `role`: str (enum: "user", "assistant") - The role of the message sender
- `content`: str - The text content of the message
- `timestamp`: DateTime - When the message was created
- `metadata`: Dict[str, Any] - Additional message-specific data

**Validation Rules**:
- role must be either "user" or "assistant"
- content must not be empty
- content length must not exceed 10,000 characters

## Entity: QueryRequest
**Description**: Request object for the chatbot query endpoint

**Fields**:
- `query`: str - The user's query text
- `session_id`: Optional[UUID] - Session identifier (creates new if not provided)
- `temperature`: Optional[float] - LLM temperature parameter (0.0-1.0)
- `max_tokens`: Optional[int] - Maximum tokens for response (default: 1024)

**Validation Rules**:
- query must not be empty
- query length must not exceed 5,000 characters
- temperature must be between 0.0 and 1.0 if provided
- max_tokens must be between 1 and 4096 if provided

## Entity: QueryResponse
**Description**: Response object from the chatbot query endpoint

**Fields**:
- `response`: str - The agent's response to the query
- `sources`: List[SourceReference] - List of sources used in the response
- `session_id`: UUID - The session identifier
- `tokens_used`: TokenUsage - Information about token usage
- `timestamp`: DateTime - When the response was generated

**Validation Rules**:
- response must not be empty
- sources list may be empty but must be present
- session_id must be a valid UUID

## Entity: SourceReference
**Description**: Reference to a source document used in the response

**Fields**:
- `title`: str - Title of the source document
- `url`: Optional[str] - URL to the source document
- `text_snippet`: str - Relevant text snippet from the source
- `score`: float - Relevance score (0.0-1.0)

**Validation Rules**:
- title must not be empty
- text_snippet must not be empty
- score must be between 0.0 and 1.0

## Entity: TokenUsage
**Description**: Token usage statistics for the LLM call

**Fields**:
- `input_tokens`: int - Number of tokens in the input
- `output_tokens`: int - Number of tokens in the output
- `total_tokens`: int - Total number of tokens (input + output)

**Validation Rules**:
- All values must be non-negative integers
- total_tokens must equal input_tokens + output_tokens

## Entity: ConversationContext
**Description**: Context object that maintains conversation state for the agent

**Fields**:
- `session_id`: UUID - The session identifier
- `history_summary`: str - Brief summary of conversation history
- `current_topic`: Optional[str] - Current topic of conversation
- `follow_up_context`: Optional[str] - Context for follow-up questions

**Validation Rules**:
- session_id must be a valid UUID
- history_summary length must not exceed 2000 characters

## State Transitions

### ChatSession
- **Created**: When a new query comes without a session_id or with an invalid session_id
- **Active**: When the session receives new messages (last_activity updated)
- **Expired**: When last_activity exceeds the timeout threshold (30 minutes)
- **Cleared**: When the session is explicitly cleared or automatically cleaned up

## Relationships

```
ChatSession 1 ---- * Message
QueryRequest 1 ---- 1 ChatSession (via session_id)
QueryResponse 1 ---- 1 ChatSession (via session_id)
QueryResponse * ---- * SourceReference (sources)
QueryResponse 1 ---- 1 TokenUsage (tokens_used)
```

## Constraints

1. **Session Isolation**: Messages from one session cannot be accessed from another session
2. **History Limit**: Conversation history is limited to prevent excessive memory usage
3. **Timeout Enforcement**: Sessions are automatically expired after inactivity period
4. **Data Integrity**: All timestamps are stored in UTC
5. **Response Grounding**: Responses must be based only on retrieved context from Qdrant