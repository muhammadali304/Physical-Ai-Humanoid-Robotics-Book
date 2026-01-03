# Data Model: Frontend ↔ Backend Integration using ChatKit

## Frontend Data Models

### ChatSession
- **id**: string (UUID) - Unique identifier for the conversation session
- **messages**: Array<ChatMessage> - Collection of messages in the conversation
- **createdAt**: ISODateString - Timestamp when session was created
- **lastActiveAt**: ISODateString - Timestamp of last message
- **pageContext**: DocumentationContext - Context of the page where session started
- **uiState**: UIState - Current UI state (minimized, expanded, etc.)

### ChatMessage
- **id**: string (UUID) - Unique identifier for the message
- **role**: "user" | "assistant" - Role of the message sender
- **content**: string - Content of the message
- **timestamp**: ISODateString - When the message was created
- **selectedTextContext?**: string - Optional selected text that was sent with the message
- **status**: "sent" | "sending" | "error" - Status of the message delivery

### DocumentationContext
- **pageUrl**: string - Current page URL
- **pageTitle**: string - Title of the current page
- **selectedText?**: string - Text that was selected when message was sent
- **section?**: string - Section of the page where text was selected

### UIState
- **isMinimized**: boolean - Whether the chat panel is minimized
- **hasUnreadMessages**: boolean - Whether there are unread messages
- **position**: { x: number, y: number } - Position of the chat panel (if floating)
- **size**: { width: number, height: number } - Size of the chat panel

## API Data Models

### ChatRequest
- **sessionId**: string - ID of the conversation session
- **message**: string - The user's message
- **context?**: DocumentationContext - Additional context from the documentation page
- **selectedText?**: string - Text selected by the user (if applicable)

### ChatResponse
- **sessionId**: string - ID of the conversation session
- **response**: string - The AI's response
- **sources**: Array<SourceReference> - References to sources used in the response
- **timestamp**: ISODateString - When the response was generated
- **tokensUsed**: TokenUsage - Information about token consumption

### SourceReference (from existing RAG system)
- **documentId**: string - Unique identifier for the source document
- **title**: string - Title of the source document
- **url?**: string - URL to the source document
- **page?**: number - Page number (for multi-page documents)
- **relevanceScore**: number - Relevance score (0.0 to 1.0)
- **textSnippet?**: string - Snippet of text from the source

### TokenUsage (from existing RAG system)
- **inputTokens**: number - Number of tokens in the input query
- **outputTokens**: number - Number of tokens in the generated response
- **totalTokens**: number - Total tokens consumed

## Validation Rules

### ChatSession
- Must have a valid UUID format for id
- Messages array cannot exceed 100 messages (configurable limit)
- createdAt must be before or equal to lastActiveAt
- pageContext must include a valid URL

### ChatMessage
- Content must be between 1 and 2000 characters
- Role must be either "user" or "assistant"
- Timestamp must be a valid ISO date string
- Status must be one of the defined values

### DocumentationContext
- pageUrl must be a valid URL
- selectedText, if present, must be between 1 and 5000 characters
- pageTitle, if present, must be between 1 and 200 characters

## State Transitions

### ChatSession States
1. **Created**: Session initialized, no messages
2. **Active**: At least one message sent, conversation ongoing
3. **Inactive**: No activity for 30 minutes (potential cleanup)
4. **Archived**: Session completed and archived for history

### ChatMessage States
1. **Sending**: Message sent to backend, awaiting response
2. **Sent**: Message successfully processed by backend
3. **Error**: Error occurred during processing

### UIState Transitions
- **Expanded** ↔ **Minimized**: User toggles the chat panel
- **Minimized** → **Expanded**: User clicks the minimized panel
- **Normal** → **Floating**: Panel is moved to floating position (if supported)