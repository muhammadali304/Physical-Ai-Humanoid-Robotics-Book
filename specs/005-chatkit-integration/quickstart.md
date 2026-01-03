# Quickstart: Frontend ↔ Backend Integration using ChatKit

## Prerequisites

- Node.js 18+ and npm/yarn
- Access to existing RAG backend API
- API key for RAG backend authentication
- Docusaurus documentation site set up

## Setup Process

### 1. Install Dependencies
```bash
cd physical-ai-book
npm install
```

### 2. Environment Configuration
Update `.env` file in the physical-ai-book directory with:
```
REACT_APP_BACKEND_URL=https://your-rag-backend.com
RAG_API_BASE_URL=https://your-rag-backend.com/api/v1
RAG_API_KEY=your_api_key_here
CHAT_SESSION_TIMEOUT=1800  # 30 minutes in seconds
```

### 3. Backend API Preparation
Ensure the RAG backend API endpoint is available:
- Endpoint: `POST /api/v1/query`
- Expected request format: `{ query: string, session_id?: string, selected_text?: string }`
- Expected response format: `{ response: string, sources: Array<SourceReference>, session_id: string }`

### 4. Frontend Integration
The ChatKit component is already integrated at the layout level via the Root component.
1. The ChatKit component is in `physical-ai-book/src/components/ChatKit/`
2. The component is integrated at the layout level in `physical-ai-book/src/theme/Root.js`
3. API client connects to RAG backend via `physical-ai-book/src/services/api-client.js`
4. Selected text functionality is handled by `physical-ai-book/src/components/ChatKit/SelectedTextHandler.jsx`

## Running the Integration

### Development
```bash
cd physical-ai-book
npm run start
```
The ChatKit component should appear on all documentation pages.

### Production Build
```bash
cd physical-ai-book
npm run build
npm run serve
```

## Key Features Setup

### Selected Text Integration
- Select any text in documentation
- The ChatKit interface will recognize the selection
- Selected text is automatically included as context in the query

### UI State Persistence
- Chat panel minimize/expand state persists across page navigation
- Conversation history maintained during session
- Session timeout after 30 minutes of inactivity

### API Communication
- All chat messages sent to RAG backend via configured endpoint
- API key authentication included in headers
- Error handling for failed requests with user feedback

## Testing the Integration

### Basic Functionality
1. Navigate to any documentation page
2. Verify ChatKit component appears correctly
3. Send a test message and verify response
4. Test minimize/expand functionality

### Selected Text Feature
1. Select text in documentation
2. Verify it can be sent to chat as context
3. Check that response incorporates the selected context

### Cross-Page Navigation
1. Start a conversation on one page
2. Navigate to a different documentation page
3. Verify conversation history persists
4. Continue conversation to verify context maintenance

## Troubleshooting

### ChatKit Not Appearing
- Verify component is properly integrated in Root.js
- Check browser console for JavaScript errors
- Ensure all dependencies are installed

### API Communication Issues
- Verify RAG_API_BASE_URL is correctly configured
- Check that RAG_API_KEY is valid
- Confirm backend service is running and accessible

### Selected Text Not Working
- Verify selection detection JavaScript is properly implemented
- Check browser compatibility
- Ensure no conflicts with existing documentation page scripts