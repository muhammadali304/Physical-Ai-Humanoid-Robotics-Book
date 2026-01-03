/**
 * ChatInterface component for the ChatKit UI
 */
import React, { useState, useRef, useEffect, useCallback } from 'react';
import { ChatMessage } from '../../models/chat-message';
import { ApiClient } from '../../services/api-client';
import { DocumentationContext } from '../../models/doc-context';

export const ChatInterface = ({
  chatSession,
  addMessage,
  toggleMinimize,
  isLoading,
  setIsLoading,
  error,
  setError,
  uiState,
  setUiState
}) => {
  const [inputValue, setInputValue] = useState('');
  const [messages, setMessages] = useState([]);
  const [selectedText, setSelectedText] = useState(null);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);
  const messagesContainerRef = useRef(null);
  const [isUserScrolling, setIsUserScrolling] = useState(false);

  const apiClientRef = useRef(new ApiClient());

  // Load messages from chat session
  useEffect(() => {
    if (chatSession) {
      setMessages(chatSession.getMessages());
    }
  }, [chatSession]);

  // Scroll to bottom when messages change, but only if user is near the bottom
  useEffect(() => {
    if (!isUserScrolling) {
      scrollToBottom();
    }
  }, [messages]);

  // Handle user scroll events to determine if they're viewing history
  const handleScroll = useCallback(() => {
    if (messagesContainerRef.current) {
      const { scrollTop, scrollHeight, clientHeight } = messagesContainerRef.current;
      // If user is scrolled up more than 100px from the bottom, set isUserScrolling to true
      const isNearBottom = scrollHeight - scrollTop - clientHeight < 100;
      setIsUserScrolling(!isNearBottom);
    }
  }, []);

  // Reset user scrolling state when new messages arrive and we're auto-scrolling
  useEffect(() => {
    if (messagesContainerRef.current) {
      const { scrollHeight, clientHeight } = messagesContainerRef.current;
      const isNearBottom = scrollHeight - clientHeight < 100;
      if (isNearBottom) {
        setIsUserScrolling(false);
      }
    }
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSendMessage = useCallback(async (e) => {
    e.preventDefault();

    // Validate the message before sending
    const trimmedInput = inputValue.trim();
    if (!trimmedInput) return;

    // Validate message length (backend requires 1-2000 characters)
    if (trimmedInput.length < 1 || trimmedInput.length > 2000) {
      setError('Message content must be between 1 and 2000 characters');
      return;
    }

    try {
      setIsLoading(true);
      setError(null);

      // Validate message content before creating ChatMessage to prevent constructor errors
      if (!trimmedInput || trimmedInput.length < 1 || trimmedInput.length > 2000) {
        setError('Message content must be between 1 and 2000 characters');
        return;
      }

      // Create user message
      const userMessage = new ChatMessage(trimmedInput, 'user', selectedText);
      addMessage(userMessage);
      setMessages(prev => [...prev, userMessage]);

      // Prepare context
      const context = new DocumentationContext(
        window.location.href,
        document.title,
        selectedText || null
      );

      // Send message to API
      const response = await apiClientRef.current.sendWithSelectedText(
        trimmedInput,
        selectedText || '',
        chatSession?.id
      );

      // Validate assistant response before creating ChatMessage to prevent constructor errors
      const assistantResponse = response.response;
      if (!assistantResponse || assistantResponse.length < 1) {
        // Create error message instead of assistant message for empty responses
        const errorMessage = new ChatMessage(
          `I received an empty response from the documentation system.`,
          'assistant'
        );
        setMessages(prev => [...prev, errorMessage]);
        return;
      }

      // Truncate very long responses to fit within ChatMessage limits
      const truncatedResponse = assistantResponse.length > 2000
        ? assistantResponse.substring(0, 2000) + '...'
        : assistantResponse;

      // Create assistant message
      const assistantMessage = new ChatMessage(
        truncatedResponse,
        'assistant',
        null
      );

      addMessage(assistantMessage);
      setMessages(prev => [...prev, assistantMessage]);

      // Mark unread if chat is minimized
      if (uiState.isMinimized) {
        setUiState(prevUiState => {
          if (!prevUiState) return prevUiState;
          const newUiState = { ...prevUiState };
          newUiState.hasUnreadMessages = true;
          return newUiState;
        });
      }
    } catch (err) {
      console.error('Error sending message:', err);
      setError(`Failed to send message: ${err.message}`);

      // Create error message with validation to prevent constructor errors
      const errorContent = `I'm having trouble connecting to the documentation system. ${err.message}`;
      const validatedErrorContent = errorContent.length > 2000
        ? errorContent.substring(0, 2000) + '...'
        : errorContent;

      const errorMessage = new ChatMessage(
        validatedErrorContent,
        'assistant'
      );
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      // Clear input and selected text regardless of success or error
      setInputValue('');
      setSelectedText(null);
      setIsLoading(false);
    }
  }, [inputValue, selectedText, addMessage, chatSession?.id, uiState.isMinimized]);

  const handleKeyDown = useCallback((e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage(e);
    }
  }, [handleSendMessage]);

  const handleMinimize = useCallback(() => {
    toggleMinimize();
  }, [toggleMinimize]);

  const handleInputChange = useCallback((e) => {
    setInputValue(e.target.value);
  }, []);

  // Function to handle selected text from the page
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection.toString().trim()) {
        const selectedTextContent = selection.toString().trim();
        if (selectedTextContent.length > 0 && selectedTextContent.length <= 5000) {
          setSelectedText(selectedTextContent);
        }
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  return (
    <div className="chatkit-interface" style={{
      display: 'flex',
      flexDirection: 'column',
      height: '100%',
      backgroundColor: 'white',
      border: '1px solid #ddd',
      borderRadius: '8px',
      overflow: 'hidden'
    }}>
      {/* Header */}
      <div
        className="chatkit-header"
        style={{
          backgroundColor: '#f5f5f5',
          padding: '12px',
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
          cursor: 'move'
        }}
        onMouseDown={(e) => {
          // Allow dragging functionality if needed
        }}
      >
        <h3 style={{ margin: 0, fontSize: '16px', color: '#333' }}>Documentation Assistant</h3>
        <button
          onClick={handleMinimize}
          style={{
            background: 'none',
            border: 'none',
            fontSize: '18px',
            cursor: 'pointer',
            color: '#666',
            padding: '4px'
          }}
        >
          −
        </button>
      </div>

      {/* Messages Container */}
      <div
        ref={messagesContainerRef}
        className="chatkit-messages"
        style={{
          flex: 1,
          overflowY: 'auto',
          padding: '16px',
          backgroundColor: '#fafafa'
        }}
        onScroll={handleScroll}
      >
        {error && (
          <div style={{
            backgroundColor: '#ffebee',
            color: '#c62828',
            padding: '8px',
            borderRadius: '4px',
            marginBottom: '8px'
          }}>
            {error}
          </div>
        )}

        {messages.map((msg, index) => (
          <div
            key={msg.id || index}
            style={{
              marginBottom: '12px',
              textAlign: msg.role === 'user' ? 'right' : 'left'
            }}
          >
            <div
              style={{
                display: 'inline-block',
                padding: '8px 12px',
                borderRadius: '18px',
                backgroundColor: msg.role === 'user' ? '#e3f2fd' : '#ffffff',
                border: msg.role === 'user' ? '1px solid #bbdefb' : '1px solid #e0e0e0',
                maxWidth: '80%',
                wordWrap: 'break-word'
              }}
            >
              {msg.content}
              {msg.selectedTextContext && (
                <div style={{
                  marginTop: '4px',
                  padding: '4px',
                  backgroundColor: '#fff3e0',
                  border: '1px solid #ffe0b2',
                  borderRadius: '4px',
                  fontSize: '12px',
                  fontStyle: 'italic'
                }}>
                  Selected: {msg.selectedTextContext}
                </div>
              )}
            </div>
            <div style={{
              fontSize: '10px',
              color: '#999',
              marginTop: '4px',
              textAlign: 'right'
            }}>
              {new Date(msg.timestamp).toLocaleTimeString()}
            </div>
          </div>
        ))}
        {isLoading && (
          <div style={{ textAlign: 'left', marginBottom: '12px' }}>
            <div style={{
              display: 'inline-block',
              padding: '8px 12px',
              borderRadius: '18px',
              backgroundColor: '#ffffff',
              border: '1px solid #e0e0e0'
            }}>
              <div>Typing...</div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {/* Input Area */}
      {selectedText && (
        <div style={{
          backgroundColor: '#fff3e0',
          padding: '8px',
          fontSize: '12px',
          color: '#5d4037',
          border: '1px solid #ffe0b2',
          borderRadius: '4px',
          margin: '8px',
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center'
        }}>
          <span>Using selected text: "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"</span>
          <button
            onClick={() => setSelectedText(null)}
            style={{
              background: 'none',
              border: 'none',
              color: '#5d4037',
              cursor: 'pointer',
              fontSize: '14px'
            }}
          >
            ×
          </button>
        </div>
      )}

      <form
        className="chatkit-input-form"
        onSubmit={handleSendMessage}
        style={{
          padding: '12px',
          backgroundColor: 'white',
          borderTop: '1px solid #eee'
        }}
      >
        <div style={{ display: 'flex' }}>
          <textarea
            ref={inputRef}
            value={inputValue}
            onChange={handleInputChange}
            onKeyDown={handleKeyDown}
            placeholder="Ask about the documentation..."
            style={{
              flex: 1,
              padding: '10px',
              border: '1px solid #ddd',
              borderRadius: '18px',
              resize: 'none',
              minHeight: '40px',
              maxHeight: '100px',
              fontSize: '14px'
            }}
            rows={1}
          />
          <button
            type="submit"
            disabled={isLoading || !inputValue.trim()}
            style={{
              marginLeft: '8px',
              padding: '10px 16px',
              backgroundColor: inputValue.trim() ? '#1a73e8' : '#ccc',
              color: 'white',
              border: 'none',
              borderRadius: '18px',
              cursor: inputValue.trim() ? 'pointer' : 'not-allowed'
            }}
          >
            {isLoading ? 'Sending...' : 'Send'}
          </button>
        </div>
      </form>
    </div>
  );
};