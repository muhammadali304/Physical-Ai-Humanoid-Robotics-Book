/**
 * ChatKitWrapper component that will be integrated at the layout level
 */
import React, { useState, useEffect, useRef, useCallback } from 'react';
import { ChatInterface } from './ChatInterface';
import { SelectedTextHandler } from './SelectedTextHandler';
import { ChatSession } from '../../models/chat-session';
import { UIState } from '../../models/ui-state';
import { StateManager } from '../../utils/state-manager';
import { ErrorHandler } from '../../utils/error-handler';

export const ChatKitWrapper = () => {
  const [initializationComplete, setInitializationComplete] = useState(false);
  const [chatSession, setChatSession] = useState(null);
  const [uiState, setUiState] = useState(() => {
    // Initialize UI state from saved state or default to minimized
    const stateManager = new StateManager();
    const savedUIState = stateManager.loadUIState();

    // Check if this is the first time loading (no saved state)
    const isFirstLoad = !savedUIState;

    if (savedUIState) {
      // If we have saved state, use it but ensure it's properly formatted
      return new UIState(savedUIState.isMinimized, savedUIState.hasUnreadMessages);
    }

    // Default to minimized on first load
    const defaultState = new UIState(true, false); // true means minimized by default
    stateManager.saveUIState(defaultState.toJSON());
    return defaultState;
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  // Initialize state manager and error handler only once using useRef
  const stateManagerRef = useRef(new StateManager());
  const errorHandlerRef = useRef(new ErrorHandler());

  // Initialize chat session
  useEffect(() => {
    const initializeSession = async () => {
      const stateManager = stateManagerRef.current;

      // Create or load session
      const sessionId = localStorage.getItem('chatkit-current-session');
      if (sessionId) {
        const sessionData = stateManager.loadChatSession(sessionId);
        if (sessionData) {
          setChatSession(new ChatSession(sessionData.id, sessionData.pageContext));
          setInitializationComplete(true);
          return;
        }
      }

      // Create a new session if none exists
      const newSession = new ChatSession();
      setChatSession(newSession);
      localStorage.setItem('chatkit-current-session', newSession.id);
      stateManager.saveChatSession(newSession);
      setInitializationComplete(true);
    };

    // Only initialize once
    if (!initializationComplete && !chatSession) {
      initializeSession();
    }
  }, [initializationComplete]); // Only run if not initialized - removed chatSession to prevent loop

  // Save state when it changes
  useEffect(() => {
    if (chatSession) {
      const stateManager = stateManagerRef.current;
      stateManager.saveChatSession(chatSession);
    }
  }, [chatSession]);

  useEffect(() => {
    const stateManager = stateManagerRef.current;
    stateManager.saveUIState(uiState.toJSON());
  }, [uiState]);

  const toggleMinimize = useCallback(() => {
    setUiState(prevUiState => {
      if (!prevUiState) return prevUiState;

      // Create a completely new UI state from existing properties to ensure deep copy
      const newState = new UIState(prevUiState.isMinimized, prevUiState.hasUnreadMessages);
      newState.position = { ...prevUiState.position };
      newState.size = { ...prevUiState.size };
      newState.timestamp = prevUiState.timestamp;

      newState.toggleMinimize();
      return newState;
    });
  }, []);

  const addMessage = useCallback((message) => {
    setChatSession(prevSession => {
      if (!prevSession) return prevSession;

      // Create a completely new session from JSON to ensure deep copy
      const sessionData = prevSession.toJSON();
      const newSession = new ChatSession(sessionData.id, sessionData.pageContext);
      newSession.messages = [...sessionData.messages]; // Deep copy messages array
      newSession.createdAt = sessionData.createdAt;
      newSession.lastActiveAt = sessionData.lastActiveAt;
      newSession.uiState = { ...sessionData.uiState }; // Deep copy UI state

      newSession.addMessage(message);
      return newSession;
    });
  }, []);

  const updatePageContext = useCallback((context) => {
    setChatSession(prevSession => {
      if (!prevSession) return prevSession;

      // Create a completely new session from JSON to ensure deep copy
      const sessionData = prevSession.toJSON();
      const newSession = new ChatSession(sessionData.id, context);
      newSession.messages = [...sessionData.messages]; // Deep copy messages array
      newSession.createdAt = sessionData.createdAt;
      newSession.lastActiveAt = sessionData.lastActiveAt;
      newSession.uiState = { ...sessionData.uiState }; // Deep copy UI state

      newSession.updatePageContext(context);
      return newSession;
    });
  }, []);

  const markRead = useCallback(() => {
    setUiState(prevUiState => {
      if (!prevUiState) return prevUiState;

      // Create a completely new UI state from JSON to ensure deep copy
      const newState = new UIState(prevUiState.isMinimized, prevUiState.hasUnreadMessages);
      newState.position = { ...prevUiState.position };
      newState.size = { ...prevUiState.size };
      newState.timestamp = prevUiState.timestamp;

      newState.markRead();
      return newState;
    });
  }, []);

  // Update page context when URL changes
  useEffect(() => {
    if (!initializationComplete) return; // Only run after initialization is complete

    const updateContext = () => {
      if (chatSession) {
        const context = {
          pageUrl: window.location.href,
          pageTitle: document.title,
          section: window.location.hash.substring(1) || null
        };
        updatePageContext(context);
      }
    };

    // Initial update
    updateContext();

    // Update on navigation
    const handleRouteChange = () => {
      updateContext();
    };

    // For SPAs, listen to popstate or custom events
    window.addEventListener('popstate', handleRouteChange);
    window.addEventListener('locationchange', handleRouteChange);

    return () => {
      window.removeEventListener('popstate', handleRouteChange);
      window.removeEventListener('locationchange', handleRouteChange);
    };
  }, [chatSession, initializationComplete, updatePageContext]); // Include updatePageContext in dependencies

  // Don't render until initialization is complete
  if (!initializationComplete || !chatSession) {
    return null; // Don't render anything until initialization is complete
  }

  if (uiState.isMinimized) {
    return (
      <div
        className="chatkit-minimized-indicator"
        onClick={(e) => {
          e.stopPropagation(); // Prevent event bubbling
          toggleMinimize();
          markRead();
        }}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          backgroundColor: '#1a73e8',
          color: 'white',
          borderRadius: '50%',
          width: '60px',
          height: '60px',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          cursor: 'pointer',
          zIndex: 1000,
          boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
          fontSize: '24px'
        }}
      >
        💬
        {uiState.hasUnreadMessages && (
          <span
            style={{
              position: 'absolute',
              top: '-5px',
              right: '-5px',
              backgroundColor: '#ff4d4d',
              color: 'white',
              borderRadius: '50%',
              width: '20px',
              height: '20px',
              fontSize: '12px',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center'
            }}
          >
            !
          </span>
        )}
      </div>
    );
  }

  return (
    <div
      className="chatkit-wrapper"
      style={{
        position: 'fixed',
        bottom: uiState.position.y,
        right: uiState.position.x,
        width: uiState.size.width,
        height: uiState.size.height,
        zIndex: 1000,
        boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
        borderRadius: '8px',
        overflow: 'hidden'
      }}
    >
      <ChatInterface
        chatSession={chatSession}
        addMessage={addMessage}
        toggleMinimize={toggleMinimize}
        isLoading={isLoading}
        setIsLoading={setIsLoading}
        error={error}
        setError={setError}
        uiState={uiState}
        setUiState={setUiState}
      />
      <SelectedTextHandler />
    </div>
  );
};