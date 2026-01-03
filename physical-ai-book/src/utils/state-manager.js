/**
 * State management utilities for the ChatKit integration
 */
export class StateManager {
  constructor() {
    this.storageKey = 'chatkit-state';
    this.sessionStorageKey = 'chatkit-session';
  }

  // Save state to localStorage
  saveState(state) {
    try {
      // Check if localStorage is available (not available during SSR)
      if (typeof localStorage === 'undefined') {
        return false;
      }
      localStorage.setItem(this.storageKey, JSON.stringify(state));
      return true;
    } catch (error) {
      console.error('Error saving state to localStorage:', error);
      return false;
    }
  }

  // Load state from localStorage
  loadState() {
    try {
      // Check if localStorage is available (not available during SSR)
      if (typeof localStorage === 'undefined') {
        return null;
      }
      const state = localStorage.getItem(this.storageKey);
      return state ? JSON.parse(state) : null;
    } catch (error) {
      console.error('Error loading state from localStorage:', error);
      return null;
    }
  }

  // Save session-specific data
  saveSessionData(sessionId, data) {
    try {
      // Check if localStorage is available (not available during SSR)
      if (typeof localStorage === 'undefined') {
        return false;
      }
      const key = `${this.sessionStorageKey}-${sessionId}`;
      localStorage.setItem(key, JSON.stringify(data));
      return true;
    } catch (error) {
      console.error('Error saving session data:', error);
      return false;
    }
  }

  // Load session-specific data
  loadSessionData(sessionId) {
    try {
      // Check if localStorage is available (not available during SSR)
      if (typeof localStorage === 'undefined') {
        return null;
      }
      const key = `${this.sessionStorageKey}-${sessionId}`;
      const data = localStorage.getItem(key);
      return data ? JSON.parse(data) : null;
    } catch (error) {
      console.error('Error loading session data:', error);
      return null;
    }
  }

  // Clear session data
  clearSessionData(sessionId) {
    try {
      // Check if localStorage is available (not available during SSR)
      if (typeof localStorage === 'undefined') {
        return false;
      }
      const key = `${this.sessionStorageKey}-${sessionId}`;
      localStorage.removeItem(key);
      return true;
    } catch (error) {
      console.error('Error clearing session data:', error);
      return false;
    }
  }

  // Save UI state
  saveUIState(uiState) {
    try {
      // Check if localStorage is available (not available during SSR)
      if (typeof localStorage === 'undefined') {
        return false;
      }
      localStorage.setItem('chatkit-ui-state', JSON.stringify(uiState));
      return true;
    } catch (error) {
      console.error('Error saving UI state:', error);
      return false;
    }
  }

  // Load UI state
  loadUIState() {
    try {
      // Check if localStorage is available (not available during SSR)
      if (typeof localStorage === 'undefined') {
        return null;
      }
      const state = localStorage.getItem('chatkit-ui-state');
      return state ? JSON.parse(state) : null;
    } catch (error) {
      console.error('Error loading UI state:', error);
      return null;
    }
  }

  // Save chat session
  saveChatSession(session) {
    try {
      // Check if localStorage is available (not available during SSR)
      if (typeof localStorage === 'undefined') {
        return false;
      }
      const sessionKey = `chatkit-session-${session.id}`;
      localStorage.setItem(sessionKey, JSON.stringify(session.toJSON()));
      return true;
    } catch (error) {
      console.error('Error saving chat session:', error);
      return false;
    }
  }

  // Load chat session
  loadChatSession(sessionId) {
    try {
      // Check if localStorage is available (not available during SSR)
      if (typeof localStorage === 'undefined') {
        return null;
      }
      const sessionKey = `chatkit-session-${sessionId}`;
      const sessionData = localStorage.getItem(sessionKey);
      return sessionData ? JSON.parse(sessionData) : null;
    } catch (error) {
      console.error('Error loading chat session:', error);
      return null;
    }
  }

  // Get all saved sessions
  getAllSessions() {
    try {
      // Check if localStorage is available (not available during SSR)
      if (typeof localStorage === 'undefined') {
        return [];
      }
      const sessions = [];
      for (let i = 0; i < localStorage.length; i++) {
        const key = localStorage.key(i);
        if (key && key.startsWith('chatkit-session-')) {
          const sessionData = localStorage.getItem(key);
          if (sessionData) {
            sessions.push(JSON.parse(sessionData));
          }
        }
      }
      return sessions;
    } catch (error) {
      console.error('Error getting all sessions:', error);
      return [];
    }
  }

  // Clear all chat data
  clearAllChatData() {
    try {
      // Check if localStorage is available (not available during SSR)
      if (typeof localStorage === 'undefined') {
        return false;
      }
      for (let i = 0; i < localStorage.length; i++) {
        const key = localStorage.key(i);
        if (key && (key.startsWith('chatkit-session-') || key.startsWith('chatkit-'))) {
          localStorage.removeItem(key);
          i--; // Adjust index after removal
        }
      }
      return true;
    } catch (error) {
      console.error('Error clearing all chat data:', error);
      return false;
    }
  }

  // Update session last active time
  updateSessionLastActive(sessionId) {
    try {
      const session = this.loadChatSession(sessionId);
      if (session) {
        session.lastActiveAt = new Date().toISOString();
        this.saveChatSession(session);
      }
    } catch (error) {
      console.error('Error updating session last active time:', error);
    }
  }
}