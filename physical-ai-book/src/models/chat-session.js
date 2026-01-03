/**
 * ChatSession model for managing conversation context
 */
export class ChatSession {
  constructor(id, initialPageContext = null) {
    this.id = id || this.generateId();
    this.messages = [];
    this.createdAt = new Date().toISOString();
    this.lastActiveAt = new Date().toISOString();
    this.pageContext = initialPageContext || null;
    this.uiState = {
      isMinimized: false,
      hasUnreadMessages: false
    };
  }

  generateId() {
    return 'chat-session-' + Date.now() + '-' + Math.random().toString(36).substr(2, 9);
  }

  addMessage(message) {
    // Validate message content length
    if (message.content && message.content.length > 2000) {
      throw new Error('Message content exceeds 2000 character limit');
    }

    this.messages.push(message);
    this.lastActiveAt = new Date().toISOString();

    // Limit message history to 100 messages
    if (this.messages.length > 100) {
      this.messages = this.messages.slice(-100);
    }
  }

  getMessages() {
    return this.messages;
  }

  updatePageContext(pageContext) {
    this.pageContext = pageContext;
    this.lastActiveAt = new Date().toISOString();
  }

  minimize() {
    this.uiState.isMinimized = true;
  }

  expand() {
    this.uiState.isMinimized = false;
    this.uiState.hasUnreadMessages = false;
  }

  markRead() {
    this.uiState.hasUnreadMessages = false;
  }

  updateUIState(newState) {
    this.uiState = { ...this.uiState, ...newState };
    this.lastActiveAt = new Date().toISOString();
  }

  toJSON() {
    return {
      id: this.id,
      messages: this.messages,
      createdAt: this.createdAt,
      lastActiveAt: this.lastActiveAt,
      pageContext: this.pageContext,
      uiState: this.uiState
    };
  }
}