/**
 * ChatMessage model for individual messages in a conversation
 */
export class ChatMessage {
  constructor(content, role, selectedTextContext = null) {
    if (!content || content.length < 1 || content.length > 2000) {
      throw new Error('Message content must be between 1 and 2000 characters');
    }

    if (!role || !['user', 'assistant'].includes(role)) {
      throw new Error('Message role must be either "user" or "assistant"');
    }

    this.id = this.generateId();
    this.content = content;
    this.role = role;
    this.timestamp = new Date().toISOString();
    this.selectedTextContext = selectedTextContext || null;
    this.status = 'sent'; // Default status
  }

  generateId() {
    return 'msg-' + Date.now() + '-' + Math.random().toString(36).substr(2, 9);
  }

  markSending() {
    this.status = 'sending';
  }

  markSent() {
    this.status = 'sent';
  }

  markError() {
    this.status = 'error';
  }

  updateContent(newContent) {
    if (newContent.length < 1 || newContent.length > 2000) {
      throw new Error('Message content must be between 1 and 2000 characters');
    }
    this.content = newContent;
  }

  toJSON() {
    return {
      id: this.id,
      content: this.content,
      role: this.role,
      timestamp: this.timestamp,
      selectedTextContext: this.selectedTextContext,
      status: this.status
    };
  }

  static fromJSON(json) {
    const message = new ChatMessage(json.content, json.role, json.selectedTextContext);
    message.id = json.id;
    message.timestamp = json.timestamp;
    message.status = json.status || 'sent';
    return message;
  }
}