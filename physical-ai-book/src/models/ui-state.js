/**
 * UIState model for tracking the UI state of the chat component
 */
export class UIState {
  constructor(isMinimized = false, hasUnreadMessages = false) {
    this.isMinimized = isMinimized;
    this.hasUnreadMessages = hasUnreadMessages;
    this.position = { x: 20, y: 20 }; // Default position (right bottom)
    this.size = { width: 400, height: 500 }; // Default size
    this.timestamp = new Date().toISOString();
  }

  minimize() {
    this.isMinimized = true;
  }

  expand() {
    this.isMinimized = false;
    this.hasUnreadMessages = false;
  }

  markUnread() {
    this.hasUnreadMessages = true;
  }

  markRead() {
    this.hasUnreadMessages = false;
  }

  updatePosition(x, y) {
    this.position = { x, y };
  }

  updateSize(width, height) {
    this.size = { width, height };
  }

  toggleMinimize() {
    this.isMinimized = !this.isMinimized;
    if (!this.isMinimized) {
      this.hasUnreadMessages = false;
    }
  }

  toJSON() {
    return {
      isMinimized: this.isMinimized,
      hasUnreadMessages: this.hasUnreadMessages,
      position: this.position,
      size: this.size,
      timestamp: this.timestamp
    };
  }

  static fromJSON(json) {
    const state = new UIState(json.isMinimized, json.hasUnreadMessages);
    state.position = json.position || { x: 20, y: 20 };
    state.size = json.size || { width: 400, height: 500 };
    state.timestamp = json.timestamp || new Date().toISOString();
    return state;
  }
}