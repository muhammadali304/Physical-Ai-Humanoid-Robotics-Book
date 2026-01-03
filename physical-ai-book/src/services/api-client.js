/**
 * API client service for RAG backend communication
 */
export class ApiClient {
  constructor() {
    // Use the configured backend URL from environment variables
    const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';
    this.baseUrl = `${backendUrl}/api/v1`;
    this.apiKey = process.env.RAG_API_KEY || '';
    this.defaultHeaders = {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${this.apiKey}`,
      'X-Requested-With': 'XMLHttpRequest'
    };
  }

  async sendChatMessage(message, sessionId = null, context = null) {
    try {
      const requestBody = {
        query: message
      };

      // Only add session_id if it's provided and valid
      if (sessionId) {
        requestBody.session_id = sessionId;
      } else {
        // Generate a proper session ID if none provided
        requestBody.session_id = this.generateSessionId();
      }

      // Add context if provided
      if (context) {
        requestBody.context = context;
      }

      console.log('Sending request to:', `${this.baseUrl}/query`);
      console.log('Request body:', requestBody);
      console.log('Headers:', this.defaultHeaders);

      const response = await fetch(`${this.baseUrl}/query`, {
        method: 'POST',
        headers: this.defaultHeaders,
        body: JSON.stringify(requestBody)
      });

      console.log('Response status:', response.status);

      if (!response.ok) {
        const errorText = await response.text();
        console.error('API request failed:', response.status, response.statusText, errorText);
        throw new Error(`API request failed: ${response.status} ${response.statusText} - ${errorText}`);
      }

      const data = await response.json();
      return {
        sessionId: data.session_id,
        response: data.response,
        sources: data.sources || [],
        timestamp: new Date().toISOString()
      };
    } catch (error) {
      console.error('Error sending chat message:', error);
      if (error.message.includes('Failed to fetch')) {
        console.error('This may indicate that the backend server is not running or is unreachable.');
        console.error('Please ensure the backend server is running on:', this.baseUrl);
      }
      throw error;
    }
  }

  async getChatHistory(sessionId) {
    try {
      const response = await fetch(`${this.baseUrl}/query/history/${sessionId}`, {
        method: 'GET',
        headers: this.defaultHeaders
      });

      if (!response.ok) {
        throw new Error(`Failed to get chat history: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error getting chat history:', error);
      throw error;
    }
  }

  async healthCheck() {
    try {
      console.log('Performing health check to:', `${this.baseUrl}/query/health`);

      const response = await fetch(`${this.baseUrl}/query/health`, {
        method: 'GET',
        headers: this.defaultHeaders
      });

      console.log('Health check response status:', response.status);
      return response.ok;
    } catch (error) {
      console.error('Health check failed:', error);
      if (error.message.includes('Failed to fetch')) {
        console.error('Backend server may not be running or is unreachable.');
        console.error('Please ensure the backend server is running on:', this.baseUrl);
      }
      return false;
    }
  }

  async sendWithSelectedText(message, selectedText, sessionId = null) {
    try {
      // Validate message before sending (backend requires 1-2000 characters)
      if (!message || message.trim().length === 0) {
        throw new Error('Message cannot be empty');
      }

      if (message.trim().length > 2000) {
        throw new Error('Message exceeds 2000 character limit');
      }

      const requestBody = {
        query: message,
        selected_text: selectedText
      };

      // Only add session_id if it's provided and valid
      if (sessionId) {
        requestBody.session_id = sessionId;
      } else {
        // Generate a proper session ID if none provided
        requestBody.session_id = this.generateSessionId();
      }

      console.log('Sending request to:', `${this.baseUrl}/query`);
      console.log('Request body:', requestBody);
      console.log('Headers:', this.defaultHeaders);

      const response = await fetch(`${this.baseUrl}/query`, {
        method: 'POST',
        headers: this.defaultHeaders,
        body: JSON.stringify(requestBody)
      });

      console.log('Response status:', response.status);

      if (!response.ok) {
        const errorText = await response.text();
        console.error('API request failed:', response.status, response.statusText, errorText);
        throw new Error(`API request failed: ${response.status} ${response.statusText} - ${errorText}`);
      }

      const data = await response.json();
      return {
        sessionId: data.session_id,
        response: data.response,
        sources: data.sources || [],
        timestamp: new Date().toISOString()
      };
    } catch (error) {
      console.error('Error sending message with selected text:', error);
      if (error.message.includes('Failed to fetch')) {
        console.error('This may indicate that the backend server is not running or is unreachable.');
        console.error('Please ensure the backend server is running on:', this.baseUrl);
      }
      throw error;
    }
  }

  generateSessionId() {
    // Generate a proper UUID v4 format
    return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
      const r = Math.random() * 16 | 0;
      const v = c === 'x' ? r : (r & 0x3 | 0x8);
      return v.toString(16);
    });
  }

  // Method for graceful degradation when backend is unavailable
  async sendOfflineResponse(message) {
    // Fallback response when backend is unavailable
    return {
      sessionId: this.generateSessionId(),
      response: `I'm currently unable to connect to the documentation system. Please check your connection and try again later. In the meantime, you might want to check the documentation directly on the page.`,
      sources: [],
      timestamp: new Date().toISOString()
    };
  }

  // Method for rate limiting and request validation
  async validateAndSend(message, sessionId = null, context = null) {
    // Basic validation
    if (!message || message.trim().length === 0) {
      throw new Error('Message cannot be empty');
    }

    if (message.length > 2000) {
      throw new Error('Message exceeds 2000 character limit');
    }

    // Send the message
    return await this.sendChatMessage(message, sessionId, context);
  }
}