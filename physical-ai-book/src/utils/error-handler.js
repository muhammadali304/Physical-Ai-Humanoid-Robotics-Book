/**
 * Error handling utilities for the ChatKit integration
 */
export class ErrorHandler {
  constructor() {
    this.errorLog = [];
  }

  // Log error with context
  logError(error, context = {}) {
    const errorEntry = {
      timestamp: new Date().toISOString(),
      error: error.message || error.toString(),
      stack: error.stack,
      context: context,
      url: typeof window !== 'undefined' ? window.location.href : 'server'
    };

    this.errorLog.push(errorEntry);
    console.error('ChatKit Error:', errorEntry);

    // Limit error log to prevent memory issues
    if (this.errorLog.length > 100) {
      this.errorLog = this.errorLog.slice(-50);
    }

    return errorEntry;
  }

  // Handle API errors
  handleApiError(error, context = {}) {
    const errorContext = {
      ...context,
      type: 'API_ERROR',
      severity: 'HIGH'
    };

    return this.logError(error, errorContext);
  }

  // Handle message sending errors
  handleMessageError(error, context = {}) {
    const errorContext = {
      ...context,
      type: 'MESSAGE_ERROR',
      severity: 'MEDIUM'
    };

    return this.logError(error, errorContext);
  }

  // Handle UI errors
  handleUIError(error, context = {}) {
    const errorContext = {
      ...context,
      type: 'UI_ERROR',
      severity: 'LOW'
    };

    return this.logError(error, errorContext);
  }

  // Format error for display to user
  formatUserFriendlyError(error, errorType = 'GENERAL') {
    const errorMessages = {
      'NETWORK_ERROR': 'Unable to connect to the documentation system. Please check your internet connection and try again.',
      'API_ERROR': 'The documentation system is temporarily unavailable. Please try again in a moment.',
      'MESSAGE_ERROR': 'There was an issue sending your message. Please try again.',
      'VALIDATION_ERROR': 'Your message did not meet the required format. Please check and try again.',
      'GENERAL': 'An unexpected error occurred. Please try again or refresh the page.'
    };

    // Check if it's a network error
    if (error.message && (error.message.includes('fetch') || error.message.includes('network'))) {
      return errorMessages['NETWORK_ERROR'];
    }

    // Check if it's an API error
    if (error.message && error.message.includes('API')) {
      return errorMessages['API_ERROR'];
    }

    return errorMessages[errorType] || errorMessages['GENERAL'];
  }

  // Get recent errors
  getRecentErrors(limit = 10) {
    return this.errorLog.slice(-limit);
  }

  // Clear error log
  clearErrorLog() {
    this.errorLog = [];
  }

  // Check if error is recoverable
  isRecoverable(error) {
    if (!error) return true;

    const nonRecoverableErrors = [
      'Invalid API key',
      'Authentication failed',
      'Forbidden',
      'Unauthorized'
    ];

    return !nonRecoverableErrors.some(msg =>
      error.message && error.message.includes(msg)
    );
  }

  // Handle and return user-friendly message
  handleAndReturnMessage(error, context = {}, defaultType = 'GENERAL') {
    // Log the error internally
    this.logError(error, context);

    // Return user-friendly message
    return this.formatUserFriendlyError(error, defaultType);
  }

  // Error boundary style error handling for React components
  static catchError(error, errorInfo) {
    console.error('Error caught by ChatKit error handler:', error, errorInfo);

    return {
      hasError: true,
      error: error,
      errorInfo: errorInfo
    };
  }
}