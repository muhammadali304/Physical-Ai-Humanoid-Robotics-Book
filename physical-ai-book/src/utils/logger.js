/**
 * Comprehensive logging utility for the ChatKit system
 */
export class Logger {
  constructor(level = 'info') {
    this.level = level;
    this.levelMap = {
      'debug': 0,
      'info': 1,
      'warn': 2,
      'error': 3
    };
    this.minLevel = this.levelMap[level] || 1;
  }

  /**
   * Check if a log level should be output based on current minimum level
   * @param {string} level - The log level to check
   * @returns {boolean} True if the level should be logged
   */
  shouldLog(level) {
    return (this.levelMap[level] || 1) >= this.minLevel;
  }

  /**
   * Log a debug message
   * @param {string} message - The message to log
   * @param {Object} meta - Additional metadata to include
   */
  debug(message, meta = {}) {
    if (this.shouldLog('debug')) {
      this._log('DEBUG', message, meta);
    }
  }

  /**
   * Log an info message
   * @param {string} message - The message to log
   * @param {Object} meta - Additional metadata to include
   */
  info(message, meta = {}) {
    if (this.shouldLog('info')) {
      this._log('INFO', message, meta);
    }
  }

  /**
   * Log a warning message
   * @param {string} message - The message to log
   * @param {Object} meta - Additional metadata to include
   */
  warn(message, meta = {}) {
    if (this.shouldLog('warn')) {
      this._log('WARN', message, meta);
    }
  }

  /**
   * Log an error message
   * @param {string} message - The message to log
   * @param {Object} meta - Additional metadata to include
   */
  error(message, meta = {}) {
    if (this.shouldLog('error')) {
      this._log('ERROR', message, meta);
    }
  }

  /**
   * Internal logging function that formats and outputs the log
   * @private
   * @param {string} level - The log level
   * @param {string} message - The message to log
   * @param {Object} meta - Additional metadata to include
   */
  _log(level, message, meta = {}) {
    const timestamp = new Date().toISOString();
    const logEntry = {
      timestamp,
      level,
      message,
      ...meta
    };

    // Output to console
    const logFn = level.toLowerCase() === 'error' ? console.error :
                  level.toLowerCase() === 'warn' ? console.warn : console.log;

    logFn(`[${timestamp}] ${level}:`, message, meta);

    // In a production environment, you might also want to send logs to a service
    // For now, we'll just log to the console
    this._sendToLoggingService(logEntry);
  }

  /**
   * Send log entry to external logging service (stub implementation)
   * @private
   * @param {Object} logEntry - The log entry to send
   */
  _sendToLoggingService(logEntry) {
    // In a real implementation, this would send logs to an external service
    // like Sentry, LogRocket, or a custom logging service
    // For now, we just have the stub
  }

  /**
   * Create a child logger with additional context
   * @param {Object} context - Additional context to include in logs
   * @returns {Logger} A new logger instance with the context
   */
  child(context) {
    const childLogger = new Logger(this.level);
    childLogger.context = { ...this.context, ...context };
    return childLogger;
  }

  /**
   * Log an API request
   * @param {string} method - HTTP method
   * @param {string} url - Request URL
   * @param {Object} params - Request parameters
   * @param {Object} response - Response object
   */
  logApiRequest(method, url, params = {}, response = {}) {
    this.info(`API Request: ${method} ${url}`, {
      method,
      url,
      params,
      statusCode: response.status,
      duration: response.duration,
      success: response.success
    });
  }

  /**
   * Log a chat message event
   * @param {string} eventType - Type of chat event (send, receive, error)
   * @param {Object} message - The message object
   * @param {Object} context - Additional context
   */
  logChatEvent(eventType, message, context = {}) {
    this.info(`Chat Event: ${eventType}`, {
      eventType,
      messageId: message.id,
      messageRole: message.role,
      messageContentLength: message.content?.length,
      timestamp: message.timestamp,
      ...context
    });
  }

  /**
   * Log a performance metric
   * @param {string} metricName - Name of the metric
   * @param {number} value - Value of the metric
   * @param {string} unit - Unit of measurement
   * @param {Object} context - Additional context
   */
  logPerformance(metricName, value, unit, context = {}) {
    this.info(`Performance Metric: ${metricName}`, {
      metricName,
      value,
      unit,
      ...context
    });
  }

  /**
   * Set the logging level
   * @param {string} level - The new log level
   */
  setLevel(level) {
    this.level = level;
    this.minLevel = this.levelMap[level] || 1;
  }
}

// Create a singleton instance
export const logger = new Logger();

// Export for use in other modules
export default logger;