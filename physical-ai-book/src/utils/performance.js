/**
 * Performance monitoring utility for the ChatKit system
 */
export class PerformanceMonitor {
  constructor() {
    this.metrics = {
      chatResponseTimes: [],
      componentLoadTimes: [],
      apiErrorRates: [],
      concurrentSessions: 0
    };
    this.startTime = Date.now();
  }

  /**
   * Track chat response time
   * @param {number} responseTimeMs - Response time in milliseconds
   */
  trackChatResponseTime(responseTimeMs) {
    this.metrics.chatResponseTimes.push({
      timestamp: Date.now(),
      responseTime: responseTimeMs
    });

    // Keep only the last 1000 measurements to prevent memory issues
    if (this.metrics.chatResponseTimes.length > 1000) {
      this.metrics.chatResponseTimes = this.metrics.chatResponseTimes.slice(-1000);
    }
  }

  /**
   * Track component load time
   * @param {string} componentName - Name of the component
   * @param {number} loadTimeMs - Load time in milliseconds
   */
  trackComponentLoadTime(componentName, loadTimeMs) {
    if (!this.metrics.componentLoadTimes[componentName]) {
      this.metrics.componentLoadTimes[componentName] = [];
    }

    this.metrics.componentLoadTimes[componentName].push({
      timestamp: Date.now(),
      loadTime: loadTimeMs
    });

    // Keep only the last 500 measurements per component
    if (this.metrics.componentLoadTimes[componentName].length > 500) {
      this.metrics.componentLoadTimes[componentName] =
        this.metrics.componentLoadTimes[componentName].slice(-500);
    }
  }

  /**
   * Track API error rate
   * @param {boolean} isError - Whether the request resulted in an error
   */
  trackApiErrorRate(isError) {
    this.metrics.apiErrorRates.push({
      timestamp: Date.now(),
      isError: isError
    });

    // Keep only the last 1000 measurements
    if (this.metrics.apiErrorRates.length > 1000) {
      this.metrics.apiErrorRates = this.metrics.apiErrorRates.slice(-1000);
    }
  }

  /**
   * Track concurrent sessions
   * @param {number} sessionCount - Number of concurrent sessions
   */
  trackConcurrentSessions(sessionCount) {
    this.metrics.concurrentSessions = sessionCount;
  }

  /**
   * Get p95 percentile of chat response times
   * @returns {number} p95 response time in milliseconds
   */
  getChatResponseTimeP95() {
    if (this.metrics.chatResponseTimes.length === 0) {
      return 0;
    }

    const sorted = this.metrics.chatResponseTimes
      .map(m => m.responseTime)
      .sort((a, b) => a - b);

    const index = Math.floor(sorted.length * 0.95);
    return sorted[index] || 0;
  }

  /**
   * Get p99 percentile of chat response times
   * @returns {number} p99 response time in milliseconds
   */
  getChatResponseTimeP99() {
    if (this.metrics.chatResponseTimes.length === 0) {
      return 0;
    }

    const sorted = this.metrics.chatResponseTimes
      .map(m => m.responseTime)
      .sort((a, b) => a - b);

    const index = Math.floor(sorted.length * 0.99);
    return sorted[index] || 0;
  }

  /**
   * Get average component load time for a specific component
   * @param {string} componentName - Name of the component
   * @returns {number} Average load time in milliseconds
   */
  getAverageComponentLoadTime(componentName) {
    if (!this.metrics.componentLoadTimes[componentName] ||
        this.metrics.componentLoadTimes[componentName].length === 0) {
      return 0;
    }

    const total = this.metrics.componentLoadTimes[componentName]
      .reduce((sum, m) => sum + m.loadTime, 0);

    return total / this.metrics.componentLoadTimes[componentName].length;
  }

  /**
   * Get API error rate (percentage)
   * @param {number} minutes - Number of minutes to calculate for (default: 5)
   * @returns {number} Error rate as percentage
   */
  getApiErrorRate(minutes = 5) {
    const now = Date.now();
    const cutoff = now - (minutes * 60 * 1000);

    const relevantMetrics = this.metrics.apiErrorRates.filter(
      m => m.timestamp >= cutoff
    );

    if (relevantMetrics.length === 0) {
      return 0;
    }

    const errorCount = relevantMetrics.filter(m => m.isError).length;
    return (errorCount / relevantMetrics.length) * 100;
  }

  /**
   * Measure execution time of a function
   * @param {Function} fn - Function to measure
   * @param {string} name - Name for the measurement
   * @returns {*} Result of the function
   */
  async measureFunction(fn, name) {
    const startTime = performance.now();
    let result;
    let error = false;

    try {
      if (fn.constructor.name === 'AsyncFunction') {
        result = await fn();
      } else {
        result = fn();
      }
    } catch (e) {
      error = true;
      throw e;
    } finally {
      const endTime = performance.now();
      const duration = endTime - startTime;

      // Track the duration based on the name
      if (name.startsWith('chat:')) {
        this.trackChatResponseTime(duration);
      } else if (name.startsWith('component:')) {
        this.trackComponentLoadTime(name.replace('component:', ''), duration);
      }

      // Track error rate
      this.trackApiErrorRate(error);
    }

    return result;
  }

  /**
   * Measure API call performance
   * @param {Function} apiCall - API call function to measure
   * @param {string} endpoint - API endpoint name
   * @returns {*} Result of the API call
   */
  async measureApiCall(apiCall, endpoint) {
    const startTime = Date.now();
    let result;
    let error = false;

    try {
      result = await apiCall();
    } catch (e) {
      error = true;
      throw e;
    } finally {
      const duration = Date.now() - startTime;

      // Track response time for chat endpoints
      if (endpoint.includes('/query')) {
        this.trackChatResponseTime(duration);
      }

      // Track error rate
      this.trackApiErrorRate(error);
    }

    return result;
  }

  /**
   * Get all current metrics
   * @returns {Object} Current performance metrics
   */
  getMetrics() {
    return {
      ...this.metrics,
      chatResponseTimeP95: this.getChatResponseTimeP95(),
      chatResponseTimeP99: this.getChatResponseTimeP99(),
      apiErrorRate: this.getApiErrorRate(),
      concurrentSessions: this.metrics.concurrentSessions,
      uptime: Date.now() - this.startTime,
      timestamp: Date.now()
    };
  }

  /**
   * Reset metrics collection
   */
  reset() {
    this.metrics = {
      chatResponseTimes: [],
      componentLoadTimes: [],
      apiErrorRates: [],
      concurrentSessions: 0
    };
    this.startTime = Date.now();
  }

  /**
   * Generate a performance report
   * @returns {string} Performance report
   */
  generateReport() {
    const metrics = this.getMetrics();

    return `
Performance Report - ChatKit System
====================================

Response Times:
- P95: ${metrics.chatResponseTimeP95.toFixed(2)}ms
- P99: ${metrics.chatResponseTimeP99.toFixed(2)}ms

API Error Rate (5min): ${metrics.apiErrorRate.toFixed(2)}%

Concurrent Sessions: ${metrics.concurrentSessions}

Uptime: ${(metrics.uptime / 1000 / 60).toFixed(2)} minutes

Total Measurements:
- Chat responses: ${metrics.chatResponseTimes.length}
- API errors tracked: ${metrics.apiErrorRates.length}
    `;
  }

  /**
   * Export metrics in a format suitable for monitoring systems
   * @returns {Array} Array of metrics in key-value format
   */
  exportMetrics() {
    const metrics = this.getMetrics();
    const exported = [];

    exported.push({
      name: 'chat_response_time_p95',
      value: metrics.chatResponseTimeP95,
      unit: 'milliseconds',
      timestamp: metrics.timestamp
    });

    exported.push({
      name: 'chat_response_time_p99',
      value: metrics.chatResponseTimeP99,
      unit: 'milliseconds',
      timestamp: metrics.timestamp
    });

    exported.push({
      name: 'api_error_rate',
      value: metrics.apiErrorRate,
      unit: 'percentage',
      timestamp: metrics.timestamp
    });

    exported.push({
      name: 'concurrent_sessions',
      value: metrics.concurrentSessions,
      unit: 'count',
      timestamp: metrics.timestamp
    });

    exported.push({
      name: 'uptime',
      value: metrics.uptime,
      unit: 'milliseconds',
      timestamp: metrics.timestamp
    });

    return exported;
  }
}

// Create a singleton instance
export const performanceMonitor = new PerformanceMonitor();

// Export for use in other modules
export default performanceMonitor;