/**
 * Utility for handling selected text detection using JavaScript Selection API
 */
export class SelectionHandler {
  constructor() {
    this.selectedText = null;
  }

  /**
   * Get the currently selected text on the page
   * @returns {string|null} The selected text or null if nothing is selected
   */
  getSelectedText() {
    const selection = window.getSelection();
    const text = selection.toString().trim();

    // Validate text length (between 1 and 5000 characters)
    if (text && text.length >= 1 && text.length <= 5000) {
      this.selectedText = text;
      return text;
    }

    return null;
  }

  /**
   * Get selected text with additional context
   * @returns {Object|null} Object containing selected text and context or null
   */
  getSelectedTextWithContext() {
    const selection = window.getSelection();
    const text = selection.toString().trim();

    if (text && text.length >= 1 && text.length <= 5000) {
      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();

      return {
        text: text,
        rect: {
          top: rect.top,
          left: rect.left,
          width: rect.width,
          height: rect.height
        },
        pageUrl: window.location.href,
        pageTitle: document.title,
        timestamp: new Date().toISOString()
      };
    }

    return null;
  }

  /**
   * Clear the current selection
   */
  clearSelection() {
    if (window.getSelection) {
      window.getSelection().removeAllRanges();
    } else if (document.selection) {
      document.selection.empty();
    }
    this.selectedText = null;
  }

  /**
   * Check if there is a valid selection
   * @returns {boolean} True if there is selected text within valid length
   */
  hasValidSelection() {
    const text = this.getSelectedText();
    return text !== null && text.length > 0;
  }

  /**
   * Listen for text selection events
   * @param {Function} callback - Callback function to execute when text is selected
   * @returns {Function} Cleanup function to remove the event listeners
   */
  listenForSelection(callback) {
    const handleSelection = () => {
      const selected = this.getSelectedTextWithContext();
      if (selected) {
        callback(selected);
      }
    };

    // Add event listeners
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    // Return cleanup function
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }

  /**
   * Highlight selected text temporarily
   * @param {string} text - Text to highlight
   */
  highlightSelectedText(text) {
    // This is a simplified version - in a real implementation you'd want to
    // highlight the actual selected DOM elements
    console.log(`Highlighting selected text: ${text.substring(0, 50)}${text.length > 50 ? '...' : ''}`);
  }

  /**
   * Get word count of selected text
   * @param {string} text - Text to count words for
   * @returns {number} Number of words in the text
   */
  getWordCount(text) {
    if (!text) return 0;
    return text.trim().split(/\s+/).filter(word => word.length > 0).length;
  }

  /**
   * Get character count of selected text
   * @param {string} text - Text to count characters for
   * @returns {number} Number of characters in the text
   */
  getCharacterCount(text) {
    if (!text) return 0;
    return text.length;
  }

  /**
   * Sanitize selected text (remove extra whitespace, etc.)
   * @param {string} text - Text to sanitize
   * @returns {string} Sanitized text
   */
  sanitizeText(text) {
    if (!text) return '';

    // Remove extra whitespace and normalize line breaks
    return text
      .replace(/\s+/g, ' ')  // Replace multiple spaces with single space
      .replace(/\n/g, ' ')   // Replace newlines with spaces
      .trim();               // Remove leading/trailing whitespace
  }
}

// Export a singleton instance
export const selectionHandler = new SelectionHandler();

// Also provide a convenience function
export const getSelectedText = () => {
  return selectionHandler.getSelectedText();
};

// Export for use in other modules
export default SelectionHandler;