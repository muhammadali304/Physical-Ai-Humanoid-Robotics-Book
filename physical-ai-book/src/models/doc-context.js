/**
 * DocumentationContext model for tracking the documentation context
 */
export class DocumentationContext {
  constructor(pageUrl, pageTitle = '', selectedText = null, section = null) {
    if (!pageUrl || !this.isValidUrl(pageUrl)) {
      throw new Error('Page URL must be a valid URL');
    }

    if (pageTitle && (pageTitle.length < 1 || pageTitle.length > 200)) {
      throw new Error('Page title must be between 1 and 200 characters if provided');
    }

    if (selectedText && (selectedText.length < 1 || selectedText.length > 5000)) {
      throw new Error('Selected text must be between 1 and 5000 characters if provided');
    }

    this.pageUrl = pageUrl;
    this.pageTitle = pageTitle;
    this.selectedText = selectedText;
    this.section = section;
    this.timestamp = new Date().toISOString();
  }

  isValidUrl(string) {
    try {
      new URL(string);
      return true;
    } catch (_) {
      return false;
    }
  }

  updateSelectedText(text) {
    if (text && (text.length < 1 || text.length > 5000)) {
      throw new Error('Selected text must be between 1 and 5000 characters');
    }
    this.selectedText = text;
  }

  updateSection(section) {
    this.section = section;
  }

  toJSON() {
    return {
      pageUrl: this.pageUrl,
      pageTitle: this.pageTitle,
      selectedText: this.selectedText,
      section: this.section,
      timestamp: this.timestamp
    };
  }

  static fromJSON(json) {
    return new DocumentationContext(
      json.pageUrl,
      json.pageTitle,
      json.selectedText,
      json.section
    );
  }
}