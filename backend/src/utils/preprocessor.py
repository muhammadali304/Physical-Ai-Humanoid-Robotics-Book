"""
Content preprocessing utilities for the RAG Ingestion Pipeline.
Provides functions to clean and preprocess content before chunking.
"""

import re
from typing import List, Optional, Tuple
from bs4 import BeautifulSoup


class ContentPreprocessor:
    """
    Utility class for preprocessing content before chunking.
    Provides methods to clean, normalize, and prepare content for semantic chunking.
    """

    def __init__(self):
        self.clean_patterns = [
            # Remove extra whitespace and normalize
            (r'\s+', ' '),
            # Remove leading/trailing whitespace
            (r'^\s+|\s+$', ''),
            # Remove multiple consecutive newlines
            (r'\n\s*\n\s*\n+', '\n\n'),
        ]

        # Patterns to identify and preserve important structural elements
        self.structural_patterns = [
            # Headings (Markdown style)
            (r'^(#{1,6})\s+(.*)', r'\1 \2'),
            # List items
            (r'^(\s*[-*+]\s+)', r'\1'),
            # Numbered lists
            (r'^(\s*\d+\.\s+)', r'\1'),
        ]

    def preprocess_content(self, content: str) -> str:
        """
        Preprocess content by cleaning and normalizing it.

        Args:
            content: Raw content to preprocess

        Returns:
            Cleaned and normalized content
        """
        if not content:
            return ""

        # Apply cleaning patterns
        cleaned_content = content
        for pattern, replacement in self.clean_patterns:
            cleaned_content = re.sub(pattern, replacement, cleaned_content)

        # Normalize line endings
        cleaned_content = cleaned_content.replace('\r\n', '\n').replace('\r', '\n')

        # Remove HTML tags if present (but preserve content)
        cleaned_content = self.remove_html_tags(cleaned_content)

        # Normalize whitespace again after HTML removal
        for pattern, replacement in self.clean_patterns[:2]:  # Apply first two patterns again
            cleaned_content = re.sub(pattern, replacement, cleaned_content)

        return cleaned_content.strip()

    def remove_html_tags(self, content: str) -> str:
        """
        Remove HTML tags from content while preserving text content.

        Args:
            content: Content that may contain HTML tags

        Returns:
            Content with HTML tags removed
        """
        try:
            # Use BeautifulSoup to parse and extract text content
            soup = BeautifulSoup(content, 'html.parser')
            return soup.get_text(separator=' ', strip=True)
        except Exception:
            # If BeautifulSoup fails, use regex as fallback
            # Remove HTML tags but keep the content inside
            clean_content = re.sub(r'<[^>]+>', ' ', content)
            return clean_content

    def extract_headings_and_structure(self, content: str) -> List[Tuple[str, str, int]]:
        """
        Extract headings and their positions from content to preserve structure.

        Args:
            content: Content to analyze for headings

        Returns:
            List of tuples (heading_text, heading_level, position_in_content)
        """
        headings = []

        # Split content into lines to process
        lines = content.split('\n')

        for i, line in enumerate(lines):
            # Check for markdown headings
            heading_match = re.match(r'^(#{1,6})\s+(.+)', line.strip())
            if heading_match:
                hashes, heading_text = heading_match.groups()
                level = len(hashes)
                position = sum(len(l) + 1 for l in lines[:i])  # Calculate character position
                headings.append((heading_text.strip(), str(level), position))

            # Check for HTML headings
            html_heading_match = re.search(r'<h([1-6])[^>]*>(.*?)</h\1>', line, re.IGNORECASE)
            if html_heading_match:
                level, heading_text = html_heading_match.groups()
                position = sum(len(l) + 1 for l in lines[:i]) + line.find(heading_text)
                headings.append((BeautifulSoup(heading_text, 'html.parser').get_text().strip(), level, position))

        return headings

    def preserve_section_context(self, content: str, section_start: int, section_end: int) -> str:
        """
        Preserve context around a section by including relevant headings.

        Args:
            content: Full content
            section_start: Start position of the section
            section_end: End position of the section

        Returns:
            Section content with relevant headings included
        """
        headings = self.extract_headings_and_structure(content)

        # Find relevant headings before this section
        relevant_headings = []
        for heading_text, level, position in headings:
            if position < section_start:
                # Include headings that are not too far from the section
                if section_start - position <= 1000:  # Within 1000 characters
                    relevant_headings.append((heading_text, level, position))

        # Sort by position (most recent first)
        relevant_headings.sort(key=lambda x: x[2], reverse=True)

        # Add the most relevant heading to the section
        section_content = content[section_start:section_end]

        # Include the closest heading as context
        if relevant_headings:
            closest_heading = relevant_headings[0]
            heading_text, level, position = closest_heading
            section_content = f"H{level}: {heading_text}\n\n{section_content}"

        return section_content

    def normalize_content(self, content: str) -> str:
        """
        Normalize content by standardizing formatting and structure.

        Args:
            content: Content to normalize

        Returns:
            Normalized content
        """
        if not content:
            return ""

        # Normalize different types of quotes
        content = re.sub(r'[`\'"]+', '"', content)  # Standardize quotes

        # Normalize different types of dashes
        content = re.sub(r'[—–-]+', '-', content)  # Standardize dashes

        # Ensure consistent spacing around punctuation
        content = re.sub(r'\s*([,.!?;:])\s*', r'\1 ', content)
        content = re.sub(r'\s+', ' ', content)  # Normalize internal whitespace

        # Ensure proper sentence spacing
        content = re.sub(r'([.!?])\s*([A-Z])', r'\1 \2', content)

        return content.strip()

    def clean_special_characters(self, content: str) -> str:
        """
        Clean special characters that might interfere with tokenization.

        Args:
            content: Content to clean

        Returns:
            Content with special characters cleaned
        """
        if not content:
            return ""

        # Remove control characters
        content = re.sub(r'[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]', ' ', content)

        # Replace special characters that might cause issues
        content = re.sub(r'[^\x00-\x7F]+', ' ', content)  # Remove non-ASCII characters

        # Clean up multiple consecutive special characters
        content = re.sub(r'[!@#$%^&*()_+=\[\]{}|\\:";\'<>?,./]{3,}', ' ', content)

        return content

    def preprocess_for_chunking(self, content: str) -> str:
        """
        Complete preprocessing pipeline for content before chunking.

        Args:
            content: Raw content to preprocess

        Returns:
            Fully preprocessed content ready for chunking
        """
        if not content:
            return ""

        # Apply preprocessing steps in order
        content = self.remove_html_tags(content)
        content = self.clean_special_characters(content)
        content = self.preprocess_content(content)
        content = self.normalize_content(content)

        return content.strip()


def create_default_preprocessor() -> ContentPreprocessor:
    """Create a default content preprocessor instance."""
    return ContentPreprocessor()


# Convenience functions
def preprocess_content(content: str) -> str:
    """Convenience function to preprocess content."""
    preprocessor = create_default_preprocessor()
    return preprocessor.preprocess_for_chunking(content)


def extract_headings(content: str) -> List[Tuple[str, str, int]]:
    """Convenience function to extract headings from content."""
    preprocessor = create_default_preprocessor()
    return preprocessor.extract_headings_and_structure(content)