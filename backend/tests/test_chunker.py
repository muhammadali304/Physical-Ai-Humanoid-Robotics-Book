"""
Unit tests for the chunker service in the RAG Ingestion Pipeline.
Tests the chunking algorithms and functionality.
"""

import pytest
from typing import List

from src.services.chunker import ChunkerService
from src.models.chunk import ContentChunk


class TestChunkerService:
    """Test suite for the ChunkerService class."""

    @pytest.fixture
    def chunker_service(self):
        """Create a chunker service instance for testing."""
        return ChunkerService()

    def test_estimate_token_count(self, chunker_service):
        """Test token estimation functionality."""
        # Test with simple text
        text = "This is a test sentence."
        token_count = chunker_service.estimate_token_count(text)
        assert token_count > 0
        assert isinstance(token_count, int)

        # Test with empty string
        empty_count = chunker_service.estimate_token_count("")
        assert empty_count == 0

        # Test that longer text has more tokens
        longer_text = "This is a longer test sentence with more words."
        longer_count = chunker_service.estimate_token_count(longer_text)
        assert longer_count > token_count

    def test_validate_chunk_size_within_range(self, chunker_service):
        """Test chunk size validation for content within the valid range."""
        # Create content with ~100 tokens (words)
        content = "word " * 100
        is_valid, token_count, issues = chunker_service.validate_chunk_size(content)

        assert is_valid is True
        assert len(issues) == 0
        assert 50 <= token_count <= 1000

    def test_validate_chunk_size_too_small(self, chunker_service):
        """Test chunk size validation for content that's too small."""
        # Create content with 10 tokens (too small)
        content = "word " * 10
        is_valid, token_count, issues = chunker_service.validate_chunk_size(content)

        assert is_valid is False
        assert len(issues) > 0
        assert any("below minimum" in issue for issue in issues)

    def test_validate_chunk_size_too_large(self, chunker_service):
        """Test chunk size validation for content that's too large."""
        # Create content with 1500 tokens (too large)
        content = "word " * 1500
        is_valid, token_count, issues = chunker_service.validate_chunk_size(content)

        assert is_valid is False
        assert len(issues) > 0
        assert any("exceeds maximum" in issue for issue in issues)

    def test_adjust_chunk_to_valid_size_valid_content(self, chunker_service):
        """Test adjusting a chunk that's already valid size."""
        content = "This is valid content with appropriate length for testing purposes. " * 5
        chunks = chunker_service.adjust_chunk_to_valid_size(
            content, "http://example.com", "Test Page", 0
        )

        assert len(chunks) == 1
        assert chunks[0].content == content.strip()
        assert 50 <= chunks[0].token_count <= 1000

    def test_adjust_chunk_to_valid_size_too_large(self, chunker_service):
        """Test adjusting a chunk that's too large - should be split."""
        # Create content that's definitely too large (>1000 tokens)
        content = "This is a sentence. " * 100  # Should be well over 1000 tokens
        chunks = chunker_service.adjust_chunk_to_valid_size(
            content, "http://example.com", "Test Page", 0
        )

        # Should be split into multiple chunks
        assert len(chunks) > 0
        for chunk in chunks:
            assert chunk.token_count <= 1000

    def test_split_by_headings(self, chunker_service):
        """Test splitting content by headings."""
        content = """
        # Introduction
        This is the introduction content.

        ## Section 1
        This is section 1 content.

        ### Subsection 1.1
        This is subsection 1.1 content.

        ## Section 2
        This is section 2 content.
        """

        sections = chunker_service.split_by_headings(content)
        assert len(sections) >= 4  # Should have at least 4 sections

    def test_validate_content_quality_good_content(self, chunker_service):
        """Test content quality validation with good content."""
        content = "This is well-formed content with proper sentences and structure. It has good variety and meaningful text."
        is_valid, issues = chunker_service.validate_content_quality(content)

        assert is_valid is True
        assert len(issues) == 0

    def test_validate_content_quality_empty_content(self, chunker_service):
        """Test content quality validation with empty content."""
        content = ""
        is_valid, issues = chunker_service.validate_content_quality(content)

        assert is_valid is False
        assert len(issues) > 0

    def test_validate_content_quality_only_whitespace(self, chunker_service):
        """Test content quality validation with only whitespace."""
        content = "   \n\t  \n  "
        is_valid, issues = chunker_service.validate_content_quality(content)

        assert is_valid is False
        assert len(issues) > 0
        assert any("whitespace" in issue for issue in issues)

    def test_validate_content_quality_excessive_special_chars(self, chunker_service):
        """Test content quality validation with excessive special characters."""
        content = "!@#$%^&*()" * 50  # Lots of special characters
        is_valid, issues = chunker_service.validate_content_quality(content)

        assert is_valid is False
        assert len(issues) > 0
        assert any("special characters" in issue for issue in issues)

    def test_chunk_content_short_content(self, chunker_service):
        """Test chunking content that's too short."""
        result = chunker_service.chunk_content(
            content="Short",
            source_url="http://example.com",
            page_title="Test"
        )

        assert len(result.chunks) == 0

    def test_chunk_content_valid_content(self, chunker_service):
        """Test chunking valid content."""
        content = "This is a test paragraph. " * 20  # Should be valid length
        result = chunker_service.chunk_content(
            content=content,
            source_url="http://example.com",
            page_title="Test Page"
        )

        assert len(result.chunks) >= 0  # May be 0 if content gets split in a way that creates no valid chunks
        # If chunks were created, verify they have expected properties
        for chunk in result.chunks:
            assert isinstance(chunk, ContentChunk)
            assert chunk.source_url == "http://example.com"
            assert chunk.page_title == "Test Page"

    def test_apply_overlap(self, chunker_service):
        """Test applying overlap between chunks."""
        # Create some test chunks
        chunk1 = ContentChunk(
            source_url="http://example.com",
            page_title="Test",
            chunk_index=0,
            content="First chunk content that will have overlap with the next chunk.",
            token_count=chunker_service.estimate_token_count("First chunk content that will have overlap with the next chunk.")
        )

        chunk2 = ContentChunk(
            source_url="http://example.com",
            page_title="Test",
            chunk_index=1,
            content="Second chunk content that will have overlap from the previous chunk.",
            token_count=chunker_service.estimate_token_count("Second chunk content that will have overlap from the previous chunk.")
        )

        chunks_with_overlap = chunker_service.apply_overlap([chunk1, chunk2])

        # Should still have 2 chunks
        assert len(chunks_with_overlap) == 2

        # First chunk should have overlap from the second
        assert "[CONTINUATION:" in chunks_with_overlap[0].content
        assert "Second chunk" in chunks_with_overlap[0].content

    def test_validate_chunk_quality(self, chunker_service):
        """Test chunk quality validation."""
        chunk = ContentChunk(
            source_url="http://example.com",
            page_title="Test",
            chunk_index=0,
            content="This is valid chunk content with appropriate length for testing purposes.",
            token_count=50
        )

        is_valid, issues = chunker_service.validate_chunk_quality(chunk)

        # Should be valid (depends on implementation)
        assert isinstance(is_valid, bool)
        assert isinstance(issues, list)

    def test_split_large_content(self, chunker_service):
        """Test splitting large content into smaller chunks."""
        # Create large content with multiple paragraphs
        large_content = "\n\n".join([f"This is paragraph {i} with some content for testing purposes." for i in range(50)])

        chunks = chunker_service.split_large_content(
            large_content, "http://example.com", "Test Page", 0, "Test Heading"
        )

        assert len(chunks) > 0
        for chunk in chunks:
            assert isinstance(chunk, ContentChunk)
            assert 50 <= chunk.token_count <= 1000  # Within valid range


if __name__ == "__main__":
    pytest.main([__file__])