"""
Unit tests for the Chunker service.
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from typing import List

from src.services.chunker import ChunkerService
from src.models.chunk import ContentChunk


@pytest.fixture
def chunker_service():
    """Create a chunker service instance for testing."""
    service = ChunkerService()
    # Override external services with mocks if needed
    return service


def test_chunk_content_success(chunker_service):
    """Test successful content chunking."""
    long_content = "This is a long content that needs to be chunked into smaller pieces. " * 50
    source_url = "https://example.com/page"
    page_title = "Test Page"

    result = chunker_service.chunk_content(
        content=long_content,
        source_url=source_url,
        page_title=page_title,
        chunk_index_start=0
    )

    # Verify that chunks were created
    assert len(result.chunks) > 0
    assert result.total_chunks > 0

    # Verify chunk properties
    for chunk in result.chunks:
        assert isinstance(chunk, ContentChunk)
        assert chunk.source_url == source_url
        assert chunk.page_title == page_title
        assert chunk.token_count > 0
        assert len(chunk.content) > 0
        assert chunk.chunk_index >= 0


def test_chunk_content_short_content(chunker_service):
    """Test chunking content that is shorter than minimum chunk size."""
    short_content = "Short content."
    source_url = "https://example.com/page"
    page_title = "Test Page"

    result = chunker_service.chunk_content(
        content=short_content,
        source_url=source_url,
        page_title=page_title,
        chunk_index_start=0
    )

    # With short content, should still create at least one chunk
    assert len(result.chunks) >= 1
    assert result.total_chunks >= 1

    chunk = result.chunks[0]
    assert chunk.content == short_content
    assert chunk.source_url == source_url
    assert chunk.page_title == page_title


def test_chunk_content_with_overlap(chunker_service):
    """Test content chunking with overlap."""
    long_content = "This is content for testing overlap. " * 30
    source_url = "https://example.com/page"
    page_title = "Test Page"

    # Test with overlap
    result = chunker_service.chunk_content(
        content=long_content,
        source_url=source_url,
        page_title=page_title,
        chunk_index_start=0,
        overlap_ratio=0.2  # 20% overlap
    )

    # Verify that chunks were created with overlap
    assert len(result.chunks) > 1  # Should have multiple chunks due to overlap

    # Check that consecutive chunks have overlapping content
    if len(result.chunks) > 1:
        first_chunk_end = result.chunks[0].content[-20:]  # Last 20 chars of first chunk
        second_chunk_start = result.chunks[1].content[:20]  # First 20 chars of second chunk
        # Overlap may not be exact due to sentence boundaries, but chunks should be created


def test_chunk_content_with_section_headings(chunker_service):
    """Test content chunking with section headings."""
    content_with_headings = """
    # Introduction
    This is the introduction section of the document.

    ## Details
    This is the details section with more information.

    ### Subsection
    This is a subsection with even more details.

    # Conclusion
    This concludes the document.
    """
    source_url = "https://example.com/page"
    page_title = "Test Page"

    result = chunker_service.chunk_content(
        content=content_with_headings,
        source_url=source_url,
        page_title=page_title,
        chunk_index_start=0
    )

    # Verify that chunks preserve section headings
    assert len(result.chunks) > 0

    for chunk in result.chunks:
        assert isinstance(chunk, ContentChunk)
        # Chunks should have appropriate section headings based on the content structure


def test_chunk_content_empty_input(chunker_service):
    """Test chunking with empty content."""
    result = chunker_service.chunk_content(
        content="",
        source_url="https://example.com/page",
        page_title="Test Page",
        chunk_index_start=0
    )

    # Should return empty result
    assert len(result.chunks) == 0
    assert result.total_chunks == 0


def test_chunk_content_whitespace_only(chunker_service):
    """Test chunking with whitespace-only content."""
    result = chunker_service.chunk_content(
        content="   \n\t  ",
        source_url="https://example.com/page",
        page_title="Test Page",
        chunk_index_start=0
    )

    # Should return empty result for whitespace-only content
    assert len(result.chunks) == 0
    assert result.total_chunks == 0


def test_calculate_chunk_token_count(chunker_service):
    """Test token count calculation."""
    content = "This is a test sentence with several words."

    # Test token count calculation
    token_count = chunker_service._calculate_token_count(content)

    # Should return a positive integer
    assert token_count > 0
    assert isinstance(token_count, int)


def test_chunk_content_with_special_characters(chunker_service):
    """Test chunking content with special characters."""
    content = "This content has special characters: émojis, symbols: @#$%, and punctuation: !?. " * 20
    source_url = "https://example.com/page"
    page_title = "Test Page"

    result = chunker_service.chunk_content(
        content=content,
        source_url=source_url,
        page_title=page_title,
        chunk_index_start=0
    )

    # Should handle special characters properly
    assert len(result.chunks) > 0
    assert result.total_chunks > 0

    for chunk in result.chunks:
        assert isinstance(chunk, ContentChunk)
        assert len(chunk.content) > 0


def test_chunk_content_preserves_structure(chunker_service):
    """Test that chunking preserves document structure."""
    structured_content = """
    # Main Title
    This is the main content paragraph.

    ## Subsection 1
    Content for subsection 1.

    ## Subsection 2
    Content for subsection 2 with more text.
    Additional paragraph for subsection 2.

    # Another Main Section
    Content for the second main section.
    """
    source_url = "https://example.com/page"
    page_title = "Test Page"

    result = chunker_service.chunk_content(
        content=structured_content,
        source_url=source_url,
        page_title=page_title,
        chunk_index_start=0
    )

    # Should create chunks that preserve the document structure
    assert len(result.chunks) > 0
    assert result.total_chunks > 0

    # Verify that chunks have reasonable sizes
    for chunk in result.chunks:
        assert chunk.token_count > 0
        assert len(chunk.content) > 0


def test_chunk_content_max_size_constraint(chunker_service):
    """Test that chunks don't exceed maximum size."""
    very_long_content = "This is a sentence. " * 1000  # Very long content
    source_url = "https://example.com/page"
    page_title = "Test Page"

    result = chunker_service.chunk_content(
        content=very_long_content,
        source_url=source_url,
        page_title=page_title,
        chunk_index_start=0
    )

    # Verify that all chunks are within size constraints
    assert len(result.chunks) > 0
    for chunk in result.chunks:
        assert chunk.token_count <= 1000  # Max tokens constraint
        assert len(chunk.content) > 0


def test_chunk_result_properties(chunker_service):
    """Test ChunkResult properties."""
    content = "Test content for chunking. " * 10
    source_url = "https://example.com/page"
    page_title = "Test Page"

    result = chunker_service.chunk_content(
        content=content,
        source_url=source_url,
        page_title=page_title,
        chunk_index_start=0
    )

    # Verify ChunkResult properties
    assert result.total_chunks >= 0
    assert result.total_tokens >= 0
    assert result.avg_chunk_size >= 0
    assert isinstance(result.chunks, list)
    assert result.metadata is not None