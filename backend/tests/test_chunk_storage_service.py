"""
Unit tests for the Chunk Storage service.
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from typing import List
from uuid import UUID
from datetime import datetime

from src.services.chunk_storage_service import ChunkStorageService
from src.models.chunk import ContentChunk


@pytest.fixture
async def chunk_storage_service():
    """Create a chunk storage service instance for testing."""
    service = ChunkStorageService()
    # Override external services with mocks
    service.repository = AsyncMock()
    return service


@pytest.mark.asyncio
async def test_save_chunks_success(chunk_storage_service):
    """Test successful saving of multiple chunks."""
    # Create test chunks
    chunks = [
        ContentChunk(
            id=UUID("12345678-1234-5678-1234-567812345678"),
            source_url="https://example.com/page1",
            page_title="Test Page 1",
            chunk_index=0,
            content="Content of first chunk",
            token_count=10
        ),
        ContentChunk(
            id=UUID("87654321-4321-8765-4321-876543218765"),
            source_url="https://example.com/page1",
            page_title="Test Page 1",
            chunk_index=1,
            content="Content of second chunk",
            token_count=15
        )
    ]

    job_id = "test-job-id"

    # Mock repository behavior
    chunk_storage_service.repository.batch_create.return_value = True

    result = await chunk_storage_service.save_chunks(chunks, job_id)

    # Verify the result
    assert result is True
    chunk_storage_service.repository.batch_create.assert_called_once_with(chunks)


@pytest.mark.asyncio
async def test_save_chunks_empty_list(chunk_storage_service):
    """Test saving an empty list of chunks."""
    result = await chunk_storage_service.save_chunks([], "test-job-id")

    # Should return True for empty list
    assert result is True
    # Should not call repository for empty list
    chunk_storage_service.repository.batch_create.assert_not_called()


@pytest.mark.asyncio
async def test_save_chunks_repository_failure(chunk_storage_service):
    """Test chunk saving when repository fails."""
    chunks = [
        ContentChunk(
            id=UUID("12345678-1234-5678-1234-567812345678"),
            source_url="https://example.com",
            page_title="Test Page",
            chunk_index=0,
            content="Test content",
            token_count=10
        )
    ]

    # Mock repository to return False (failure)
    chunk_storage_service.repository.batch_create.return_value = False

    result = await chunk_storage_service.save_chunks(chunks, "test-job-id")

    # Should return False when repository fails
    assert result is False


@pytest.mark.asyncio
async def test_get_chunk_by_id_success(chunk_storage_service):
    """Test successful retrieval of a chunk by ID."""
    chunk_id = UUID("12345678-1234-5678-1234-567812345678")
    expected_chunk = ContentChunk(
        id=chunk_id,
        source_url="https://example.com",
        page_title="Test Page",
        chunk_index=0,
        content="Test content",
        token_count=10
    )

    # Mock repository behavior
    chunk_storage_service.repository.get_by_id.return_value = expected_chunk

    result = await chunk_storage_service.get_chunk_by_id(str(chunk_id))

    # Verify the result
    assert result == expected_chunk
    chunk_storage_service.repository.get_by_id.assert_called_once_with(chunk_id)


@pytest.mark.asyncio
async def test_get_chunk_by_id_not_found(chunk_storage_service):
    """Test chunk retrieval when chunk doesn't exist."""
    chunk_id = UUID("12345678-1234-5678-1234-567812345678")

    # Mock repository to return None
    chunk_storage_service.repository.get_by_id.return_value = None

    result = await chunk_storage_service.get_chunk_by_id(str(chunk_id))

    # Should return None when not found
    assert result is None


@pytest.mark.asyncio
async def test_get_chunks_by_job_id_success(chunk_storage_service):
    """Test successful retrieval of chunks by job ID."""
    job_id = "test-job-id"
    expected_chunks = [
        ContentChunk(
            id=UUID("12345678-1234-5678-1234-567812345678"),
            source_url="https://example.com",
            page_title="Test Page 1",
            chunk_index=0,
            content="First chunk",
            token_count=10
        ),
        ContentChunk(
            id=UUID("87654321-4321-8765-4321-876543218765"),
            source_url="https://example.com",
            page_title="Test Page 1",
            chunk_index=1,
            content="Second chunk",
            token_count=12
        )
    ]

    # Mock repository behavior - filter chunks by job ID in metadata
    all_chunks = expected_chunks[:]
    # Add metadata with job_id
    for chunk in all_chunks:
        chunk.metadata = {"crawl_job_id": job_id}

    chunk_storage_service.repository.list.return_value = all_chunks

    result = await chunk_storage_service.get_chunks_by_job_id(job_id)

    # Verify the result
    assert result == expected_chunks
    assert len(result) == 2


@pytest.mark.asyncio
async def test_get_chunks_by_job_id_empty_result(chunk_storage_service):
    """Test chunk retrieval by job ID when no chunks exist for the job."""
    job_id = "test-job-id"
    other_job_id = "other-job-id"

    # Mock repository to return chunks from a different job
    other_chunks = [
        ContentChunk(
            id=UUID("12345678-1234-5678-1234-567812345678"),
            source_url="https://example.com",
            page_title="Test Page",
            chunk_index=0,
            content="Other job chunk",
            token_count=10,
            metadata={"crawl_job_id": other_job_id}
        )
    ]

    chunk_storage_service.repository.list.return_value = other_chunks

    result = await chunk_storage_service.get_chunks_by_job_id(job_id)

    # Should return empty list when no chunks found for job
    assert result == []


@pytest.mark.asyncio
async def test_get_chunks_by_source_url_success(chunk_storage_service):
    """Test successful retrieval of chunks by source URL."""
    source_url = "https://example.com/docs"
    expected_chunks = [
        ContentChunk(
            id=UUID("12345678-1234-5678-1234-567812345678"),
            source_url=source_url,
            page_title="Documentation",
            chunk_index=0,
            content="First doc chunk",
            token_count=15
        ),
        ContentChunk(
            id=UUID("87654321-4321-8765-4321-876543218765"),
            source_url=source_url,
            page_title="Documentation",
            chunk_index=1,
            content="Second doc chunk",
            token_count=18
        )
    ]

    # Mock repository behavior - filter chunks by source URL
    all_chunks = expected_chunks[:]
    chunk_storage_service.repository.list.return_value = all_chunks

    result = await chunk_storage_service.get_chunks_by_source_url(source_url)

    # Verify the result
    assert result == expected_chunks
    assert len(result) == 2


@pytest.mark.asyncio
async def test_get_chunks_by_source_url_not_found(chunk_storage_service):
    """Test chunk retrieval by source URL when no chunks exist."""
    source_url = "https://example.com/nonexistent"
    other_url = "https://example.com/different"

    # Mock repository to return chunks from a different URL
    other_chunks = [
        ContentChunk(
            id=UUID("12345678-1234-5678-1234-567812345678"),
            source_url=other_url,
            page_title="Different Page",
            chunk_index=0,
            content="Different content",
            token_count=10
        )
    ]

    chunk_storage_service.repository.list.return_value = other_chunks

    result = await chunk_storage_service.get_chunks_by_source_url(source_url)

    # Should return empty list when no chunks found for URL
    assert result == []


@pytest.mark.asyncio
async def test_get_all_chunks_success(chunk_storage_service):
    """Test successful retrieval of all chunks."""
    expected_chunks = [
        ContentChunk(
            id=UUID("12345678-1234-5678-1234-567812345678"),
            source_url="https://example.com/page1",
            page_title="Page 1",
            chunk_index=0,
            content="Content 1",
            token_count=10
        ),
        ContentChunk(
            id=UUID("87654321-4321-8765-4321-876543218765"),
            source_url="https://example.com/page2",
            page_title="Page 2",
            chunk_index=0,
            content="Content 2",
            token_count=12
        )
    ]

    # Mock repository behavior
    chunk_storage_service.repository.list.return_value = expected_chunks

    result = await chunk_storage_service.get_all_chunks(limit=10, offset=0)

    # Verify the result
    assert result == expected_chunks
    assert len(result) == 2


@pytest.mark.asyncio
async def test_get_all_chunks_with_pagination(chunk_storage_service):
    """Test retrieval of all chunks with pagination."""
    all_chunks = []
    for i in range(20):
        chunk = ContentChunk(
            id=UUID(f"12345678-1234-5678-1234-{i:012d}"),
            source_url=f"https://example.com/page{i}",
            page_title=f"Page {i}",
            chunk_index=0,
            content=f"Content {i}",
            token_count=10
        )
        all_chunks.append(chunk)

    # Mock repository behavior
    chunk_storage_service.repository.list.return_value = all_chunks

    # Test with limit and offset
    result = await chunk_storage_service.get_all_chunks(limit=5, offset=10)

    # Should return 5 chunks starting from the 11th chunk (index 10)
    assert len(result) == 5
    assert result[0].page_title == "Page 10"
    assert result[-1].page_title == "Page 14"


@pytest.mark.asyncio
async def test_get_chunks_count(chunk_storage_service):
    """Test retrieval of total chunks count."""
    expected_chunks = [
        ContentChunk(id=UUID("1"), source_url="https://a.com", page_title="A", chunk_index=0, content="A", token_count=1),
        ContentChunk(id=UUID("2"), source_url="https://b.com", page_title="B", chunk_index=0, content="B", token_count=1),
        ContentChunk(id=UUID("3"), source_url="https://c.com", page_title="C", chunk_index=0, content="C", token_count=1)
    ]

    # Mock repository behavior
    chunk_storage_service.repository.list.return_value = expected_chunks

    count = await chunk_storage_service.get_chunks_count()

    # Verify the count
    assert count == 3


@pytest.mark.asyncio
async def test_get_chunks_count_empty(chunk_storage_service):
    """Test retrieval of chunks count when no chunks exist."""
    # Mock repository to return empty list
    chunk_storage_service.repository.list.return_value = []

    count = await chunk_storage_service.get_chunks_count()

    # Verify the count
    assert count == 0


@pytest.mark.asyncio
async def test_delete_chunk_by_id_success(chunk_storage_service):
    """Test successful deletion of a chunk by ID."""
    chunk_id = UUID("12345678-1234-5678-1234-567812345678")

    # Mock repository behavior
    chunk_storage_service.repository.delete.return_value = True

    result = await chunk_storage_service.delete_chunk_by_id(str(chunk_id))

    # Verify the result
    assert result is True
    chunk_storage_service.repository.delete.assert_called_once_with(chunk_id)


@pytest.mark.asyncio
async def test_delete_chunk_by_id_failure(chunk_storage_service):
    """Test chunk deletion failure."""
    chunk_id = UUID("12345678-1234-5678-1234-567812345678")

    # Mock repository to return False (failure)
    chunk_storage_service.repository.delete.return_value = False

    result = await chunk_storage_service.delete_chunk_by_id(str(chunk_id))

    # Verify the result
    assert result is False


@pytest.mark.asyncio
async def test_update_chunk_success(chunk_storage_service):
    """Test successful chunk update."""
    updated_chunk = ContentChunk(
        id=UUID("12345678-1234-5678-1234-567812345678"),
        source_url="https://example.com",
        page_title="Updated Page",
        chunk_index=0,
        content="Updated content",
        token_count=15
    )

    # Mock repository behavior
    chunk_storage_service.repository.update.return_value = True

    result = await chunk_storage_service.update_chunk(updated_chunk)

    # Verify the result
    assert result is True
    chunk_storage_service.repository.update.assert_called_once_with(updated_chunk)


@pytest.mark.asyncio
async def test_update_chunk_failure(chunk_storage_service):
    """Test chunk update failure."""
    updated_chunk = ContentChunk(
        id=UUID("12345678-1234-5678-1234-567812345678"),
        source_url="https://example.com",
        page_title="Updated Page",
        chunk_index=0,
        content="Updated content",
        token_count=15
    )

    # Mock repository to return False (failure)
    chunk_storage_service.repository.update.return_value = False

    result = await chunk_storage_service.update_chunk(updated_chunk)

    # Verify the result
    assert result is False


@pytest.mark.asyncio
async def test_search_chunks_by_content_success(chunk_storage_service):
    """Test successful content-based chunk search."""
    search_term = "test"
    expected_chunks = [
        ContentChunk(
            id=UUID("12345678-1234-5678-1234-567812345678"),
            source_url="https://example.com",
            page_title="Test Page",
            chunk_index=0,
            content="This is a test content",
            token_count=10
        )
    ]

    # Mock repository to return chunks that contain the search term
    all_chunks = expected_chunks[:]
    chunk_storage_service.repository.list.return_value = all_chunks

    result = await chunk_storage_service.search_chunks_by_content(search_term)

    # Verify the result
    assert result == expected_chunks
    assert len(result) == 1


@pytest.mark.asyncio
async def test_search_chunks_by_content_case_insensitive(chunk_storage_service):
    """Test content-based chunk search with case insensitivity."""
    search_term = "TEST"
    expected_chunks = [
        ContentChunk(
            id=UUID("12345678-1234-5678-1234-567812345678"),
            source_url="https://example.com",
            page_title="Test Page",
            chunk_index=0,
            content="This is a test content",
            token_count=10
        )
    ]

    # Mock repository to return chunks that contain the search term (case-insensitive)
    all_chunks = expected_chunks[:]
    chunk_storage_service.repository.list.return_value = all_chunks

    result = await chunk_storage_service.search_chunks_by_content(search_term)

    # Verify the result
    assert result == expected_chunks
    assert len(result) == 1


@pytest.mark.asyncio
async def test_search_chunks_by_content_not_found(chunk_storage_service):
    """Test content-based chunk search when no matches found."""
    search_term = "nonexistentterm"

    # Mock repository to return chunks without the search term
    all_chunks = [
        ContentChunk(
            id=UUID("12345678-1234-5678-1234-567812345678"),
            source_url="https://example.com",
            page_title="Test Page",
            chunk_index=0,
            content="This is different content",
            token_count=10
        )
    ]

    chunk_storage_service.repository.list.return_value = all_chunks

    result = await chunk_storage_service.search_chunks_by_content(search_term)

    # Should return empty list when no matches found
    assert result == []


@pytest.mark.asyncio
async def test_get_chunks_by_page_title_success(chunk_storage_service):
    """Test successful retrieval of chunks by page title."""
    page_title = "API Documentation"
    expected_chunks = [
        ContentChunk(
            id=UUID("12345678-1234-5678-1234-567812345678"),
            source_url="https://example.com/api",
            page_title=page_title,
            chunk_index=0,
            content="API content",
            token_count=20
        )
    ]

    # Mock repository to return chunks with matching page title
    all_chunks = expected_chunks[:]
    chunk_storage_service.repository.list.return_value = all_chunks

    result = await chunk_storage_service.get_chunks_by_page_title(page_title)

    # Verify the result
    assert result == expected_chunks
    assert len(result) == 1


@pytest.mark.asyncio
async def test_get_chunks_by_page_title_multiple_pages(chunk_storage_service):
    """Test retrieval of chunks by page title with multiple chunks per page."""
    page_title = "Guide"
    expected_chunks = [
        ContentChunk(
            id=UUID("12345678-1234-5678-1234-567812345678"),
            source_url="https://example.com/guide",
            page_title=page_title,
            chunk_index=0,
            content="Guide intro",
            token_count=15
        ),
        ContentChunk(
            id=UUID("87654321-4321-8765-4321-876543218765"),
            source_url="https://example.com/guide",
            page_title=page_title,
            chunk_index=1,
            content="Guide details",
            token_count=25
        )
    ]

    # Mock repository to return multiple chunks with same page title
    all_chunks = expected_chunks[:]
    chunk_storage_service.repository.list.return_value = all_chunks

    result = await chunk_storage_service.get_chunks_by_page_title(page_title)

    # Verify the result
    assert result == expected_chunks
    assert len(result) == 2


@pytest.mark.asyncio
async def test_get_chunks_statistics(chunk_storage_service):
    """Test retrieval of chunks statistics."""
    test_chunks = [
        ContentChunk(
            id=UUID("12345678-1234-5678-1234-567812345678"),
            source_url="https://example.com/page1",
            page_title="Page 1",
            chunk_index=0,
            content="Content 1",
            token_count=50
        ),
        ContentChunk(
            id=UUID("87654321-4321-8765-4321-876543218765"),
            source_url="https://example.com/page2",
            page_title="Page 2",
            chunk_index=0,
            content="Content 2",
            token_count=30
        ),
        ContentChunk(
            id=UUID("11111111-1111-1111-1111-111111111111"),
            source_url="https://example.com/page1",  # Same URL as first
            page_title="Page 1",  # Same title as first
            chunk_index=1,
            content="Content 3",
            token_count=40
        )
    ]

    # Mock repository behavior
    chunk_storage_service.repository.list.return_value = test_chunks

    stats = await chunk_storage_service.get_chunks_statistics()

    # Verify statistics
    assert stats["total_chunks"] == 3
    assert stats["total_tokens"] == 120  # 50 + 30 + 40
    assert stats["unique_sources"] == 2  # page1 and page2
    assert stats["avg_tokens_per_chunk"] == 40  # 120 / 3
    assert stats["max_tokens_in_chunk"] == 50
    assert stats["min_tokens_in_chunk"] == 30


@pytest.mark.asyncio
async def test_get_chunks_statistics_empty(chunk_storage_service):
    """Test retrieval of chunks statistics when no chunks exist."""
    # Mock repository to return empty list
    chunk_storage_service.repository.list.return_value = []

    stats = await chunk_storage_service.get_chunks_statistics()

    # Verify statistics
    assert stats["total_chunks"] == 0
    assert stats["total_tokens"] == 0
    assert stats["unique_sources"] == 0
    assert stats["avg_tokens_per_chunk"] == 0
    assert stats["max_tokens_in_chunk"] == 0
    assert stats["min_tokens_in_chunk"] == 0