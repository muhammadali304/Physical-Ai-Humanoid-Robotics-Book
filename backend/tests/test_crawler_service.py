"""
Unit tests for the Crawler service.
"""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from typing import List
from uuid import UUID

from src.services.crawler import CrawlerService
from src.models.job import CrawlJob, JobStatus
from src.models.chunk import ContentChunk


@pytest.fixture
async def crawler_service():
    """Create a crawler service instance for testing."""
    service = CrawlerService()
    # Override external services with mocks
    service.http_client = AsyncMock()
    service.embedding_service = AsyncMock()
    service.chunk_storage_service = AsyncMock()
    service.job_service = AsyncMock()
    return service


@pytest.mark.asyncio
async def test_crawl_page_success(crawler_service):
    """Test successful page crawling."""
    # Mock HTTP response
    mock_response = AsyncMock()
    mock_response.status_code = 200
    mock_response.text = """
    <html>
        <head><title>Test Page</title></head>
        <body>
            <div class="main-wrapper">
                <div class="markdown">This is test content for crawling.</div>
            </div>
        </body>
    </html>
    """
    crawler_service.http_client.get.return_value.__aenter__.return_value = mock_response

    # Mock chunking service
    mock_chunk_result = MagicMock()
    mock_chunk_result.chunks = [
        ContentChunk(
            id="test-chunk-id",
            source_url="https://example.com",
            page_title="Test Page",
            chunk_index=0,
            content="This is test content for crawling.",
            token_count=10
        )
    ]
    crawler_service.chunker_service.chunk_content.return_value = mock_chunk_result

    # Mock embedding service
    crawler_service.embedding_service.generate_and_store_embeddings_from_chunks.return_value = [
        MagicMock()
    ]

    # Mock storage service
    crawler_service.chunk_storage_service.save_chunks = AsyncMock()

    url = "https://example.com"
    result = await crawler_service.crawl_page(url)

    # Verify the result
    assert result is not None
    assert len(result) == 1
    assert result[0].content == "This is test content for crawling."

    # Verify HTTP call
    crawler_service.http_client.get.assert_called_once()


@pytest.mark.asyncio
async def test_crawl_page_http_error(crawler_service):
    """Test page crawling with HTTP error."""
    # Mock HTTP response with error
    mock_response = AsyncMock()
    mock_response.status_code = 404
    crawler_service.http_client.get.return_value.__aenter__.return_value = mock_response

    url = "https://example.com/nonexistent"
    result = await crawler_service.crawl_page(url)

    # Should return None for failed crawl
    assert result is None


@pytest.mark.asyncio
async def test_crawl_page_content_extraction(crawler_service):
    """Test content extraction from HTML."""
    html_content = """
    <html>
        <head><title>Test Page</title></head>
        <body>
            <nav>Navigation content</nav>
            <div class="main-wrapper">
                <div class="markdown">Main content to extract.</div>
            </div>
            <footer>Footer content</footer>
        </body>
    </html>
    """

    # Test the extract_content method directly
    extracted = crawler_service.extract_content(html_content, "https://example.com")

    # Verify that main content is extracted but navigation/footer is not
    assert "Main content to extract." in extracted
    assert "Navigation content" not in extracted
    assert "Footer content" not in extracted


@pytest.mark.asyncio
async def test_crawl_page_title_extraction(crawler_service):
    """Test page title extraction from HTML."""
    html_content = """
    <html>
        <head><title>Page Title</title></head>
        <body><h1>Content Title</h1></body>
    </html>
    """

    # Test the extract_page_title method directly
    extracted = crawler_service.extract_page_title(html_content, "https://example.com")

    # Should extract from <title> tag
    assert extracted == "Page Title"


@pytest.mark.asyncio
async def test_extract_links(crawler_service):
    """Test link extraction from HTML."""
    html_content = """
    <html>
        <body>
            <a href="/page1">Page 1</a>
            <a href="https://example.com/page2">Page 2</a>
            <a href="https://other.com/page3">Other Site</a>
            <a href="#section">Anchor</a>
            <a href="mailto:test@example.com">Email</a>
        </body>
    </html>
    """

    # Test the extract_links method directly
    base_url = "https://example.com"
    extracted_links = crawler_service.extract_links(html_content, base_url)

    # Should include same-domain links, exclude others
    assert len(extracted_links) == 2
    assert "https://example.com/page1" in extracted_links
    assert "https://example.com/page2" in extracted_links
    assert "https://other.com/page3" not in extracted_links  # Different domain
    assert "#section" not in extracted_links  # Anchor link
    assert "mailto:test@example.com" not in extracted_links  # Email link


@pytest.mark.asyncio
async def test_start_crawl_job_success(crawler_service):
    """Test successful crawl job execution."""
    # Create a test job
    job = CrawlJob(
        id=UUID("12345678-1234-5678-1234-567812345678"),
        target_url="https://example.com",
        options={"max_depth": 1}
    )

    # Mock the crawl_page method to return chunks
    crawler_service.crawl_page = AsyncMock(return_value=[
        ContentChunk(
            id="test-chunk-id",
            source_url="https://example.com",
            page_title="Test Page",
            chunk_index=0,
            content="Test content",
            token_count=5
        )
    ])

    # Mock fetch_with_retry to return a successful response
    mock_response = AsyncMock()
    mock_response.status_code = 200
    mock_response.text = """
    <html>
        <body>
            <a href="https://example.com/page2">Next Page</a>
        </body>
    </html>
    """
    crawler_service.fetch_with_retry = AsyncMock(return_value=mock_response)

    # Mock job service updates
    crawler_service.job_service.update_job_status = AsyncMock()
    crawler_service.job_service.update_job_progress = AsyncMock()
    crawler_service.job_service.update_job_completion_time = AsyncMock()

    # Execute the crawl job
    result = await crawler_service.start_crawl_job(job)

    # Verify the results
    assert result.processed_pages >= 1
    assert result.total_pages >= 1

    # Verify job status updates
    crawler_service.job_service.update_job_status.assert_called()
    crawler_service.job_service.update_job_progress.assert_called()
    crawler_service.job_service.update_job_completion_time.assert_called()


@pytest.mark.asyncio
async def test_fetch_with_retry_success(crawler_service):
    """Test successful fetch with retry mechanism."""
    # Mock HTTP response
    mock_response = AsyncMock()
    mock_response.status_code = 200
    mock_response.text = "Success content"
    crawler_service.http_client.get.return_value.__aenter__.return_value = mock_response

    url = "https://example.com"
    result = await crawler_service.fetch_with_retry(url)

    # Should return the successful response
    assert result is not None
    assert result.status_code == 200


@pytest.mark.asyncio
async def test_fetch_with_retry_failure(crawler_service):
    """Test fetch with retry mechanism that ultimately fails."""
    # Mock HTTP response with failure
    mock_response = AsyncMock()
    mock_response.status_code = 500
    crawler_service.http_client.get.return_value.__aenter__.return_value = mock_response

    url = "https://example.com"
    result = await crawler_service.fetch_with_retry(url, max_retries=1)

    # Should return None after retries are exhausted
    assert result is None


@pytest.mark.asyncio
async def test_extract_content_with_no_content(crawler_service):
    """Test content extraction when no content is found."""
    html_content = """
    <html>
        <head><title>Test Page</title></head>
        <body>
            <nav>Navigation</nav>
            <footer>Footer</footer>
        </body>
    </html>
    """

    extracted = crawler_service.extract_content(html_content, "https://example.com")

    # Should return None when no content is found
    assert extracted is None


@pytest.mark.asyncio
async def test_crawl_page_short_content(crawler_service):
    """Test crawling page with content that is too short."""
    # Mock HTTP response with very short content
    mock_response = AsyncMock()
    mock_response.status_code = 200
    mock_response.text = "<html><body><p>Hi.</p></body></html>"
    crawler_service.http_client.get.return_value.__aenter__.return_value = mock_response

    # Mock chunking to return no chunks
    mock_chunk_result = MagicMock()
    mock_chunk_result.chunks = []
    crawler_service.chunker_service.chunk_content.return_value = mock_chunk_result

    url = "https://example.com"
    result = await crawler_service.crawl_page(url)

    # Should return empty list for short content
    assert result == []


@pytest.mark.asyncio
async def test_validate_selectors_on_site(crawler_service):
    """Test CSS selector validation."""
    # Mock successful fetch
    mock_response = AsyncMock()
    mock_response.status_code = 200
    mock_response.text = """
    <html>
        <body>
            <div class="main-wrapper">Content</div>
            <nav>Navigation</nav>
        </body>
    </html>
    """
    crawler_service.fetch_with_retry = AsyncMock(return_value=mock_response)

    # Test selector validation
    result = await crawler_service.validate_selectors_on_site("https://example.com")

    # Should have validation results
    assert "overall_success" in result
    assert "results" in result