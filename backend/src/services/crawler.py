"""
Web crawler service for the RAG Ingestion Pipeline.
Implements Docusaurus-specific content extraction and crawling functionality.
"""

import asyncio
import re
from datetime import datetime
from typing import List, Dict, Any, Optional, Set
from urllib.parse import urljoin, urlparse
from dataclasses import dataclass

import httpx
from bs4 import BeautifulSoup
from pydantic import BaseModel

from src.config.settings import settings
from src.config.constants import (
    DEFAULT_MAX_DEPTH,
    DEFAULT_RATE_LIMIT_DELAY,
    MIN_CONTENT_LENGTH
)
from src.models.job import CrawlJob, JobStatus
from src.models.chunk import ContentChunk
from src.utils.http_client import HttpClient, create_default_http_client
from src.utils.logging import get_logger, log_performance
from src.utils.queue import get_queue_manager
from src.services.job_service import CrawlJobService
from src.services.chunker import ChunkerService
from src.services.chunk_storage_service import ChunkStorageService
from src.services.embedding_service import CohereEmbeddingService
from src.services.event_publisher import EventPublisher
from src.services.worker import RQWorkerService


@dataclass
class CrawlResult:
    """Result of a crawl operation."""
    processed_pages: int = 0
    failed_pages: int = 0
    total_pages: int = 0
    crawled_urls: List[str] = None

    def __post_init__(self):
        if self.crawled_urls is None:
            self.crawled_urls = []


class CrawlerService:
    """Service to crawl Docusaurus-based documentation sites."""

    def __init__(self):
        self.http_client = create_default_http_client()
        self.rate_limit_delay = settings.rate_limit_delay
        self.job_service = CrawlJobService()
        self.chunker_service = ChunkerService()
        self.chunk_storage_service = ChunkStorageService()
        self.embedding_service = CohereEmbeddingService()
        self.event_publisher = EventPublisher()
        self.worker_service = RQWorkerService()
        self.logger = get_logger("crawler")
        self.visited_urls: Set[str] = set()
        self.discovered_urls: Set[str] = set()

        # Docusaurus-specific CSS selectors for content extraction
        self.content_selectors = [
            ".main-wrapper .markdown",
            ".theme-doc-markdown",
            ".doc-content",
            "article",
            ".container .col",
            ".post",
            ".docs-content",
            ".markdown",
            ".content"
        ]

        # Docusaurus-specific CSS selectors for navigation elements to exclude
        self.navigation_selectors = [
            ".menu",
            ".nav",
            ".sidebar",
            ".navbar",
            ".footer",
            ".header",
            ".toc",
            ".pagination",
            ".breadcrumb",
            ".tag",
            ".social",
            ".share"
        ]

        # Initialize queue manager for processing pages
        from src.utils.queue import get_queue_manager
        self.queue_manager = get_queue_manager()

    async def _apply_rate_limit(self):
        """Apply rate limiting delay between requests."""
        if self.rate_limit_delay > 0:
            await asyncio.sleep(self.rate_limit_delay)

    @log_performance("fetch_with_retry")
    async def fetch_with_retry(self, url: str, max_retries: int = settings.max_retries) -> Optional[httpx.Response]:
        """
        Fetch a URL with retry mechanism and exponential backoff.

        Args:
            url: URL to fetch
            max_retries: Maximum number of retry attempts

        Returns:
            HTTP response or None if all retries failed
        """
        last_exception = None

        for attempt in range(max_retries + 1):
            try:
                self.logger.debug(
                    f"Fetching {url} (attempt {attempt + 1}/{max_retries + 1})",
                    url=url,
                    attempt=attempt + 1,
                    max_retries=max_retries
                )

                # Apply rate limiting before each request
                await self._apply_rate_limit()

                async with self.http_client as client:
                    response = await client.get(url)

                # If successful, return the response
                if response.status_code == 200:
                    self.logger.debug(
                        f"Successfully fetched {url} on attempt {attempt + 1}",
                        url=url,
                        attempt=attempt + 1,
                        status_code=response.status_code
                    )
                    return response

                # For client errors (4xx) except 429, don't retry
                if 400 <= response.status_code < 500 and response.status_code != 429:
                    self.logger.warning(
                        f"Client error {response.status_code} for {url}, not retrying",
                        url=url,
                        status_code=response.status_code
                    )
                    return response

                # For server errors (5xx) or rate limit (429), continue to retry
                self.logger.warning(
                    f"Server error {response.status_code} for {url}, will retry",
                    url=url,
                    status_code=response.status_code,
                    attempt=attempt + 1
                )

            except Exception as e:
                self.logger.warning(
                    f"Request failed for {url} (attempt {attempt + 1}): {str(e)}",
                    url=url,
                    attempt=attempt + 1,
                    error=str(e)
                )
                last_exception = e

            # If this is not the last attempt, wait before retrying (exponential backoff)
            if attempt < max_retries:
                # Calculate backoff time: base_delay * (backoff_factor ^ attempt)
                base_delay = 1.0  # seconds
                backoff_factor = 2.0
                wait_time = base_delay * (backoff_factor ** attempt)

                self.logger.info(
                    f"Waiting {wait_time:.2f}s before retry {attempt + 2} for {url}",
                    url=url,
                    wait_time=wait_time,
                    next_attempt=attempt + 2
                )

                await asyncio.sleep(wait_time)

        # If we've exhausted all retries, log the failure and return None
        if last_exception:
            self.logger.error(
                f"All retry attempts failed for {url}, last error: {str(last_exception)}",
                url=url,
                max_retries=max_retries,
                error=str(last_exception)
            )
        else:
            self.logger.error(
                f"All retry attempts failed for {url} due to HTTP errors",
                url=url,
                max_retries=max_retries
            )

        return None

    @log_performance("crawl_single_page")
    async def crawl_page(self, url: str, job_id: Optional[str] = None) -> Optional[List[ContentChunk]]:
        """
        Crawl a single page, extract its content, chunk it, and store the chunks.

        Args:
            url: URL to crawl
            job_id: Optional crawl job ID to associate with chunks

        Returns:
            List of ContentChunk objects or None if failed
        """
        try:
            self.logger.info(f"Crawling page: {url}", url=url)

            # Use the retry mechanism to fetch the page
            response = await self.fetch_with_retry(url)

            if response is None:
                self.logger.error(
                    f"Failed to crawl {url} after all retry attempts",
                    url=url
                )
                return None

            if response.status_code != 200:
                self.logger.error(
                    f"Failed to crawl {url}, final status: {response.status_code}",
                    url=url,
                    status_code=response.status_code
                )
                return None

            content = response.text
            extracted_content = self.extract_content(content, url)

            if extracted_content and len(extracted_content.strip()) >= MIN_CONTENT_LENGTH:
                self.logger.info(
                    f"Successfully extracted content from {url}",
                    url=url,
                    content_length=len(extracted_content)
                )

                # Extract page title from HTML
                page_title = self.extract_page_title(content, url)

                # Chunk the content
                chunk_result = self.chunker_service.chunk_content(
                    content=extracted_content,
                    source_url=url,
                    page_title=page_title,
                    chunk_index_start=0
                )

                # Store the chunks
                if chunk_result.chunks:
                    # Filter out chunks that are too small to meet validation requirements
                    valid_chunks = []
                    for chunk in chunk_result.chunks:
                        if len(chunk.content) >= 50 and chunk.token_count >= 50:
                            valid_chunks.append(chunk)
                        else:
                            self.logger.warning(
                                f"Skipping chunk with {len(chunk.content)} chars and {chunk.token_count} tokens (too small)",
                                url=url,
                                chunk_index=chunk.chunk_index
                            )

                    if valid_chunks:
                        await self.chunk_storage_service.save_chunks(valid_chunks, job_id)
                        self.logger.info(
                            f"Successfully chunked and stored {len(valid_chunks)} valid chunks from {url} (filtered from {len(chunk_result.chunks)} total)",
                            url=url,
                            chunk_count=len(valid_chunks),
                            total_chunks=len(chunk_result.chunks)
                        )

                        # Generate and store embeddings for the valid chunks
                        try:
                            embedding_results = await self.embedding_service.generate_and_store_embeddings_from_chunks(valid_chunks)
                            self.logger.info(
                                f"Successfully generated and stored {len(embedding_results)} embeddings for {url}",
                                url=url,
                                embedding_count=len(embedding_results)
                            )
                        except Exception as e:
                            self.logger.error(
                                f"Error generating embeddings for {url}: {str(e)}",
                                url=url,
                                error=str(e)
                            )
                            # Continue processing even if embedding generation fails

                        return valid_chunks
                    else:
                        self.logger.warning(
                            f"No valid chunks (meeting size requirements) created from {url}",
                            url=url
                        )
                        return []
                else:
                    self.logger.warning(
                        f"No chunks created from {url}",
                        url=url
                    )
                    return []
            else:
                self.logger.warning(
                    f"Content too short or empty from {url}",
                    url=url,
                    content_length=len(extracted_content) if extracted_content else 0
                )
                return []

        except Exception as e:
            self.logger.error(
                f"Error crawling page {url}: {str(e)}",
                url=url,
                error=str(e)
            )
            return None

    def extract_page_title(self, html_content: str, url: str) -> str:
        """
        Extract the page title from HTML content.

        Args:
            html_content: Raw HTML content
            url: URL of the page (for fallback)

        Returns:
            Extracted page title or URL as fallback
        """
        try:
            soup = BeautifulSoup(html_content, 'html.parser')

            # Try to find title tag
            title_tag = soup.find('title')
            if title_tag:
                return title_tag.get_text().strip()

            # Try to find h1 as title
            h1_tag = soup.find('h1')
            if h1_tag:
                return h1_tag.get_text().strip()

            # Fallback to URL
            return url
        except Exception as e:
            self.logger.warning(
                f"Error extracting title from {url}: {str(e)}",
                url=url,
                error=str(e)
            )
            return url

    def extract_content(self, html_content: str, url: str) -> Optional[str]:
        """
        Extract clean text content from HTML using Docusaurus-specific selectors.

        Args:
            html_content: Raw HTML content
            url: URL of the page (for context)

        Returns:
            Clean text content or None if extraction failed
        """
        try:
            soup = BeautifulSoup(html_content, 'html.parser')

            # Remove navigation elements
            for selector in self.navigation_selectors:
                elements = soup.select(selector)
                for element in elements:
                    element.decompose()

            # Try to find content using specific selectors
            content_element = None
            for selector in self.content_selectors:
                content_element = soup.select_one(selector)
                if content_element:
                    break

            # If no specific content found, try body
            if not content_element:
                content_element = soup.find('body')

            if not content_element:
                self.logger.warning(f"No content found for {url}")
                return None

            # Extract text and clean it up
            text = content_element.get_text(separator=' ', strip=True)

            # Remove extra whitespace
            text = re.sub(r'\s+', ' ', text)

            # Remove common non-content patterns
            text = re.sub(r'\s+', ' ', text)  # Normalize whitespace again after cleaning
            text = text.strip()

            return text

        except Exception as e:
            self.logger.error(
                f"Error extracting content from {url}: {str(e)}",
                url=url,
                error=str(e)
            )
            return None

    def clean_content(self, content: str) -> str:
        """
        Clean content by removing non-content elements like navigation, headers, footers.

        Args:
            content: Raw content to clean

        Returns:
            Clean content with non-content elements removed
        """
        if not content:
            return ""

        # Remove extra whitespace and normalize
        cleaned = re.sub(r'\s+', ' ', content)
        cleaned = cleaned.strip()

        # Additional cleaning could be implemented here
        # For now, just return the basic cleaned content
        return cleaned

    @log_performance("extract_links")
    def extract_links(self, html_content: str, base_url: str) -> List[str]:
        """
        Extract all valid documentation links from HTML content.

        Args:
            html_content: Raw HTML content
            base_url: Base URL for resolving relative links

        Returns:
            List of absolute URLs
        """
        try:
            soup = BeautifulSoup(html_content, 'html.parser')
            links = []

            for link in soup.find_all('a', href=True):
                href = link['href']

                # Skip if it's an anchor link or mailto/phone link
                if href.startswith(('#', 'mailto:', 'tel:')):
                    continue

                # Convert to absolute URL
                absolute_url = urljoin(base_url, href)

                # Only include links from the same domain
                base_domain = urlparse(base_url).netloc
                link_domain = urlparse(absolute_url).netloc

                if base_domain == link_domain:
                    links.append(absolute_url)

            # Remove duplicates while preserving order
            unique_links = list(dict.fromkeys(links))

            self.logger.debug(
                f"Extracted {len(unique_links)} links from {base_url}",
                base_url=base_url,
                link_count=len(unique_links)
            )

            return unique_links

        except Exception as e:
            self.logger.error(
                f"Error extracting links from {base_url}: {str(e)}",
                base_url=base_url,
                error=str(e)
            )
            return []

    @log_performance("crawl_job")
    async def start_crawl_job(self, job: CrawlJob) -> CrawlResult:
        """
        Start a crawl job to process a documentation site.

        Args:
            job: CrawlJob object with target URL and options

        Returns:
            CrawlResult with statistics
        """
        self.logger.info(
            f"Starting crawl job {job.id} for {job.target_url}",
            job_id=str(job.id),
            target_url=job.target_url
        )

        # Publish event for job started
        self.event_publisher.publish_crawl_job_started(
            job_id=str(job.id),
            target_url=job.target_url,
            options=job.options
        )

        # Update job status
        await self.job_service.update_job_status(job.id, JobStatus.PROCESSING)

        result = CrawlResult()
        self.visited_urls = set()
        self.discovered_urls = {job.target_url}

        try:
            # Start with the target URL
            urls_to_crawl = [job.target_url]
            current_depth = 0

            while urls_to_crawl and current_depth <= job.options.get('max_depth', DEFAULT_MAX_DEPTH):
                self.logger.info(
                    f"Crawling at depth {current_depth}, {len(urls_to_crawl)} URLs to process",
                    depth=current_depth,
                    url_count=len(urls_to_crawl)
                )

                # Process current batch of URLs
                next_batch = []
                for url in urls_to_crawl:
                    if url in self.visited_urls:
                        continue

                    self.visited_urls.add(url)
                    result.total_pages += 1

                    # Crawl the page - this now returns chunks instead of raw content
                    chunks = await self.crawl_page(url, job_id=str(job.id))
                    if chunks is not None:  # Successfully crawled (even if no chunks were created)
                        result.processed_pages += 1
                        result.crawled_urls.append(url)

                        # Extract links from the original page content
                        # We need to fetch the page again to extract links since crawl_page now returns chunks
                        response = await self.fetch_with_retry(url)
                        if response and response.status_code == 200:
                            links = self.extract_links(response.text, url)
                            for link in links:
                                if link not in self.visited_urls and link not in self.discovered_urls:
                                    self.discovered_urls.add(link)
                                    next_batch.append(link)
                    else:
                        result.failed_pages += 1

                    # Update job progress
                    await self.job_service.update_job_progress(
                        job.id,
                        result.processed_pages,
                        result.failed_pages,
                        result.total_pages
                    )

                    # Publish progress event periodically
                    if result.total_pages % 10 == 0:  # Every 10 pages
                        progress_data = {
                            "processed_pages": result.processed_pages,
                            "failed_pages": result.failed_pages,
                            "total_pages": result.total_pages,
                            "current_depth": current_depth,
                            "discovered_urls_count": len(self.discovered_urls)
                        }
                        self.event_publisher.publish_crawl_job_progress(
                            job_id=str(job.id),
                            progress_data=progress_data
                        )

                urls_to_crawl = next_batch
                current_depth += 1

            # Mark job as completed
            await self.job_service.update_job_status(job.id, JobStatus.COMPLETED)
            await self.job_service.update_job_completion_time(job.id)

            # Publish event for job completed
            result_data = {
                "processed_pages": result.processed_pages,
                "failed_pages": result.failed_pages,
                "total_pages": result.total_pages,
                "crawled_urls": result.crawled_urls
            }
            self.event_publisher.publish_crawl_job_completed(
                job_id=str(job.id),
                result=result_data
            )

            self.logger.info(
                f"Crawl job {job.id} completed",
                job_id=str(job.id),
                processed=result.processed_pages,
                failed=result.failed_pages
            )

        except Exception as e:
            self.logger.error(
                f"Error in crawl job {job.id}: {str(e)}",
                job_id=str(job.id),
                error=str(e)
            )
            await self.job_service.update_job_status(job.id, JobStatus.FAILED)
            await self.job_service.update_job_error_log(job.id, str(e))

            # Publish event for job failed
            self.event_publisher.publish_crawl_job_failed(
                job_id=str(job.id),
                error_message=str(e),
                error_details={
                    "timestamp": datetime.utcnow().isoformat(),
                    "processed_pages": result.processed_pages,
                    "failed_pages": result.failed_pages
                }
            )

        return result

    @staticmethod
    async def process_crawl_job(job_data: Dict[str, Any]):
        """
        Static method to process a crawl job in a queue worker.

        Args:
            job_data: Dictionary containing job information
        """
        from src.services.job_service import CrawlJobService
        from src.services.event_publisher import EventPublisher

        crawl_service = CrawlerService()
        job_service = CrawlJobService()
        event_publisher = EventPublisher()

        crawl_job_id = job_data['crawl_job_id']
        target_url = job_data['target_url']
        options = job_data['options']

        try:
            # Get the job from the database
            job = await job_service.get_job_by_id(crawl_job_id)
            if not job:
                # Publish error event
                event_publisher.publish_error_event(
                    error_context="process_crawl_job",
                    error_message=f"Job {crawl_job_id} not found",
                    error_details={
                        "target_url": target_url,
                        "options": options
                    }
                )
                raise ValueError(f"Job {crawl_job_id} not found")

            # Update job with options
            job.options = options

            # Process the crawl
            result = await crawl_service.start_crawl_job(job)

            return {
                "job_id": crawl_job_id,
                "result": result.__dict__
            }
        except Exception as e:
            # Publish error event
            event_publisher.publish_error_event(
                error_context="process_crawl_job",
                error_message=str(e),
                error_details={
                    "crawl_job_id": crawl_job_id,
                    "target_url": target_url,
                    "options": options
                }
            )
            raise


    async def queue_page_for_processing(self, url: str, job_id: str, depth: int = 0) -> str:
        """
        Queue a page for processing using RQ message queues.

        Args:
            url: URL to process
            job_id: ID of the crawl job
            depth: Current depth of crawling

        Returns:
            Job ID of the queued task
        """
        from src.services.worker import create_default_worker_service

        # Prepare the job data
        page_data = {
            'url': url,
            'job_id': job_id,
            'depth': depth
        }

        # Get the worker service and queue
        worker_service = create_default_worker_service()
        job = worker_service.schedule_job(
            self.process_single_page,
            kwargs=page_data,
            queue_name="crawling",
            timeout="5m"  # 5 minute timeout for page processing
        )

        self.logger.info(
            f"Queued page {url} for processing in job {job_id}",
            url=url,
            job_id=job_id,
            queued_job_id=job.id,
            queue_name="crawling"
        )

        return job.id

    async def process_single_page(self, page_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Process a single page as part of a queued job.

        Args:
            page_data: Dictionary containing page information

        Returns:
            Dictionary with processing results
        """
        url = page_data['url']
        job_id = page_data['job_id']
        depth = page_data.get('depth', 0)

        self.logger.info(
            f"Processing page {url} from queue",
            url=url,
            job_id=job_id,
            depth=depth
        )

        # Crawl the page - this now returns chunks instead of raw content
        chunks = await self.crawl_page(url, job_id=job_id)

        result = {
            'url': url,
            'job_id': job_id,
            'depth': depth,
            'success': chunks is not None,
            'chunks_created': len(chunks) if chunks else 0,
            'links': []
        }

        if chunks is not None:
            # Extract links from the original page content
            # We need to fetch the page again to extract links since crawl_page now returns chunks
            response = await self.fetch_with_retry(url)
            if response and response.status_code == 200:
                links = self.extract_links(response.text, url)
                result['links'] = links

        self.logger.info(
            f"Completed processing page {url}",
            url=url,
            job_id=job_id,
            success=result['success'],
            chunks_created=result['chunks_created'],
            link_count=len(result['links'])
        )

        return result

    async def validate_selectors_on_site(self, test_url: str = "https://docs.anthropic.com/en/docs") -> Dict[str, Any]:
        """
        Validate the CSS selectors against a target documentation site.

        Args:
            test_url: URL to test the selectors against (default: Anthropic docs)

        Returns:
            Dictionary with validation results
        """
        self.logger.info(f"Validating CSS selectors against: {test_url}")

        validation_results = {
            "site_url": test_url,
            "content_selectors_tested": [],
            "navigation_selectors_tested": [],
            "results": {}
        }

        try:
            # Fetch the test page
            response = await self.fetch_with_retry(test_url)
            if not response or response.status_code != 200:
                self.logger.error(f"Could not fetch test page: {test_url}")
                return validation_results

            html_content = response.text
            soup = BeautifulSoup(html_content, 'html.parser')

            # Test content selectors
            for selector in self.content_selectors:
                elements_found = soup.select(selector)
                validation_results["content_selectors_tested"].append(selector)
                validation_results["results"][selector] = {
                    "type": "content",
                    "elements_found": len(elements_found),
                    "success": len(elements_found) > 0
                }
                if elements_found:
                    self.logger.debug(f"Content selector '{selector}' found {len(elements_found)} elements")

            # Test navigation selectors
            for selector in self.navigation_selectors:
                elements_found = soup.select(selector)
                validation_results["navigation_selectors_tested"].append(selector)
                validation_results["results"][selector] = {
                    "type": "navigation",
                    "elements_found": len(elements_found),
                    "success": len(elements_found) > 0
                }
                if elements_found:
                    self.logger.debug(f"Navigation selector '{selector}' found {len(elements_found)} elements")

            # Overall success is if at least one content selector works
            successful_content_selectors = [
                k for k, v in validation_results["results"].items()
                if v["type"] == "content" and v["success"]
            ]

            validation_results["overall_success"] = len(successful_content_selectors) > 0
            validation_results["successful_content_selectors"] = successful_content_selectors

            if validation_results["overall_success"]:
                self.logger.info(
                    f"Selector validation successful for {test_url}: {len(successful_content_selectors)} content selectors found elements"
                )
            else:
                self.logger.warning(
                    f"Selector validation partially failed for {test_url}: no content selectors found elements"
                )

            return validation_results

        except Exception as e:
            self.logger.error(f"Error validating selectors on {test_url}: {str(e)}")
            validation_results["error"] = str(e)
            return validation_results


# Convenience function to create a default crawler
def create_default_crawler() -> CrawlerService:
    """Create a default crawler service instance."""
    return CrawlerService()