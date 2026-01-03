#!/usr/bin/env python3
"""
Script to run the RAG ingestion pipeline: crawl documentation, extract content,
chunk it, generate embeddings, and store in Qdrant.
"""

import asyncio
import argparse
from pathlib import Path
import sys
from typing import Optional

# Add the src directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from src.config.settings import settings
from src.services.crawler import CrawlerService
from src.services.job_service import CrawlJobService
from src.utils.logging import get_logger


async def main():
    parser = argparse.ArgumentParser(description="RAG Ingestion Pipeline")
    parser.add_argument("--url", required=True, help="Target URL to crawl")
    parser.add_argument("--max-depth", type=int, default=settings.max_depth,
                       help="Maximum depth to crawl")
    parser.add_argument("--workers", type=int, default=settings.max_workers,
                       help="Number of concurrent workers")
    parser.add_argument("--force", action="store_true",
                       help="Force re-crawl even if job exists")

    args = parser.parse_args()

    logger = get_logger("standalone_crawl")
    logger.info(f"Starting RAG ingestion pipeline for: {args.url}")

    # Initialize services
    crawler_service = CrawlerService()
    job_service = CrawlJobService()

    try:
        # Create and start the crawl job
        job = await job_service.create_job(
            target_url=args.url,
            max_depth=args.max_depth,
            max_workers=args.workers
        )

        logger.info(f"Created crawl job: {job.id}")

        # Start the crawling process
        result = await crawler_service.start_crawl_job(job)

        logger.info(f"Crawl completed. Processed {result.processed_pages} pages, "
                   f"failed {result.failed_pages} pages")

        return 0

    except Exception as e:
        logger.error(f"Error during crawl: {str(e)}")
        return 1


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)