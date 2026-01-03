#!/usr/bin/env python3
"""
Direct script to run the RAG ingestion pipeline for the Physical-AI book.
This bypasses the full dependency installation and runs the core functionality.
"""

import asyncio
import sys
import os
from pathlib import Path

# Add the src directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent / "src"))

from src.config.settings import settings
from src.services.crawler import CrawlerService
from src.services.job_service import CrawlJobService
from src.utils.logging import get_logger

async def main():
    target_url = "https://muhammadali304.github.io/Physical-Ai-Humanoid-Robotics-Book/"

    logger = get_logger("standalone_crawl")
    logger.info(f"Starting RAG ingestion pipeline for: {target_url}")

    # Initialize services
    crawler_service = CrawlerService()
    job_service = CrawlJobService()

    try:
        # Create and start the crawl job
        job = await job_service.create_job(
            target_url=target_url,
            max_depth=settings.max_depth,
            max_workers=settings.max_workers
        )

        logger.info(f"Created crawl job: {job.id}")

        # Start the crawling process
        result = await crawler_service.start_crawl_job(job)

        logger.info(f"Crawl completed. Processed {result.processed_pages} pages, "
                   f"failed {result.failed_pages} pages")

        # Verify data was stored in Qdrant
        qdrant_client = crawler_service.chunk_storage_service.qdrant_client
        collection_info = await qdrant_client.get_collection_info()
        logger.info(f"Collection '{collection_info['collection_name']}' now has {collection_info['point_count']} vectors")

        return 0

    except Exception as e:
        logger.error(f"Error during crawl: {str(e)}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)