#!/usr/bin/env python3
"""
Script to validate that content was properly ingested into the RAG system.
"""

import argparse
from pathlib import Path
import sys
import asyncio

# Add the src directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from src.config.settings import settings
from src.services.qdrant_client import QdrantClientService


async def main():
    parser = argparse.ArgumentParser(description="Validate RAG Ingestion")
    parser.add_argument("--collection", default=None,
                       help="Qdrant collection to validate")
    parser.add_argument("--limit", type=int, default=10,
                       help="Number of records to sample")

    args = parser.parse_args()

    # Use default collection name if not specified
    collection_name = args.collection or settings.qdrant_collection_name
    print(f"Validating ingestion in collection: {collection_name}")

    # Initialize Qdrant client service
    qdrant_client = QdrantClientService()

    try:
        # Check if collection exists and get info
        collection_info = await qdrant_client.get_collection_info()
        print(f"Collection info: {collection_info}")

        # Count total vectors
        total_count = await qdrant_client.count_vectors()
        print(f"Total vectors in collection: {total_count}")

        if total_count > 0:
            # Sample some vectors to validate content
            sample_points = await qdrant_client.sample_vectors(
                limit=min(args.limit, total_count)
            )

            print(f"Sampled {len(sample_points)} vectors:")
            for i, point in enumerate(sample_points):
                payload = point.payload
                print(f"  {i+1}. URL: {payload.get('source_url', 'N/A')[:100]}...")
                print(f"     Page title: {payload.get('page_title', 'N/A')[:50]}...")
                print(f"     Content length: {len(payload.get('raw_content', ''))}")
                print(f"     Chunk index: {payload.get('chunk_index', 'N/A')}")
                print(f"     Section heading: {payload.get('section_heading', 'N/A')}")
                print()

            print("✓ Ingestion validation passed!")
            return 0
        else:
            print("✗ No vectors found in collection")
            return 1

    except Exception as e:
        print(f"✗ Validation failed: {str(e)}")
        return 1


if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)