#!/usr/bin/env python3
"""
Book Content Ingestion Script for Qdrant Vector Database

This script ingests book content from various sources (URLs or local files)
into a Qdrant vector database for RAG (Retrieval-Augmented Generation) purposes.

Features:
- Fetches content from URLs or reads from local files
- Splits content into meaningful chunks
- Generates embeddings using Cohere
- Stores embeddings in Qdrant with metadata
- Uses upsert to avoid duplicates
- Detailed logging at every step
"""

import asyncio
import logging
import os
import re
import uuid
from pathlib import Path
from typing import List, Dict, Any, Optional, Union
from urllib.parse import urlparse
from dotenv import load_dotenv

import requests
from bs4 import BeautifulSoup
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct
import cohere
from cohere import EmbedResponse

# Load environment variables from .env file
load_dotenv(os.path.join(os.path.dirname(__file__), 'backend', '.env'))

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class BookIngestionPipeline:
    """Main class for ingesting book content into Qdrant vector database"""

    def __init__(self,
                 qdrant_url: Optional[str] = None,
                 qdrant_api_key: Optional[str] = None,
                 cohere_api_key: Optional[str] = None,
                 collection_name: str = "book_embeddings"):

        # Use environment variables if not provided explicitly
        qdrant_url = qdrant_url or os.getenv("QDRANT_URL")
        qdrant_api_key = qdrant_api_key or os.getenv("QDRANT_API_KEY")
        cohere_api_key = cohere_api_key or os.getenv("COHERE_API_KEY")

        # Initialize Qdrant client
        if qdrant_url and qdrant_api_key:
            self.qdrant_client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                timeout=30.0
            )
            logger.info(f"Connected to Qdrant at {qdrant_url}")
        else:
            # Use local instance if no remote URL is provided
            self.qdrant_client = QdrantClient(location=":memory:")  # For testing
            logger.warning("Using in-memory Qdrant instance. Use remote Qdrant for production.")

        self.collection_name = collection_name
        self.vector_size = 1024  # For Cohere embeddings
        self.distance = Distance.COSINE

        # Initialize Cohere client
        if not cohere_api_key:
            raise ValueError("Cohere API key is required for embedding generation")

        self.cohere_client = cohere.AsyncClient(api_key=cohere_api_key)
        self.embedding_model = "embed-multilingual-v3.0"
        self.input_type = "search_document"

        # Collection setup will be handled by the setup method

    async def setup(self):
        """Async setup method to initialize the collection."""
        await self._setup_collection()

    async def _setup_collection(self):
        """Set up the Qdrant collection for storing embeddings."""
        try:
            logger.info(f"Setting up Qdrant collection: {self.collection_name}")

            # Check if collection already exists
            collections = self.qdrant_client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if collection_exists:
                logger.info(f"Collection {self.collection_name} already exists")
                # Verify the collection has the correct configuration
                existing_collection = self.qdrant_client.get_collection(self.collection_name)
                vector_size = existing_collection.config.params.vectors.size if hasattr(existing_collection.config.params, 'vectors') else existing_collection.config.params.size
                distance = existing_collection.config.params.vectors.distance if hasattr(existing_collection.config.params, 'vectors') else existing_collection.config.params.distance
                if (vector_size != self.vector_size or distance != self.distance):
                    raise ValueError(f"Collection {self.collection_name} has incorrect configuration")
            else:
                # Create new collection
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=self.vector_size,
                        distance=self.distance
                    )
                )
                logger.info(f"Created new collection: {self.collection_name}")

            logger.info(f"Successfully set up collection {self.collection_name}")

        except Exception as e:
            logger.error(f"Error setting up Qdrant collection: {str(e)}")
            raise

    async def fetch_content(self, source: Union[str, Path]) -> str:
        """Fetch content from URL or local file."""
        logger.info(f"Step 1: Loading content from source: {source}")

        try:
            # Check if source is a URL
            parsed = urlparse(str(source))
            if parsed.scheme in ['http', 'https']:
                logger.info(f"Fetching content from URL: {source}")
                response = requests.get(str(source))
                response.raise_for_status()

                # Try to parse as HTML first, then fallback to plain text
                try:
                    soup = BeautifulSoup(response.text, 'html.parser')
                    content = soup.get_text(separator=' ', strip=True)
                    logger.info(f"Successfully extracted {len(content)} characters from HTML")
                except Exception:
                    content = response.text
                    logger.info(f"Used raw text from URL, length: {len(content)} characters")

            else:
                # Local file
                file_path = Path(source)
                if not file_path.exists():
                    raise FileNotFoundError(f"File does not exist: {file_path}")

                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()

                logger.info(f"Successfully loaded {len(content)} characters from file: {file_path}")

            logger.info(f"Step 1: Content loading completed successfully")
            return content

        except Exception as e:
            logger.error(f"Error loading content from {source}: {str(e)}")
            raise

    def split_content(self, content: str, max_chunk_size: int = 1000, overlap: int = 100) -> List[Dict[str, Any]]:
        """Split content into meaningful chunks."""
        logger.info(f"Step 2: Splitting content into chunks (max size: {max_chunk_size}, overlap: {overlap})")

        try:
            # Split by paragraphs first
            paragraphs = [p.strip() for p in content.split('\n\n') if p.strip()]

            chunks = []
            current_chunk = ""
            chunk_index = 0

            for paragraph in paragraphs:
                # If adding this paragraph would exceed the max size, save the current chunk
                if len(current_chunk) + len(paragraph) > max_chunk_size and current_chunk:
                    # Save the current chunk
                    chunks.append({
                        'content': current_chunk.strip(),
                        'chunk_index': chunk_index,
                        'source_type': 'paragraph'
                    })
                    chunk_index += 1

                    # Start new chunk with overlap from the previous chunk
                    if overlap > 0:
                        words = current_chunk.split()
                        overlap_text = ' '.join(words[-min(len(words), overlap):])
                        current_chunk = overlap_text + " " + paragraph
                    else:
                        current_chunk = paragraph
                else:
                    current_chunk += "\n\n" + paragraph if current_chunk else paragraph

            # Add the last chunk if it has content
            if current_chunk.strip():
                chunks.append({
                    'content': current_chunk.strip(),
                    'chunk_index': chunk_index,
                    'source_type': 'paragraph'
                })

            # Further split any chunks that are still too large
            final_chunks = []
            for chunk in chunks:
                if len(chunk['content']) <= max_chunk_size:
                    final_chunks.append(chunk)
                else:
                    # Split by sentences for oversized chunks
                    sentences = re.split(r'[.!?]+', chunk['content'])
                    current_subchunk = ""

                    for sentence in sentences:
                        sentence = sentence.strip()
                        if not sentence:
                            continue

                        if len(current_subchunk) + len(sentence) > max_chunk_size and current_subchunk:
                            final_chunks.append({
                                'content': current_subchunk.strip(),
                                'chunk_index': len(final_chunks),
                                'source_type': 'sentence'
                            })
                            current_subchunk = sentence
                        else:
                            current_subchunk += " " + sentence if current_subchunk else sentence

                    # Add the last subchunk if it has content
                    if current_subchunk.strip():
                        final_chunks.append({
                            'content': current_subchunk.strip(),
                            'chunk_index': len(final_chunks),
                            'source_type': 'sentence'
                        })

            logger.info(f"Step 2: Created {len(final_chunks)} chunks from content")
            logger.info(f"Step 2: Chunking completed successfully")
            return final_chunks

        except Exception as e:
            logger.error(f"Error splitting content: {str(e)}")
            raise

    async def generate_embeddings(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Generate embeddings for each chunk."""
        logger.info(f"Step 3: Generating embeddings for {len(chunks)} chunks")

        try:
            embeddings = []

            # Process in batches of up to 96 (Cohere's limit)
            batch_size = 96
            for i in range(0, len(chunks), batch_size):
                batch = chunks[i:i + batch_size]
                batch_texts = [chunk['content'] for chunk in batch]

                logger.info(f"Processing batch {i//batch_size + 1}/{(len(chunks) - 1)//batch_size + 1} ({len(batch)} chunks)")

                try:
                    response: EmbedResponse = await self.cohere_client.embed(
                        texts=batch_texts,
                        model=self.embedding_model,
                        input_type=self.input_type
                    )

                    if not response.embeddings or len(response.embeddings) != len(batch):
                        raise ValueError(f"Expected {len(batch)} embeddings, got {len(response.embeddings) if response.embeddings else 0}")

                    for j, embedding_vector in enumerate(response.embeddings):
                        # Validate the embedding dimensions
                        if len(embedding_vector) != 1024:
                            raise ValueError(f"Expected 1024-dimensional embedding, got {len(embedding_vector)} dimensions for chunk {i+j}")

                        chunk_with_embedding = {**batch[j]}  # Copy chunk data
                        chunk_with_embedding['embedding'] = embedding_vector
                        chunk_with_embedding['embedding_id'] = str(uuid.uuid4())
                        embeddings.append(chunk_with_embedding)

                    logger.info(f"Successfully processed batch {i//batch_size + 1}, generated {len(response.embeddings)} embeddings")

                except Exception as batch_error:
                    logger.error(f"Error processing batch {i//batch_size + 1}: {str(batch_error)}")
                    # Try to process individually for failed batches
                    for j, chunk in enumerate(batch):
                        try:
                            single_response: EmbedResponse = await self.cohere_client.embed(
                                texts=[chunk['content']],
                                model=self.embedding_model,
                                input_type=self.input_type
                            )

                            if single_response.embeddings and len(single_response.embeddings[0]) == 1024:
                                chunk_with_embedding = {**chunk}
                                chunk_with_embedding['embedding'] = single_response.embeddings[0]
                                chunk_with_embedding['embedding_id'] = str(uuid.uuid4())
                                embeddings.append(chunk_with_embedding)
                                logger.debug(f"Successfully generated embedding for individual chunk {i+j}")
                            else:
                                logger.warning(f"Failed to generate embedding for chunk {i+j}, skipping")
                        except Exception as single_error:
                            logger.error(f"Failed to generate embedding for chunk {i+j}: {str(single_error)}")
                            continue

            logger.info(f"Step 3: Successfully generated embeddings for {len(embeddings)} out of {len(chunks)} chunks")
            logger.info(f"Step 3: Embedding generation completed successfully")
            return embeddings

        except Exception as e:
            logger.error(f"Error generating embeddings: {str(e)}")
            raise

    async def store_embeddings(self, embeddings: List[Dict[str, Any]], book_title: str, source: str) -> int:
        """Store embeddings in Qdrant with proper metadata."""
        logger.info(f"Step 4: Storing {len(embeddings)} embeddings in Qdrant collection '{self.collection_name}'")

        try:
            points = []
            successful_insertions = 0

            for i, item in enumerate(embeddings):
                try:
                    # Prepare the payload with all required metadata
                    payload = {
                        "content_chunk_id": item.get('embedding_id', str(uuid.uuid4())),
                        "book_title": book_title,
                        "source": source,
                        "chunk_index": item.get('chunk_index', i),
                        "raw_content": item['content'][:2000],  # Limit content size in payload
                        "source_type": item.get('source_type', 'unknown'),
                        "content_length": len(item['content'])
                    }

                    # Prepare the point data
                    point = PointStruct(
                        id=item['embedding_id'],
                        vector=item['embedding'],
                        payload=payload
                    )

                    points.append(point)
                    successful_insertions += 1

                    if len(points) >= 100:  # Batch upsert every 100 points
                        self.qdrant_client.upsert(
                            collection_name=self.collection_name,
                            points=points
                        )
                        logger.info(f"Upserted batch of {len(points)} points to Qdrant")
                        points = []

                except Exception as e:
                    logger.error(f"Error preparing chunk {i} for storage: {str(e)}")
                    continue

            # Upsert remaining points
            if points:
                self.qdrant_client.upsert(
                    collection_name=self.collection_name,
                    points=points
                )
                logger.info(f"Upserted final batch of {len(points)} points to Qdrant")

            logger.info(f"Step 4: Successfully stored {successful_insertions} embeddings in Qdrant")
            logger.info(f"Step 4: Qdrant insertion completed successfully")
            return successful_insertions

        except Exception as e:
            logger.error(f"Error storing embeddings in Qdrant: {str(e)}")
            raise

    async def ingest_book(self, source: Union[str, Path], book_title: str) -> Dict[str, Any]:
        """Main method to ingest a book from source to Qdrant."""
        logger.info(f"Starting book ingestion for: {book_title} from source: {source}")

        try:
            # Step 1: Load content
            content = await self.fetch_content(source)

            # Step 2: Split content into chunks
            chunks = self.split_content(content)

            # Step 3: Generate embeddings
            embeddings = await self.generate_embeddings(chunks)

            # Step 4: Store embeddings in Qdrant
            stored_count = await self.store_embeddings(embeddings, book_title, str(source))

            # Log summary
            summary = {
                "book_title": book_title,
                "source": str(source),
                "total_chunks_created": len(chunks),
                "embeddings_generated": len(embeddings),
                "embeddings_stored": stored_count,
                "status": "success"
            }

            logger.info(f"Book ingestion completed successfully!")
            logger.info(f"Summary: {summary}")

            return summary

        except Exception as e:
            logger.error(f"Error during book ingestion: {str(e)}")
            error_summary = {
                "book_title": book_title,
                "source": str(source),
                "error": str(e),
                "status": "error"
            }
            return error_summary


async def main():
    """Main function to demonstrate the ingestion pipeline."""
    # The API keys will be loaded from the .env file automatically
    # Configuration is handled by environment variables in the .env file

    # Initialize the ingestion pipeline - it will automatically load keys from .env
    try:
        ingestion_pipeline = BookIngestionPipeline()
        await ingestion_pipeline.setup()  # Initialize the collection asynchronously
        logger.info("Successfully initialized ingestion pipeline with API keys from .env file")
    except Exception as e:
        logger.error(f"Failed to initialize ingestion pipeline: {str(e)}")
        return

    # Example usage - replace with your book source and title
    source = "https://muhammadali304.github.io/Physical-Ai-Humanoid-Robotics-Book/"  # Physical AI & Humanoid Robotics
    book_title = "Physical AI & Humanoid Robotics"

    # Run the ingestion
    result = await ingestion_pipeline.ingest_book(source, book_title)

    print(f"\nIngestion Result: {result}")

    # You can also check the collection info
    try:
        collection_info = ingestion_pipeline.qdrant_client.get_collection(ingestion_pipeline.collection_name)
        print(f"\nCollection Info:")
        print(f"  Name: {collection_info.config.params.vectors.size if hasattr(collection_info.config.params, 'vectors') else collection_info.config.params.size}")
        print(f"  Point Count: {collection_info.points_count}")
    except Exception as e:
        logger.error(f"Error getting collection info: {str(e)}")


if __name__ == "__main__":
    # Example of how to use the script with different sources
    async def example_usage():
        # Initialize the ingestion pipeline - it will automatically load keys from .env
        try:
            ingestion_pipeline = BookIngestionPipeline()
            await ingestion_pipeline.setup()  # Initialize the collection asynchronously
            logger.info("Successfully initialized ingestion pipeline with API keys from .env file")
        except Exception as e:
            logger.error(f"Failed to initialize ingestion pipeline: {str(e)}")
            return

        # Example 1: Ingest from URL
        try:
            result1 = await ingestion_pipeline.ingest_book(
                source="https://muhammadali304.github.io/Physical-Ai-Humanoid-Robotics-Book/",  # Physical AI & Humanoid Robotics
                book_title="Physical AI & Humanoid Robotics"
            )
            print(f"URL ingestion result: {result1}")
        except Exception as e:
            logger.error(f"URL ingestion failed: {e}")

        # Example 2: Ingest from local file
        # Uncomment the following lines if you have a local book file
        # try:
        #     result2 = await ingestion_pipeline.ingest_book(
        #         source="./path/to/local/book.txt",
        #         book_title="Local Book Title"
        #     )
        #     print(f"Local file ingestion result: {result2}")
        # except Exception as e:
        #     logger.error(f"Local file ingestion failed: {e}")

    # Run the example
    asyncio.run(example_usage())