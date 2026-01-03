"""
Chunk storage service for the RAG Ingestion Pipeline.
Implements storage functionality for content chunks using Qdrant directly.
"""

from typing import List, Optional, Dict, Any
from datetime import datetime

from src.models.chunk import ContentChunk
from src.models.job import CrawlJob
from src.services.qdrant_client import QdrantClientService
from src.utils.logging import get_logger


class ChunkStorageService:
    """
    Service to store content chunks to Qdrant vector database.
    Bypasses repository pattern for direct Qdrant integration.
    """

    def __init__(self, qdrant_client: Optional[QdrantClientService] = None):
        self.logger = get_logger("chunk_storage")
        self.qdrant_client = qdrant_client or QdrantClientService()

    async def save_chunk(self, chunk: ContentChunk) -> str:
        """
        Save a single content chunk to storage.
        For now, this method will be a placeholder since chunks need embeddings to be stored in Qdrant.
        The actual storage happens in the embedding service after embedding generation.

        Args:
            chunk: ContentChunk object to save

        Returns:
            ID of the chunk (using the chunk's ID)
        """
        try:
            self.logger.info(
                f"Processing chunk {chunk.chunk_index} from {chunk.source_url}",
                chunk_index=chunk.chunk_index,
                source_url=chunk.source_url,
                token_count=chunk.token_count
            )

            # Validate the chunk before processing
            if not chunk.content.strip():
                raise ValueError(f"Chunk {chunk.chunk_index} has empty content")

            # Return the chunk's ID (or generate if not present)
            chunk_id = str(chunk.id)

            self.logger.info(
                f"Successfully processed chunk {chunk.chunk_index}",
                chunk_id=chunk_id,
                chunk_index=chunk.chunk_index
            )

            return chunk_id

        except Exception as e:
            self.logger.error(
                f"Error processing chunk {chunk.chunk_index}: {str(e)}",
                chunk_index=chunk.chunk_index,
                source_url=chunk.source_url,
                error=str(e)
            )
            raise

    async def save_chunks(self, chunks: List[ContentChunk], job_id: Optional[str] = None) -> List[str]:
        """
        Save multiple content chunks to storage.

        Args:
            chunks: List of ContentChunk objects to save
            job_id: Optional crawl job ID to associate with chunks

        Returns:
            List of IDs of the saved chunks
        """
        chunk_ids = []
        successful_saves = 0
        failed_saves = 0

        self.logger.info(
            f"Saving {len(chunks)} chunks for job {job_id}",
            total_chunks=len(chunks),
            job_id=job_id
        )

        for i, chunk in enumerate(chunks):
            try:
                # Add job_id to metadata if provided
                if job_id and chunk.metadata is None:
                    chunk.metadata = {"crawl_job_id": job_id}
                elif job_id and chunk.metadata:
                    chunk.metadata["crawl_job_id"] = job_id

                chunk_id = await self.save_chunk(chunk)
                chunk_ids.append(chunk_id)
                successful_saves += 1

            except Exception as e:
                failed_saves += 1
                self.logger.error(
                    f"Failed to save chunk {chunk.chunk_index}: {str(e)}",
                    chunk_index=chunk.chunk_index,
                    error=str(e)
                )
                # Continue with other chunks even if one fails

        self.logger.info(
            f"Chunk save operation completed: {successful_saves} successful, {failed_saves} failed",
            successful_saves=successful_saves,
            failed_saves=failed_saves,
            job_id=job_id
        )

        return chunk_ids

    async def get_chunk_by_id(self, chunk_id: str) -> Optional[ContentChunk]:
        """
        Retrieve a content chunk by its ID.
        This method is a placeholder - actual retrieval would happen via embedding search.

        Args:
            chunk_id: ID of the chunk to retrieve

        Returns:
            ContentChunk object if found, None otherwise
        """
        # This is a placeholder since chunks are stored as embeddings in Qdrant
        # and retrieval happens through semantic search
        self.logger.warning(f"Direct chunk retrieval by ID not implemented: {chunk_id}")
        return None

    async def get_chunks_by_source_url(self, source_url: str) -> List[ContentChunk]:
        """
        Retrieve all chunks associated with a specific source URL.
        This method is a placeholder - actual retrieval would happen via embedding search.

        Args:
            source_url: URL to filter chunks by

        Returns:
            List of ContentChunk objects
        """
        # This is a placeholder since chunks are stored as embeddings in Qdrant
        # and retrieval happens through semantic search
        self.logger.warning(f"Direct chunk retrieval by source URL not implemented: {source_url}")
        return []

    async def get_chunks_by_job_id(self, job_id: str) -> List[ContentChunk]:
        """
        Retrieve all chunks associated with a specific crawl job.
        This method is a placeholder - actual retrieval would happen via embedding search.

        Args:
            job_id: ID of the crawl job

        Returns:
            List of ContentChunk objects
        """
        # This is a placeholder since chunks are stored as embeddings in Qdrant
        # and retrieval happens through semantic search
        self.logger.warning(f"Direct chunk retrieval by job ID not implemented: {job_id}")
        return []

    async def validate_and_save_chunks(self, chunks: List[ContentChunk], job_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Validate chunks and save them to storage, returning validation results.

        Args:
            chunks: List of ContentChunk objects to validate and save
            job_id: Optional crawl job ID to associate with chunks

        Returns:
            Dictionary with results including saved chunk IDs and validation issues
        """
        from src.services.chunker import ChunkerService

        chunker_service = ChunkerService()
        saved_chunk_ids = []
        validation_results = {
            "total_chunks": len(chunks),
            "valid_chunks": 0,
            "invalid_chunks": 0,
            "saved_chunks": 0,
            "failed_chunks": 0,
            "issues": [],
            "chunk_validation_details": []
        }

        for i, chunk in enumerate(chunks):
            # Validate the chunk
            is_valid, issues = chunker_service.validate_chunk_quality(chunk)

            validation_detail = {
                "chunk_index": chunk.chunk_index,
                "is_valid": is_valid,
                "issues": issues,
                "token_count": chunk.token_count
            }
            validation_results["chunk_validation_details"].append(validation_detail)

            if is_valid:
                validation_results["valid_chunks"] += 1
            else:
                validation_results["invalid_chunks"] += 1
                validation_results["issues"].extend([f"Chunk {chunk.chunk_index}: {issue}" for issue in issues])

            # Save the chunk regardless of validation (to allow for later processing)
            try:
                chunk_id = await self.save_chunk(chunk)
                saved_chunk_ids.append(chunk_id)
                validation_results["saved_chunks"] += 1
            except Exception as e:
                validation_results["failed_chunks"] += 1
                validation_results["issues"].append(f"Failed to save chunk {chunk.chunk_index}: {str(e)}")

        validation_results["saved_chunk_ids"] = saved_chunk_ids

        self.logger.info(
            f"Validation and save completed: {validation_results['valid_chunks']} valid, {validation_results['invalid_chunks']} invalid",
            **validation_results
        )

        return validation_results

    async def update_chunk(self, chunk_id: str, updated_chunk: ContentChunk) -> bool:
        """
        Update an existing chunk in storage.
        This method is a placeholder - actual updates would happen via embedding service.

        Args:
            chunk_id: ID of the chunk to update
            updated_chunk: Updated ContentChunk object

        Returns:
            True if update was successful, False otherwise
        """
        # This is a placeholder since chunks are stored as embeddings in Qdrant
        # and updates happen through the embedding service
        self.logger.warning(f"Direct chunk update not implemented: {chunk_id}")
        return False

    async def delete_chunk(self, chunk_id: str) -> bool:
        """
        Delete a chunk from storage.
        This method is a placeholder - actual deletion would happen via Qdrant.

        Args:
            chunk_id: ID of the chunk to delete

        Returns:
            True if deletion was successful, False otherwise
        """
        # This is a placeholder since chunks are stored as embeddings in Qdrant
        # and deletion happens through the Qdrant client
        self.logger.warning(f"Direct chunk deletion not implemented: {chunk_id}")
        return False

    async def get_all_chunks(self, limit: Optional[int] = None, offset: int = 0) -> List[ContentChunk]:
        """
        Retrieve all chunks from storage with optional pagination.
        This method is a placeholder - actual retrieval would happen via embedding search.

        Args:
            limit: Maximum number of chunks to return
            offset: Number of chunks to skip

        Returns:
            List of ContentChunk objects
        """
        # This is a placeholder since chunks are stored as embeddings in Qdrant
        # and retrieval happens through semantic search
        self.logger.warning("Direct retrieval of all chunks not implemented")
        return []


def create_default_chunk_storage_service() -> ChunkStorageService:
    """Create a default chunk storage service instance."""
    return ChunkStorageService()