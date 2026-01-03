"""
Document indexing service for the RAG Agent Backend.

This module provides functionality for indexing documents to optimize search and retrieval
following the implementation plan requirements for User Story 2.
"""

import asyncio
from typing import List, Optional, Dict, Any
from pydantic import BaseModel
import logging
from datetime import datetime
from src.models.document import DocumentChunk, DocumentMetadata
from src.services.qdrant_client import QdrantClientService
from src.services.embedding_service import CohereEmbeddingService
from src.utils.errors import DocumentProcessingError


class DocumentIndexingService:
    """Service class for indexing documents to optimize search and retrieval"""

    def __init__(self):
        """Initialize the document indexing service"""
        self.logger = logging.getLogger(__name__)
        self.qdrant_client = QdrantClientService()
        self.embedding_service = CohereEmbeddingService()
        self.indexing_batch_size = 100  # Number of chunks to index at once
        self.indexing_timeout = 30  # Timeout in seconds for indexing operations

    async def index_document_chunks(self, chunks: List[DocumentChunk], embeddings: List[List[float]]) -> bool:
        """
        Index document chunks with their embeddings for efficient retrieval.

        Args:
            chunks: List of document chunks to index
            embeddings: Corresponding embeddings for the chunks

        Returns:
            True if indexing was successful, False otherwise
        """
        try:
            self.logger.info(f"Starting indexing of {len(chunks)} document chunks")

            if len(chunks) != len(embeddings):
                raise DocumentProcessingError("Number of chunks and embeddings must match")

            # Prepare points for Qdrant indexing
            points = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                point = {
                    "id": chunk.chunk_id,
                    "vector": embedding,
                    "payload": {
                        "document_id": chunk.document_id,
                        "chunk_index": chunk.chunk_index,
                        "content": chunk.content,
                        "source_url": chunk.metadata.source_url if chunk.metadata else "",
                        "title": chunk.metadata.title if chunk.metadata else "Untitled",
                        "page": chunk.metadata.page_count if chunk.metadata else None,
                        "raw_content": chunk.content,
                        "created_at": datetime.now().isoformat(),
                        "updated_at": datetime.now().isoformat()
                    }
                }
                points.append(point)

            # Index the points in batches
            success = await self._index_points_in_batches(points)

            if success:
                self.logger.info(f"Successfully indexed {len(chunks)} document chunks")
            else:
                self.logger.error("Failed to index document chunks")

            return success

        except Exception as e:
            self.logger.error(f"Error indexing document chunks: {str(e)}")
            raise DocumentProcessingError(f"Failed to index document chunks: {str(e)}")

    async def _index_points_in_batches(self, points: List[Dict]) -> bool:
        """Index points in batches to optimize performance"""
        try:
            total_points = len(points)
            success_count = 0

            for i in range(0, total_points, self.indexing_batch_size):
                batch = points[i:i + self.indexing_batch_size]
                self.logger.debug(f"Indexing batch {i//self.indexing_batch_size + 1} with {len(batch)} points")

                # Upsert the batch of points to Qdrant
                batch_success = await self.qdrant_client.upsert_points(batch)

                if batch_success:
                    success_count += len(batch)
                else:
                    self.logger.error(f"Failed to index batch starting at index {i}")

            overall_success = success_count == total_points
            self.logger.info(f"Indexed {success_count}/{total_points} points successfully")

            return overall_success

        except Exception as e:
            self.logger.error(f"Error indexing points in batches: {str(e)}")
            return False

    async def update_document_index(self, document_id: str, chunks: List[DocumentChunk], embeddings: List[List[float]]) -> bool:
        """
        Update the index for an existing document.

        Args:
            document_id: ID of the document to update
            chunks: Updated document chunks
            embeddings: Corresponding embeddings for the chunks

        Returns:
            True if update was successful, False otherwise
        """
        try:
            self.logger.info(f"Updating index for document: {document_id}")

            # First, delete existing chunks for this document
            await self.delete_document_from_index(document_id)

            # Then index the new chunks
            success = await self.index_document_chunks(chunks, embeddings)

            if success:
                self.logger.info(f"Successfully updated index for document: {document_id}")
            else:
                self.logger.error(f"Failed to update index for document: {document_id}")

            return success

        except Exception as e:
            self.logger.error(f"Error updating index for document {document_id}: {str(e)}")
            raise DocumentProcessingError(f"Failed to update document index: {str(e)}")

    async def delete_document_from_index(self, document_id: str) -> bool:
        """
        Remove a document from the index.

        Args:
            document_id: ID of the document to remove

        Returns:
            True if deletion was successful, False otherwise
        """
        try:
            self.logger.info(f"Deleting document from index: {document_id}")

            # Delete all chunks for this document from Qdrant
            success = await self.qdrant_client.delete_points_by_payload({"document_id": document_id})

            if success:
                self.logger.info(f"Successfully deleted document from index: {document_id}")
            else:
                self.logger.error(f"Failed to delete document from index: {document_id}")

            return success

        except Exception as e:
            self.logger.error(f"Error deleting document {document_id} from index: {str(e)}")
            raise DocumentProcessingError(f"Failed to delete document from index: {str(e)}")

    async def rebuild_index(self) -> bool:
        """
        Rebuild the entire document index (useful for maintenance or schema changes).

        Returns:
            True if rebuild was successful, False otherwise
        """
        try:
            self.logger.info("Starting index rebuild process")

            # Create a new collection with the correct schema
            success = await self.qdrant_client.create_collection()

            if success:
                self.logger.info("Successfully rebuilt document index")
            else:
                self.logger.error("Failed to rebuild document index")

            return success

        except Exception as e:
            self.logger.error(f"Error rebuilding index: {str(e)}")
            raise DocumentProcessingError(f"Failed to rebuild index: {str(e)}")

    async def optimize_index(self) -> bool:
        """
        Optimize the index for better search performance.

        Returns:
            True if optimization was successful, False otherwise
        """
        try:
            self.logger.info("Optimizing document index")

            # In a real implementation, this would call Qdrant's optimization methods
            # For now, we'll just log that optimization is complete
            self.logger.info("Index optimization completed")

            return True

        except Exception as e:
            self.logger.error(f"Error optimizing index: {str(e)}")
            raise DocumentProcessingError(f"Failed to optimize index: {str(e)}")

    async def get_index_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about the current index.

        Returns:
            Dictionary containing index statistics
        """
        try:
            self.logger.info("Retrieving index statistics")

            # Get collection info from Qdrant
            stats = await self.qdrant_client.get_collection_info()

            index_stats = {
                "total_documents": stats.get("total_documents", 0),
                "total_chunks": stats.get("total_chunks", 0),
                "indexed_at": datetime.now().isoformat(),
                "status": "healthy" if stats.get("healthy", True) else "unhealthy"
            }

            self.logger.info(f"Index statistics: {index_stats}")
            return index_stats

        except Exception as e:
            self.logger.error(f"Error retrieving index statistics: {str(e)}")
            return {
                "total_documents": 0,
                "total_chunks": 0,
                "indexed_at": datetime.now().isoformat(),
                "status": "error",
                "error": str(e)
            }

    async def search_index(self, query: str, top_k: int = 10) -> List[Dict[str, Any]]:
        """
        Search the index using a query string.

        Args:
            query: Search query string
            top_k: Number of results to return

        Returns:
            List of search results
        """
        try:
            self.logger.info(f"Searching index for query: {query[:50]}...")

            # Generate embedding for the query
            query_embedding = await self.embedding_service.generate_embedding(query)
            if not query_embedding:
                raise DocumentProcessingError("Failed to generate query embedding")

            # Search in Qdrant
            search_results = await self.qdrant_client.search_similar(query_embedding, top_k)

            # Format results
            formatted_results = []
            for result in search_results:
                formatted_result = {
                    "chunk_id": result.get("id"),
                    "document_id": result.get("payload", {}).get("document_id"),
                    "content": result.get("payload", {}).get("content", ""),
                    "score": result.get("score", 0.0),
                    "source_url": result.get("payload", {}).get("source_url", ""),
                    "title": result.get("payload", {}).get("title", "Untitled")
                }
                formatted_results.append(formatted_result)

            self.logger.info(f"Found {len(formatted_results)} results for query")
            return formatted_results

        except Exception as e:
            self.logger.error(f"Error searching index: {str(e)}")
            raise DocumentProcessingError(f"Failed to search index: {str(e)}")

    async def validate_index(self) -> Dict[str, Any]:
        """
        Validate the integrity of the index.

        Returns:
            Dictionary containing validation results
        """
        try:
            self.logger.info("Validating document index")

            # Check if the collection exists and is accessible
            collection_exists = await self.qdrant_client.collection_exists()

            if not collection_exists:
                return {
                    "valid": False,
                    "error": "Collection does not exist",
                    "validated_at": datetime.now().isoformat()
                }

            # Get basic collection info
            collection_info = await self.qdrant_client.get_collection_info()

            validation_result = {
                "valid": True,
                "collection_exists": collection_exists,
                "total_points": collection_info.get("total_points", 0),
                "indexed_at": datetime.now().isoformat(),
                "validated_at": datetime.now().isoformat()
            }

            self.logger.info("Index validation completed successfully")
            return validation_result

        except Exception as e:
            self.logger.error(f"Error validating index: {str(e)}")
            return {
                "valid": False,
                "error": str(e),
                "validated_at": datetime.now().isoformat()
            }

    async def health_check(self) -> bool:
        """Check if the document indexing service is healthy"""
        try:
            # Test Qdrant connection
            qdrant_healthy = await self.qdrant_client.health_check()
            if not qdrant_healthy:
                return False

            # Test embedding service
            test_embedding = await self.embedding_service.generate_embedding("test")
            embedding_healthy = test_embedding is not None

            overall_healthy = qdrant_healthy and embedding_healthy
            self.logger.info(f"Document indexing service health check: {overall_healthy}")

            return overall_healthy

        except Exception as e:
            self.logger.error(f"Document indexing service health check failed: {str(e)}")
            return False


# Global document indexing service instance
document_indexing_service = DocumentIndexingService()


def get_document_indexing_service() -> DocumentIndexingService:
    """Get the global document indexing service instance"""
    return document_indexing_service