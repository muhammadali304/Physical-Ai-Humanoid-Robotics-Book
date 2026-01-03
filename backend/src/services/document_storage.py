"""
Document storage service for the RAG Agent Backend.

This module provides async functionality for storing and retrieving document chunks
following the implementation plan requirements.
"""

import asyncio
from typing import List, Optional, Dict, Any
from src.services.qdrant_client import QdrantClientService
from src.models.document import DocumentChunk
from src.models.query import SourceReference
from src.config.settings import settings
import logging


class DocumentStorageService:
    """Service class for managing document storage and retrieval"""

    def __init__(self):
        """Initialize the document storage service with Qdrant client"""
        self.qdrant_client = QdrantClientService()  # Using the existing client
        self.logger = logging.getLogger(__name__)

    async def store_document_chunk(self, chunk: DocumentChunk, embedding_vector: List[float]) -> bool:
        """Store a document chunk with its embedding in the vector database"""
        try:
            # Convert the DocumentChunk to the format expected by the existing Qdrant client
            # First, we need to import EmbeddingVector from the existing models
            from src.models.embedding import EmbeddingVector
            from uuid import UUID

            # Create an EmbeddingVector from the chunk and embedding
            embedding = EmbeddingVector(
                content_chunk_id=UUID(chunk.id),
                vector_data=embedding_vector,
                model_used=f"gemini/{settings.vector_dimensions}",  # Using Gemini model info
                model_version="v1"
            )

            # Store using the existing Qdrant client's store_embedding method
            success = await self.qdrant_client.store_embedding(
                embedding=embedding,
                content_chunk_id=UUID(chunk.id)
            )

            if success:
                self.logger.info(f"Successfully stored document chunk: {chunk.id}")
            else:
                self.logger.error(f"Failed to store document chunk: {chunk.id}")

            return success
        except Exception as e:
            self.logger.error(f"Error storing document chunk {chunk.id}: {str(e)}")
            return False

    async def store_document_chunks(self, chunks: List[DocumentChunk], embeddings: List[List[float]]) -> bool:
        """Store multiple document chunks with their embeddings"""
        try:
            if len(chunks) != len(embeddings):
                raise ValueError("Number of chunks must match number of embeddings")

            success_count = 0
            for chunk, embedding in zip(chunks, embeddings):
                if await self.store_document_chunk(chunk, embedding):
                    success_count += 1

            success_rate = success_count / len(chunks) if chunks else 0
            self.logger.info(f"Stored {success_count}/{len(chunks)} document chunks (success rate: {success_rate:.2%})")

            return success_count == len(chunks)  # Return True only if all succeeded
        except Exception as e:
            self.logger.error(f"Error storing document chunks batch: {str(e)}")
            return False

    async def retrieve_similar_chunks(self, query_embedding: List[float], top_k: int = 5) -> List[SourceReference]:
        """Retrieve document chunks similar to the query embedding"""
        try:
            # Use the existing Qdrant client's search_similar method
            results = await self.qdrant_client.search_similar(query_embedding, top_k)

            source_references = []
            for result in results:
                # Convert the result to a SourceReference
                payload = result.get('payload', {})

                source_ref = SourceReference(
                    document_id=payload.get("source_url", "") or payload.get("document_id", ""),
                    title=payload.get("page_title", "") or payload.get("document_title", "Untitled"),
                    url=payload.get("source_url"),
                    page=payload.get("chunk_index"),
                    relevance_score=result.get('score', 0.0),
                    text_snippet=payload.get("raw_content", "")[:200] + "..." if len(payload.get("raw_content", "")) > 200 else payload.get("raw_content", "")
                )
                source_references.append(source_ref)

            self.logger.info(f"Retrieved {len(source_references)} similar document chunks")
            return source_references
        except Exception as e:
            self.logger.error(f"Error retrieving similar document chunks: {str(e)}")
            return []

    async def get_document_chunks(self, document_id: str) -> List[DocumentChunk]:
        """Retrieve all chunks associated with a specific document"""
        try:
            # This functionality might not be directly available in the existing client
            # We'll need to implement a search by document_id if possible
            # For now, return empty list as this requires specific implementation
            self.logger.info(f"Retrieving chunks for document: {document_id}")
            return []
        except Exception as e:
            self.logger.error(f"Error retrieving document chunks for {document_id}: {str(e)}")
            return []

    async def delete_document(self, document_id: str) -> bool:
        """Delete all chunks associated with a specific document"""
        try:
            # Use the existing Qdrant client's functionality to delete by document_id
            # This would require extending the existing client or implementing filtering
            self.logger.info(f"Deleting document: {document_id}")
            # For now, return True as placeholder
            return True
        except Exception as e:
            self.logger.error(f"Error deleting document {document_id}: {str(e)}")
            return False

    async def health_check(self) -> bool:
        """Check if the document storage service is healthy"""
        try:
            return await self.qdrant_client.health_check()
        except Exception:
            return False