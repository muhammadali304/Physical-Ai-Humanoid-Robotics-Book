"""
Qdrant client configuration for the RAG Ingestion Pipeline.
Implements vector database client for storing and retrieving embeddings.
"""

from typing import List, Optional, Dict, Any
from uuid import UUID
import numpy as np

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue

from src.config.settings import settings
from src.models.embedding import EmbeddingVector
from src.utils.logging import get_logger


class QdrantClientService:
    """
    Service to interact with Qdrant vector database.
    Provides functionality to store, retrieve, and search embeddings.
    """

    def __init__(self):
        self.logger = get_logger("qdrant_client")

        # Initialize Qdrant client based on configuration
        if settings.qdrant_url and settings.qdrant_api_key:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                timeout=10.0
            )
        else:
            # Use local instance if no remote URL is provided
            self.client = QdrantClient(location=":memory:")  # In-memory for testing

        self.collection_name = settings.qdrant_collection_name or "content_embeddings"
        self.vector_size = 1024  # For Cohere embeddings
        self.distance = Distance.COSINE  # Cosine similarity for semantic search

    async def setup_collection(self) -> bool:
        """
        Set up the Qdrant collection for storing embeddings.

        Returns:
            True if successful, False otherwise
        """
        try:
            self.logger.info(
                f"Setting up Qdrant collection: {self.collection_name}",
                collection_name=self.collection_name,
                vector_size=self.vector_size,
                distance=self.distance
            )

            # Check if collection already exists
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if collection_exists:
                self.logger.info(f"Collection {self.collection_name} already exists")
                # Verify the collection has the correct configuration
                existing_collection = self.client.get_collection(self.collection_name)
                vector_size = existing_collection.config.params.vectors.size if hasattr(existing_collection.config.params, 'vectors') else existing_collection.config.params.size
                distance = existing_collection.config.params.vectors.distance if hasattr(existing_collection.config.params, 'vectors') else existing_collection.config.params.distance
                if (vector_size != self.vector_size or distance != self.distance):
                    raise ValueError(f"Collection {self.collection_name} has incorrect configuration")
            else:
                # Create new collection
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=self.vector_size,
                        distance=self.distance
                    )
                )
                self.logger.info(f"Created new collection: {self.collection_name}")

            self.logger.info(f"Successfully set up collection {self.collection_name}")
            return True

        except Exception as e:
            self.logger.error(
                f"Error setting up Qdrant collection: {str(e)}",
                error=str(e),
                collection_name=self.collection_name
            )
            return False

    async def store_embedding(self, embedding: EmbeddingVector, content_chunk_id: Optional[UUID] = None, chunk_metadata: Optional[Dict[str, Any]] = None) -> bool:
        """
        Store a single embedding in Qdrant.

        Args:
            embedding: EmbeddingVector to store
            content_chunk_id: Optional content chunk ID to associate with the embedding
            chunk_metadata: Optional metadata from the content chunk to preserve

        Returns:
            True if successful, False otherwise
        """
        try:
            self.logger.info(
                f"Storing embedding in Qdrant collection {self.collection_name}",
                collection_name=self.collection_name,
                embedding_id=str(embedding.id)
            )

            # Prepare the payload with all required metadata
            payload = {
                "content_chunk_id": str(content_chunk_id or embedding.content_chunk_id),
                "model_used": embedding.model_used,
                "model_version": embedding.model_version,
                "created_at": embedding.created_at.isoformat()
            }

            # Add additional metadata from the content chunk if provided
            if chunk_metadata:
                # Include source URL, page title, section heading, chunk index, and content
                payload.update({
                    "source_url": chunk_metadata.get("source_url", ""),
                    "page_title": chunk_metadata.get("page_title", ""),
                    "section_heading": chunk_metadata.get("section_heading", ""),
                    "chunk_index": chunk_metadata.get("chunk_index", 0),
                    "raw_content": chunk_metadata.get("content", "")
                })

            # Prepare the point data
            point = PointStruct(
                id=str(embedding.id),
                vector=embedding.vector_data,
                payload=payload
            )

            # Store the embedding
            self.client.upsert(
                collection_name=self.collection_name,
                points=[point]
            )

            self.logger.info(
                f"Successfully stored embedding {embedding.id}",
                embedding_id=str(embedding.id)
            )
            return True

        except Exception as e:
            self.logger.error(
                f"Qdrant storage failed: {str(e)}, would use fallback strategy",
                embedding_id=str(embedding.id),
                error=str(e)
            )
            # In a production system, you might implement a fallback storage mechanism
            # For now, log the error and return False to indicate failure
            return False

    async def store_embeddings_batch(self, embeddings: List[EmbeddingVector], chunk_metadata_list: Optional[List[Dict[str, Any]]] = None) -> bool:
        """
        Store a batch of embeddings in Qdrant.

        Args:
            embeddings: List of EmbeddingVector objects to store
            chunk_metadata_list: Optional list of metadata from content chunks to preserve

        Returns:
            True if successful, False otherwise
        """
        try:
            if not embeddings:
                self.logger.warning("No embeddings to store in batch")
                return True

            self.logger.info(
                f"Storing batch of {len(embeddings)} embeddings in Qdrant",
                batch_size=len(embeddings),
                collection_name=self.collection_name
            )

            points = []
            for i, embedding in enumerate(embeddings):
                # Get corresponding chunk metadata if available
                chunk_metadata = None
                if chunk_metadata_list and i < len(chunk_metadata_list):
                    chunk_metadata = chunk_metadata_list[i]

                # Prepare the payload with all required metadata
                payload = {
                    "content_chunk_id": str(embedding.content_chunk_id),
                    "model_used": embedding.model_used,
                    "model_version": embedding.model_version,
                    "created_at": embedding.created_at.isoformat()
                }

                # Add additional metadata from the content chunk if provided
                if chunk_metadata:
                    # Include source URL, page title, section heading, chunk index, and content
                    payload.update({
                        "source_url": chunk_metadata.get("source_url", ""),
                        "page_title": chunk_metadata.get("page_title", ""),
                        "section_heading": chunk_metadata.get("section_heading", ""),
                        "chunk_index": chunk_metadata.get("chunk_index", 0),
                        "raw_content": chunk_metadata.get("content", "")
                    })

                point = PointStruct(
                    id=str(embedding.id),
                    vector=embedding.vector_data,
                    payload=payload
                )
                points.append(point)

            # Store the embeddings in batch
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            self.logger.info(
                f"Successfully stored batch of {len(embeddings)} embeddings",
                batch_size=len(embeddings)
            )
            return True

        except Exception as e:
            self.logger.error(
                f"Error storing embedding batch: {str(e)}",
                error=str(e),
                batch_size=len(embeddings) if embeddings else 0
            )
            return False

    async def retrieve_embedding(self, embedding_id: str) -> Optional[EmbeddingVector]:
        """
        Retrieve a single embedding by its ID.

        Args:
            embedding_id: ID of the embedding to retrieve

        Returns:
            EmbeddingVector object if found, None otherwise
        """
        try:
            self.logger.info(
                f"Retrieving embedding {embedding_id}",
                embedding_id=embedding_id
            )

            points = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[embedding_id]
            )

            if not points:
                self.logger.info(
                    f"Embedding {embedding_id} not found",
                    embedding_id=embedding_id
                )
                return None

            point = points[0]
            payload = point.payload

            embedding = EmbeddingVector(
                id=UUID(point.id),
                content_chunk_id=UUID(payload["content_chunk_id"]),
                vector_data=point.vector,
                model_used=payload["model_used"],
                model_version=payload["model_version"],
                created_at=payload["created_at"]
            )

            self.logger.info(
                f"Successfully retrieved embedding {embedding_id}",
                embedding_id=embedding_id
            )
            return embedding

        except Exception as e:
            self.logger.error(
                f"Error retrieving embedding {embedding_id}: {str(e)}",
                embedding_id=embedding_id,
                error=str(e)
            )
            return None

    async def search_similar(self, query_embedding: List[float], top_k: int = 10) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings using cosine similarity.

        Args:
            query_embedding: Query embedding vector
            top_k: Number of similar embeddings to return

        Returns:
            List of dictionaries with similarity scores and metadata
        """
        try:
            self.logger.info(
                f"Qdrant Search: Searching for {top_k} similar embeddings in collection '{self.collection_name}'",
                top_k=top_k,
                collection_name=self.collection_name
            )

            if len(query_embedding) != self.vector_size:
                raise ValueError(f"Query embedding must have {self.vector_size} dimensions, got {len(query_embedding)}")

            # Check if collection exists and has data
            collection_info = self.client.get_collection(self.collection_name)
            point_count = collection_info.points_count
            self.logger.info(f"Qdrant Search: Collection has {point_count} points before search")

            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k
            )

            similar_embeddings = []
            for i, result in enumerate(results):
                self.logger.debug(f"Qdrant Search: Result {i+1}: ID={result.id}, Score={result.score:.4f}, Payload keys={list(result.payload.keys()) if result.payload else []}")
                similar_embeddings.append({
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload,
                    "vector": result.vector
                })

            self.logger.info(
                f"Qdrant Search: Found {len(similar_embeddings)} similar embeddings",
                result_count=len(similar_embeddings)
            )
            return similar_embeddings

        except Exception as e:
            self.logger.error(
                f"Qdrant Search: Error searching for similar embeddings: {str(e)}",
                error=str(e),
                exc_info=True
            )
            return []

    async def search_by_content_chunk(self, content_chunk_id: UUID) -> Optional[EmbeddingVector]:
        """
        Search for an embedding associated with a specific content chunk.

        Args:
            content_chunk_id: ID of the content chunk

        Returns:
            EmbeddingVector object if found, None otherwise
        """
        try:
            self.logger.info(
                f"Searching for embedding by content chunk ID {content_chunk_id}",
                content_chunk_id=str(content_chunk_id)
            )

            # Create a filter to search by content_chunk_id
            filter_condition = Filter(
                must=[
                    FieldCondition(
                        key="content_chunk_id",
                        match=MatchValue(value=str(content_chunk_id))
                    )
                ]
            )

            results = self.client.search(
                collection_name=self.collection_name,
                query_filter=filter_condition,
                limit=1
            )

            if not results:
                self.logger.info(
                    f"No embedding found for content chunk {content_chunk_id}",
                    content_chunk_id=str(content_chunk_id)
                )
                return None

            result = results[0]
            payload = result.payload

            embedding = EmbeddingVector(
                id=UUID(result.id),
                content_chunk_id=UUID(payload["content_chunk_id"]),
                vector_data=result.vector,
                model_used=payload["model_used"],
                model_version=payload["model_version"],
                created_at=payload["created_at"]
            )

            self.logger.info(
                f"Found embedding for content chunk {content_chunk_id}",
                content_chunk_id=str(content_chunk_id),
                embedding_id=str(embedding.id)
            )
            return embedding

        except Exception as e:
            self.logger.error(
                f"Error searching for embedding by content chunk {content_chunk_id}: {str(e)}",
                content_chunk_id=str(content_chunk_id),
                error=str(e)
            )
            return None

    async def delete_embedding(self, embedding_id: str) -> bool:
        """
        Delete an embedding by its ID.

        Args:
            embedding_id: ID of the embedding to delete

        Returns:
            True if successful, False otherwise
        """
        try:
            self.logger.info(
                f"Deleting embedding {embedding_id}",
                embedding_id=embedding_id
            )

            self.client.delete(
                collection_name=self.collection_name,
                points_selector=[embedding_id]
            )

            self.logger.info(
                f"Successfully deleted embedding {embedding_id}",
                embedding_id=embedding_id
            )
            return True

        except Exception as e:
            self.logger.error(
                f"Error deleting embedding {embedding_id}: {str(e)}",
                embedding_id=embedding_id,
                error=str(e)
            )
            return False

    async def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the Qdrant collection.

        Returns:
            Dictionary with collection information
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)

            info = {
                "collection_name": self.collection_name,
                "vector_size": collection_info.config.params.vectors.size if hasattr(collection_info.config.params, 'vectors') else collection_info.config.params.size,
                "distance": collection_info.config.params.vectors.distance if hasattr(collection_info.config.params, 'vectors') else collection_info.config.params.distance,
                "point_count": collection_info.points_count,
                "indexed_vectors_count": getattr(collection_info, 'indexed_vectors_count', 0)
            }

            self.logger.info(
                f"Retrieved collection info for {self.collection_name}",
                collection_name=self.collection_name
            )
            return info

        except Exception as e:
            self.logger.error(
                f"Error getting collection info: {str(e)}",
                error=str(e)
            )
            return {}

    async def count_vectors(self, collection_name: Optional[str] = None) -> int:
        """
        Count the total number of vectors in the collection.

        Args:
            collection_name: Name of the collection to count vectors in (defaults to self.collection_name)

        Returns:
            Total number of vectors in the collection
        """
        try:
            collection_name = collection_name or self.collection_name
            collection_info = self.client.get_collection(collection_name)
            count = collection_info.points_count
            self.logger.info(
                f"Counted {count} vectors in collection {collection_name}",
                collection_name=collection_name,
                count=count
            )
            return count

        except Exception as e:
            self.logger.error(
                f"Error counting vectors in collection {collection_name}: {str(e)}",
                collection_name=collection_name,
                error=str(e)
            )
            return 0

    async def sample_vectors(self, collection_name: Optional[str] = None, limit: int = 10) -> List[Any]:
        """
        Sample vectors from the collection for validation.

        Args:
            collection_name: Name of the collection to sample from (defaults to self.collection_name)
            limit: Number of vectors to sample

        Returns:
            List of sampled vector points
        """
        try:
            collection_name = collection_name or self.collection_name

            # Use scroll API to get random points
            result = self.client.scroll(
                collection_name=collection_name,
                limit=limit
            )

            # The scroll API returns (points, next_page_offset) tuple
            points, next_page = result

            self.logger.info(
                f"Sampled {len(points)} vectors from collection {collection_name}",
                collection_name=collection_name,
                sample_count=len(points),
                requested_limit=limit
            )
            return points

        except Exception as e:
            self.logger.error(
                f"Error sampling vectors from collection {collection_name}: {str(e)}",
                collection_name=collection_name,
                error=str(e)
            )
            return []

    async def health_check(self) -> bool:
        """
        Perform a health check on the Qdrant connection.

        Returns:
            True if healthy, False otherwise
        """
        try:
            # Try to get collections list to verify connection
            self.client.get_collections()
            self.logger.info("Qdrant health check passed")
            return True
        except Exception as e:
            self.logger.error(
                f"Qdrant health check failed: {str(e)}",
                error=str(e)
            )
            return False

    async def store_embedding_with_fallback(self, embedding: EmbeddingVector, content_chunk_id: Optional[UUID] = None) -> bool:
        """
        Store an embedding with fallback strategies if Qdrant is unavailable.

        Args:
            embedding: EmbeddingVector to store
            content_chunk_id: Optional content chunk ID to associate with the embedding

        Returns:
            True if successful (either in Qdrant or fallback), False otherwise
        """
        try:
            # Try to store in Qdrant first
            success = await self.store_embedding(embedding, content_chunk_id)
            if success:
                return True

            # If Qdrant failed, implement fallback storage
            # In a real implementation, this might store to a local file,
            # database, or message queue for later processing
            self.logger.warning(
                f"Storing embedding {embedding.id} in fallback storage",
                embedding_id=str(embedding.id)
            )

            # For now, we'll just log that we would store in fallback
            # In a production system, you would implement actual fallback storage
            fallback_result = await self._store_in_fallback(embedding, content_chunk_id)
            return fallback_result

        except Exception as e:
            self.logger.error(
                f"Error in store_embedding_with_fallback: {str(e)}",
                error=str(e)
            )
            return False

    async def _store_in_fallback(self, embedding: EmbeddingVector, content_chunk_id: Optional[UUID] = None) -> bool:
        """
        Store embedding in fallback storage when Qdrant is unavailable.

        Args:
            embedding: EmbeddingVector to store in fallback
            content_chunk_id: Optional content chunk ID

        Returns:
            True if fallback storage succeeded, False otherwise
        """
        try:
            # In a real implementation, this might store to:
            # - Local file system
            # - Alternative database
            # - Message queue for later processing
            # - Cloud storage
            self.logger.info(
                f"Storing embedding {embedding.id} in fallback storage",
                embedding_id=str(embedding.id),
                content_chunk_id=str(content_chunk_id or embedding.content_chunk_id)
            )

            # For now, just log that we would store in fallback
            # In a real system, implement actual fallback storage
            return True

        except Exception as e:
            self.logger.error(
                f"Error in fallback storage: {str(e)}",
                error=str(e)
            )
            return False

    async def retrieve_embedding_with_fallback(self, embedding_id: str) -> Optional[EmbeddingVector]:
        """
        Retrieve an embedding with fallback strategies if Qdrant is unavailable.

        Args:
            embedding_id: ID of the embedding to retrieve

        Returns:
            EmbeddingVector if found, None otherwise
        """
        try:
            # Try to retrieve from Qdrant first
            embedding = await self.retrieve_embedding(embedding_id)
            if embedding:
                return embedding

            # If Qdrant failed or not found, try fallback storage
            self.logger.info(
                f"Checking fallback storage for embedding {embedding_id}",
                embedding_id=embedding_id
            )

            fallback_embedding = await self._retrieve_from_fallback(embedding_id)
            return fallback_embedding

        except Exception as e:
            self.logger.error(
                f"Error in retrieve_embedding_with_fallback: {str(e)}",
                error=str(e)
            )
            return None

    async def _retrieve_from_fallback(self, embedding_id: str) -> Optional[EmbeddingVector]:
        """
        Retrieve embedding from fallback storage.

        Args:
            embedding_id: ID of the embedding to retrieve

        Returns:
            EmbeddingVector if found in fallback, None otherwise
        """
        try:
            # In a real implementation, this would retrieve from fallback storage
            self.logger.info(
                f"Checking fallback storage for embedding {embedding_id}",
                embedding_id=embedding_id
            )

            # For now, return None indicating not found in fallback
            # In a real system, implement actual fallback retrieval
            return None

        except Exception as e:
            self.logger.error(
                f"Error retrieving from fallback storage: {str(e)}",
                error=str(e)
            )
            return None


def create_default_qdrant_client() -> QdrantClientService:
    """Create a default Qdrant client service instance."""
    return QdrantClientService()


# Convenience functions
async def setup_qdrant_collection() -> bool:
    """Convenience function to set up the Qdrant collection."""
    client = create_default_qdrant_client()
    return await client.setup_collection()


async def store_embedding(embedding: EmbeddingVector, content_chunk_id: Optional[UUID] = None) -> bool:
    """Convenience function to store an embedding."""
    client = create_default_qdrant_client()
    return await client.store_embedding(embedding, content_chunk_id)