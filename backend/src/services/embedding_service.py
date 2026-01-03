"""
Cohere embedding service for the RAG Ingestion Pipeline.
Implements semantic embedding generation using Cohere models.
"""

import asyncio
import random
import hashlib
from typing import List, Optional, Dict, Any
from uuid import UUID

import cohere
from cohere import EmbedResponse

from src.models.chunk import ContentChunk
from src.models.embedding import EmbeddingVector
from src.config.settings import settings
from src.utils.logging import get_logger
from .qdrant_client import QdrantClientService


class CohereEmbeddingService:
    """
    Service to generate semantic embeddings using Cohere models.
    Provides functionality to create embeddings from content chunks.
    """

    def __init__(self):
        self.logger = get_logger("cohere_embedding")
        self.api_key = settings.cohere_api_key
        self.client = cohere.AsyncClient(api_key=self.api_key)
        self.model = "embed-multilingual-v3.0"  # Default Cohere model
        self.input_type = "search_document"  # Default input type for document embeddings
        self.qdrant_client = QdrantClientService()
        # Simple in-memory cache for embeddings (in production, use Redis or similar)
        self.embedding_cache = {}
        self.cache_ttl = 3600  # Cache TTL in seconds (1 hour)

    async def generate_embedding(self, text: str, content_chunk_id: Optional[UUID] = None) -> EmbeddingVector:
        """
        Generate a semantic embedding for a single text.

        Args:
            text: Text to generate embedding for
            content_chunk_id: Optional content chunk ID to associate with the embedding

        Returns:
            EmbeddingVector object with the generated embedding
        """
        # Create cache key based on text content and model
        cache_key = self._get_cache_key(text)

        # Check if embedding is in cache
        cached_embedding = await self._get_cached_embedding(cache_key)
        if cached_embedding:
            self.logger.info(
                f"Embedding Service: Retrieved embedding from cache for text of length {len(text)}",
                text_length=len(text),
                cache_key=cache_key
            )
            return cached_embedding

        try:
            self.logger.info(
                f"Embedding Service: Generating embedding for text of length {len(text)} using model '{self.model}'",
                text_length=len(text),
                model=self.model
            )

            response: EmbedResponse = await self.client.embed(
                texts=[text],
                model=self.model,
                input_type=self.input_type
            )

            if not response.embeddings or len(response.embeddings) == 0:
                raise ValueError("No embeddings returned from Cohere API")

            embedding_vector = response.embeddings[0]  # Get the first (and only) embedding

            # Validate the embedding dimensions
            if len(embedding_vector) != 1024:
                raise ValueError(f"Expected 1024-dimensional embedding, got {len(embedding_vector)} dimensions")

            embedding = EmbeddingVector(
                content_chunk_id=content_chunk_id or uuid4(),  # Use provided ID or generate new one
                vector_data=embedding_vector,
                model_used=f"cohere/{self.model}",
                model_version="v3"
            )

            self.logger.info(
                f"Embedding Service: Successfully generated embedding with {len(embedding_vector)} dimensions",
                dimensions=len(embedding_vector)
            )

            # Cache the embedding
            await self._cache_embedding(cache_key, embedding)

            return embedding

        except Exception as e:
            self.logger.error(
                f"Embedding Service: Cohere API failed: {str(e)}, using fallback strategy",
                error=str(e),
                exc_info=True
            )
            # Use fallback strategy
            embedding = await self._generate_fallback_embedding(text)

            # Cache the fallback embedding
            await self._cache_embedding(cache_key, embedding)

            return embedding

    def _get_cache_key(self, text: str) -> str:
        """
        Generate a cache key for the given text.

        Args:
            text: Text to generate cache key for

        Returns:
            Cache key string
        """
        # Create a hash of the text combined with the model info
        text_hash = hashlib.sha256(text.encode('utf-8')).hexdigest()
        cache_key = f"embedding_{text_hash}_{self.model}"
        return cache_key

    async def _get_cached_embedding(self, cache_key: str) -> Optional[EmbeddingVector]:
        """
        Retrieve an embedding from cache.

        Args:
            cache_key: Cache key to look up

        Returns:
            EmbeddingVector if found in cache, None otherwise
        """
        try:
            if cache_key in self.embedding_cache:
                cached_item = self.embedding_cache[cache_key]
                embedding, timestamp = cached_item

                # Check if cache entry is still valid (not expired)
                import time
                if time.time() - timestamp < self.cache_ttl:
                    return embedding
                else:
                    # Remove expired entry
                    del self.embedding_cache[cache_key]
                    return None
            return None
        except Exception as e:
            self.logger.error(
                f"Error retrieving from cache: {str(e)}",
                error=str(e)
            )
            return None

    async def _cache_embedding(self, cache_key: str, embedding: EmbeddingVector) -> bool:
        """
        Cache an embedding.

        Args:
            cache_key: Cache key to store under
            embedding: Embedding to cache

        Returns:
            True if caching succeeded, False otherwise
        """
        try:
            import time
            timestamp = time.time()
            self.embedding_cache[cache_key] = (embedding, timestamp)

            self.logger.debug(
                f"Cached embedding with key {cache_key}",
                cache_key=cache_key
            )
            return True
        except Exception as e:
            self.logger.error(
                f"Error caching embedding: {str(e)}",
                error=str(e)
            )
            return False

    async def clear_cache(self) -> bool:
        """
        Clear the embedding cache.

        Returns:
            True if successful, False otherwise
        """
        try:
            self.embedding_cache.clear()
            self.logger.info("Cleared embedding cache")
            return True
        except Exception as e:
            self.logger.error(
                f"Error clearing cache: {str(e)}",
                error=str(e)
            )
            return False

    async def get_cache_stats(self) -> Dict[str, Any]:
        """
        Get cache statistics.

        Returns:
            Dictionary with cache statistics
        """
        import time
        current_time = time.time()

        valid_entries = 0
        expired_entries = 0

        for key, (embedding, timestamp) in self.embedding_cache.items():
            if current_time - timestamp < self.cache_ttl:
                valid_entries += 1
            else:
                expired_entries += 1

        stats = {
            "total_entries": len(self.embedding_cache),
            "valid_entries": valid_entries,
            "expired_entries": expired_entries,
            "cache_size_mb": len(str(self.embedding_cache)) / (1024 * 1024),  # Rough estimate
            "ttl_seconds": self.cache_ttl
        }

        return stats

    async def _generate_fallback_embedding(self, text: str) -> EmbeddingVector:
        """
        Generate a fallback embedding when Cohere API is unavailable.
        This creates a simple embedding based on text characteristics.

        Args:
            text: Text to generate fallback embedding for

        Returns:
            EmbeddingVector object with the generated fallback embedding
        """
        try:
            self.logger.warning(
                f"Using fallback embedding generation for text of length {len(text)}",
                text_length=len(text)
            )

            # Generate a deterministic 1024-dimensional vector based on text content
            # This is a simple fallback that creates a vector based on text characteristics
            text_hash = hash(text) % (2**32)
            random.seed(text_hash)

            # Create a vector based on text characteristics
            vector = [0.0] * 1024

            # Use text characteristics to influence the vector
            for i in range(min(len(text), 1024)):
                vector[i] = float(ord(text[i]) % 128) / 64.0 - 1.0  # Normalize to [-1, 1]

            # Add some more sophisticated features
            word_count = len(text.split())
            char_count = len(text)
            sentence_count = max(len(text.split('.')), 1)

            # Spread these features across the vector
            for i in range(0, 100, 10):
                vector[i] = min(word_count / 100.0, 1.0)
            for i in range(1, 100, 10):
                vector[i] = min(char_count / 1000.0, 1.0)
            for i in range(2, 100, 10):
                vector[i] = min(sentence_count / 10.0, 1.0)

            # Normalize the vector
            magnitude = sum(v**2 for v in vector) ** 0.5
            if magnitude > 0:
                vector = [v / magnitude for v in vector]

            embedding = EmbeddingVector(
                vector_data=vector,
                model_used="fallback/text-characteristics-v1",
                model_version="v1"
            )

            self.logger.info(
                f"Successfully generated fallback embedding with {len(vector)} dimensions",
                dimensions=len(vector)
            )

            return embedding

        except Exception as e:
            self.logger.error(
                f"Error in fallback embedding generation: {str(e)}",
                error=str(e)
            )
            # As a last resort, return a random normalized vector
            vector = [random.uniform(-1, 1) for _ in range(1024)]
            # Normalize
            magnitude = sum(v**2 for v in vector) ** 0.5
            if magnitude > 0:
                vector = [v / magnitude for v in vector]
            else:
                vector = [0.0] * 1024

            return EmbeddingVector(
                vector_data=vector,
                model_used="fallback/random-v1",
                model_version="v1"
            )

    async def generate_embeddings_batch(self, texts: List[str], batch_size: int = 96) -> List[EmbeddingVector]:
        """
        Generate semantic embeddings for a batch of texts.

        Args:
            texts: List of texts to generate embeddings for
            batch_size: Number of texts to process in each batch (Cohere max is 96)

        Returns:
            List of EmbeddingVector objects
        """
        embeddings = []

        # Process in batches due to API limitations
        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            batch_embeddings = await self._generate_embedding_batch(batch)
            embeddings.extend(batch_embeddings)

        return embeddings

    async def _generate_embedding_batch(self, texts: List[str]) -> List[EmbeddingVector]:
        """
        Generate embeddings for a single batch of texts.

        Args:
            texts: List of texts (up to batch_size limit)

        Returns:
            List of EmbeddingVector objects
        """
        try:
            self.logger.info(
                f"Generating embeddings for batch of {len(texts)} texts",
                batch_size=len(texts)
            )

            response: EmbedResponse = await self.client.embed(
                texts=texts,
                model=self.model,
                input_type=self.input_type
            )

            if not response.embeddings or len(response.embeddings) != len(texts):
                raise ValueError(f"Expected {len(texts)} embeddings, got {len(response.embeddings)}")

            embedding_vectors = []

            for i, embedding_data in enumerate(response.embeddings):
                # Validate the embedding dimensions
                if len(embedding_data) != 1024:
                    raise ValueError(f"Expected 1024-dimensional embedding, got {len(embedding_data)} dimensions for text {i}")

                embedding = EmbeddingVector(
                    vector_data=embedding_data,
                    model_used=f"cohere/{self.model}",
                    model_version="v3"
                )
                embedding_vectors.append(embedding)

            self.logger.info(
                f"Successfully generated {len(embedding_vectors)} embeddings for batch",
                batch_size=len(embedding_vectors)
            )

            return embedding_vectors

        except Exception as e:
            self.logger.error(
                f"Cohere API batch failed: {str(e)}, using fallback strategy for each text",
                error=str(e)
            )
            # Fall back to generating embeddings one by one with fallback
            embedding_vectors = []
            for text in texts:
                try:
                    embedding = await self.generate_embedding(text)  # This will use fallback
                    embedding_vectors.append(embedding)
                except Exception as single_error:
                    self.logger.error(
                        f"Failed to generate embedding for text in batch: {str(single_error)}",
                        error=str(single_error)
                    )
                    # Skip this text and continue with others
                    continue

            return embedding_vectors

    async def generate_embedding_from_chunk(self, chunk: ContentChunk) -> EmbeddingVector:
        """
        Generate an embedding from a content chunk.

        Args:
            chunk: ContentChunk object to generate embedding for

        Returns:
            EmbeddingVector object with the generated embedding
        """
        try:
            self.logger.info(
                f"Generating embedding from content chunk {chunk.id}",
                chunk_id=str(chunk.id),
                chunk_length=len(chunk.content)
            )

            # Combine content with metadata for better embeddings
            combined_text = self._prepare_text_for_embedding(chunk)

            embedding = await self.generate_embedding(combined_text)

            # Associate the embedding with the chunk
            embedding.content_chunk_id = chunk.id

            self.logger.info(
                f"Successfully generated embedding for chunk {chunk.id}",
                chunk_id=str(chunk.id)
            )

            return embedding

        except Exception as e:
            self.logger.error(
                f"Error generating embedding from chunk {chunk.id}: {str(e)}",
                chunk_id=str(chunk.id),
                error=str(e)
            )
            raise

    def _prepare_text_for_embedding(self, chunk: ContentChunk) -> str:
        """
        Prepare text content for embedding generation by combining relevant metadata.

        Args:
            chunk: ContentChunk object

        Returns:
            Prepared text string for embedding
        """
        # Combine the content with relevant metadata for better embeddings
        parts = []

        if chunk.page_title:
            parts.append(f"Title: {chunk.page_title}")

        if chunk.section_heading:
            parts.append(f"Heading: {chunk.section_heading}")

        parts.append(f"Content: {chunk.content}")

        return " | ".join(parts)

    async def validate_embedding(self, embedding: EmbeddingVector) -> bool:
        """
        Validate an embedding vector.

        Args:
            embedding: EmbeddingVector to validate

        Returns:
            True if valid, False otherwise
        """
        try:
            # Check dimensions
            if len(embedding.vector_data) != 1024:
                self.logger.warning(
                    f"Invalid embedding dimensions: {len(embedding.vector_data)}, expected 1024",
                    dimensions=len(embedding.vector_data)
                )
                return False

            # Check for valid float values (not NaN or infinity)
            for value in embedding.vector_data:
                if not isinstance(value, (int, float)) or value != value:  # Check for NaN (NaN != NaN is True)
                    self.logger.warning("Embedding contains invalid values (NaN or non-numeric)")
                    return False

            # Check model identifier
            if not embedding.model_used.startswith("cohere/"):
                self.logger.warning(f"Invalid model identifier: {embedding.model_used}")
                return False

            self.logger.debug(
                f"Embedding validation passed for {embedding.id}",
                embedding_id=str(embedding.id)
            )
            return True

        except Exception as e:
            self.logger.error(
                f"Error validating embedding: {str(e)}",
                error=str(e)
            )
            return False

    async def store_embedding(self, embedding: EmbeddingVector, content_chunk_id: Optional[UUID] = None, chunk_metadata: Optional[Dict[str, Any]] = None) -> bool:
        """
        Store an embedding in Qdrant vector database.

        Args:
            embedding: EmbeddingVector to store
            content_chunk_id: Optional content chunk ID to associate with the embedding
            chunk_metadata: Optional metadata from the content chunk to preserve

        Returns:
            True if successful, False otherwise
        """
        try:
            self.logger.info(
                f"Storing embedding {embedding.id} in Qdrant",
                embedding_id=str(embedding.id),
                content_chunk_id=str(content_chunk_id or embedding.content_chunk_id)
            )

            success = await self.qdrant_client.store_embedding(embedding, content_chunk_id, chunk_metadata)

            if success:
                self.logger.info(
                    f"Successfully stored embedding {embedding.id}",
                    embedding_id=str(embedding.id)
                )
            else:
                self.logger.error(
                    f"Failed to store embedding {embedding.id}",
                    embedding_id=str(embedding.id)
                )

            return success

        except Exception as e:
            self.logger.error(
                f"Error storing embedding {embedding.id}: {str(e)}",
                embedding_id=str(embedding.id),
                error=str(e)
            )
            return False

    async def store_embeddings_batch(self, embeddings: List[EmbeddingVector], chunk_metadata_list: Optional[List[Dict[str, Any]]] = None) -> bool:
        """
        Store a batch of embeddings in Qdrant vector database.

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
                batch_size=len(embeddings)
            )

            success = await self.qdrant_client.store_embeddings_batch(embeddings, chunk_metadata_list)

            if success:
                self.logger.info(
                    f"Successfully stored batch of {len(embeddings)} embeddings",
                    batch_size=len(embeddings)
                )
            else:
                self.logger.error(
                    f"Failed to store batch of {len(embeddings)} embeddings",
                    batch_size=len(embeddings)
                )

            return success

        except Exception as e:
            self.logger.error(
                f"Error storing embedding batch: {str(e)}",
                error=str(e),
                batch_size=len(embeddings) if embeddings else 0
            )
            return False

    async def generate_and_store_embedding(self, text: str, content_chunk_id: Optional[UUID] = None, chunk_metadata: Optional[Dict[str, Any]] = None) -> Optional[EmbeddingVector]:
        """
        Generate an embedding and store it in Qdrant in one operation.

        Args:
            text: Text to generate embedding for
            content_chunk_id: Optional content chunk ID to associate with the embedding
            chunk_metadata: Optional metadata from the content chunk to preserve

        Returns:
            EmbeddingVector if successful, None otherwise
        """
        try:
            self.logger.info(
                f"Generating and storing embedding for text of length {len(text)}",
                text_length=len(text),
                content_chunk_id=str(content_chunk_id)
            )

            # Generate the embedding
            embedding = await self.generate_embedding(text, content_chunk_id)

            # Associate with content chunk if provided
            if content_chunk_id:
                embedding.content_chunk_id = content_chunk_id

            # Validate the embedding
            is_valid = await self.validate_embedding(embedding)
            if not is_valid:
                self.logger.error(
                    f"Generated embedding failed validation",
                    embedding_id=str(embedding.id)
                )
                return None

            # Store the embedding with metadata
            stored = await self.store_embedding(embedding, content_chunk_id, chunk_metadata)
            if not stored:
                self.logger.error(
                    f"Failed to store generated embedding",
                    embedding_id=str(embedding.id)
                )
                return None

            self.logger.info(
                f"Successfully generated and stored embedding {embedding.id}",
                embedding_id=str(embedding.id)
            )

            return embedding

        except Exception as e:
            self.logger.error(
                f"Error generating and storing embedding: {str(e)}",
                error=str(e)
            )
            return None

    async def generate_and_store_embeddings_from_chunks(self, chunks: List[ContentChunk]) -> List[EmbeddingVector]:
        """
        Generate embeddings from content chunks and store them in Qdrant.

        Args:
            chunks: List of ContentChunk objects to generate embeddings for

        Returns:
            List of successfully stored EmbeddingVector objects
        """
        stored_embeddings = []

        for chunk in chunks:
            try:
                self.logger.info(
                    f"Processing chunk {chunk.id} for embedding generation and storage",
                    chunk_id=str(chunk.id)
                )

                # Prepare chunk metadata for preservation
                chunk_metadata = {
                    "source_url": chunk.source_url,
                    "page_title": chunk.page_title,
                    "section_heading": chunk.section_heading,
                    "chunk_index": chunk.chunk_index,
                    "content": chunk.content
                }

                embedding = await self.generate_and_store_embedding(chunk.content, chunk.id, chunk_metadata)
                if embedding:
                    stored_embeddings.append(embedding)
                else:
                    self.logger.error(
                        f"Failed to generate and store embedding for chunk {chunk.id}",
                        chunk_id=str(chunk.id)
                    )

            except Exception as e:
                self.logger.error(
                    f"Error processing chunk {chunk.id}: {str(e)}",
                    chunk_id=str(chunk.id),
                    error=str(e)
                )
                continue  # Continue with other chunks even if one fails

        self.logger.info(
            f"Completed processing {len(chunks)} chunks, successfully stored {len(stored_embeddings)} embeddings",
            total_chunks=len(chunks),
            stored_embeddings=len(stored_embeddings)
        )

        return stored_embeddings

    async def retrieve_embedding(self, embedding_id: str) -> Optional[EmbeddingVector]:
        """
        Retrieve an embedding from Qdrant by its ID.

        Args:
            embedding_id: ID of the embedding to retrieve

        Returns:
            EmbeddingVector object if found, None otherwise
        """
        try:
            self.logger.info(
                f"Retrieving embedding {embedding_id} from Qdrant",
                embedding_id=embedding_id
            )

            embedding = await self.qdrant_client.retrieve_embedding(embedding_id)

            if embedding:
                self.logger.info(
                    f"Successfully retrieved embedding {embedding_id}",
                    embedding_id=embedding_id
                )
            else:
                self.logger.info(
                    f"Embedding {embedding_id} not found in Qdrant",
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

    async def search_similar_embeddings(self, query_text: str, top_k: int = 10) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings using semantic similarity.

        Args:
            query_text: Text to use as query for similarity search
            top_k: Number of similar embeddings to return

        Returns:
            List of dictionaries with similarity scores and metadata
        """
        try:
            self.logger.info(
                f"Searching for {top_k} similar embeddings to query text of length {len(query_text)}",
                query_length=len(query_text),
                top_k=top_k
            )

            # First, generate an embedding for the query text
            query_embedding = await self.generate_embedding(query_text)

            # Perform similarity search using Qdrant
            similar_results = await self.qdrant_client.search_similar(
                query_embedding.vector_data,
                top_k=top_k
            )

            self.logger.info(
                f"Found {len(similar_results)} similar embeddings",
                result_count=len(similar_results)
            )

            return similar_results

        except Exception as e:
            self.logger.error(
                f"Error searching for similar embeddings: {str(e)}",
                error=str(e)
            )
            return []

    async def search_similar_embeddings_by_chunk(self, chunk: ContentChunk, top_k: int = 10) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings to a content chunk.

        Args:
            chunk: ContentChunk to use as query for similarity search
            top_k: Number of similar embeddings to return

        Returns:
            List of dictionaries with similarity scores and metadata
        """
        try:
            self.logger.info(
                f"Searching for {top_k} similar embeddings to chunk {chunk.id}",
                chunk_id=str(chunk.id),
                top_k=top_k
            )

            # Prepare the query text from the chunk
            query_text = self._prepare_text_for_embedding(chunk)

            # Perform similarity search
            results = await self.search_similar_embeddings(query_text, top_k)

            self.logger.info(
                f"Found {len(results)} similar embeddings for chunk {chunk.id}",
                chunk_id=str(chunk.id),
                result_count=len(results)
            )

            return results

        except Exception as e:
            self.logger.error(
                f"Error searching for similar embeddings to chunk {chunk.id}: {str(e)}",
                chunk_id=str(chunk.id),
                error=str(e)
            )
            return []

    async def search_similar_embeddings_batch(self, query_texts: List[str], top_k: int = 10) -> List[List[Dict[str, Any]]]:
        """
        Search for similar embeddings for a batch of query texts.

        Args:
            query_texts: List of query texts for similarity search
            top_k: Number of similar embeddings to return for each query

        Returns:
            List of lists of dictionaries with similarity scores and metadata
        """
        all_results = []

        for query_text in query_texts:
            try:
                results = await self.search_similar_embeddings(query_text, top_k)
                all_results.append(results)
            except Exception as e:
                self.logger.error(
                    f"Error searching for similar embeddings for query: {str(e)}",
                    query_text=query_text,
                    error=str(e)
                )
                all_results.append([])  # Add empty list for failed queries

        self.logger.info(
            f"Completed batch similarity search for {len(query_texts)} queries",
            query_count=len(query_texts)
        )

        return all_results

    async def get_embedding_info(self) -> Dict[str, Any]:
        """
        Get information about the embedding service configuration.

        Returns:
            Dictionary with service configuration information
        """
        info = {
            "model": self.model,
            "model_version": "v3",
            "dimensions": 1024,
            "input_type": self.input_type,
            "max_batch_size": 96,
            "api_provider": "cohere",
            "qdrant_collection": self.qdrant_client.collection_name
        }

        self.logger.info("Retrieved embedding service information")
        return info


def create_default_embedding_service() -> CohereEmbeddingService:
    """Create a default Cohere embedding service instance."""
    return CohereEmbeddingService()


# Convenience functions
async def generate_embedding(text: str) -> EmbeddingVector:
    """Convenience function to generate an embedding for text."""
    service = create_default_embedding_service()
    return await service.generate_embedding(text)


async def generate_embeddings_from_chunks(chunks: List[ContentChunk]) -> List[EmbeddingVector]:
    """Convenience function to generate embeddings from content chunks."""
    service = create_default_embedding_service()
    embeddings = []

    for chunk in chunks:
        embedding = await service.generate_embedding_from_chunk(chunk)
        embeddings.append(embedding)

    return embeddings