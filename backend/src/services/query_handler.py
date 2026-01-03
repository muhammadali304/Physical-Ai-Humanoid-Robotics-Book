"""
Query handler service for the RAG Agent Backend.

This module provides the query processing logic with retrieval and generation
following the implementation plan requirements.
"""

import asyncio
from typing import List, Optional, Dict, Any
from pydantic import BaseModel
import logging
from src.config.settings import settings
from src.models.query import QueryRequest, QueryResponse, SourceReference, TokenUsage, RetrievalInfo
from src.services.rag_agent import RAGAgentService
from src.services.qdrant_client import QdrantClientService
from src.services.embedding_service import CohereEmbeddingService
from src.utils.errors import QueryProcessingError, RetrievalError, GenerationError


class QueryHandlerService:
    """Service class for handling query processing with retrieval and generation"""

    def __init__(self):
        """Initialize the query handler service"""
        self.rag_agent = RAGAgentService()
        self.qdrant_client = QdrantClientService()
        self.embedding_service = CohereEmbeddingService()
        self.logger = logging.getLogger(__name__)

    async def process_query(self, query_request: QueryRequest) -> QueryResponse:
        """Process a query request and return a response"""
        try:
            self.logger.info(f"Processing query: {query_request.query[:50]}...")

            # Validate the query request
            await self._validate_query_request(query_request)

            # Process the query using the RAG agent
            response = await self.rag_agent.process_query(query_request)

            self.logger.info("Query processed successfully")
            return response

        except QueryProcessingError:
            # Re-raise RAG-specific errors
            raise
        except Exception as e:
            self.logger.error(f"Error processing query: {str(e)}")
            raise QueryProcessingError(f"Failed to process query: {str(e)}")

    async def _validate_query_request(self, query_request: QueryRequest) -> bool:
        """Validate a query request before processing"""
        try:
            # Check query length
            if not query_request.query or len(query_request.query.strip()) == 0:
                raise QueryProcessingError("Query cannot be empty")

            # Check query length is within limits
            if len(query_request.query) > 1000:
                raise QueryProcessingError("Query exceeds maximum length of 1000 characters")

            # Validate session_id format if provided
            if query_request.session_id:
                # In a real implementation, you would validate the session ID format
                pass

            # Validate user_id format if provided
            if query_request.user_id:
                # In a real implementation, you would validate the user ID format
                pass

            self.logger.debug("Query request validation passed")
            return True

        except Exception as e:
            self.logger.error(f"Query validation failed: {str(e)}")
            raise

    async def process_query_with_retrieval(self, query_request: QueryRequest, top_k: int = 5) -> QueryResponse:
        """Process a query with explicit control over retrieval parameters"""
        try:
            self.logger.info(f"Processing query with retrieval (top_k={top_k}): {query_request.query[:50]}...")

            # Validate the query request
            await self._validate_query_request(query_request)

            # Generate embedding for the query
            query_embedding = await self.embedding_service.generate_embedding(query_request.query)
            if not query_embedding:
                raise RetrievalError("Failed to generate query embedding")

            # Retrieve relevant sources
            sources = await self._retrieve_sources(query_embedding, top_k)
            self.logger.info(f"Retrieved {len(sources)} sources")

            # Generate response with context
            response_text, token_usage = await self._generate_response_with_context(
                query_request.query,
                sources
            )

            # Create the response object
            # Convert token_usage to dict to avoid Pydantic v1/v2 compatibility issues
            token_usage_dict = None
            if token_usage:
                if hasattr(token_usage, "model_dump"):
                    token_usage_dict = token_usage.model_dump()
                elif hasattr(token_usage, "dict"):
                    token_usage_dict = token_usage.dict()

            response = QueryResponse(
                response=response_text,
                sources=sources,
                session_id=query_request.session_id,
                tokens_used=token_usage_dict,
                retrieval_info=RetrievalInfo(
                    retrieved_chunks=len(sources),
                    search_time_ms=0.0,  # Would need actual timing
                    top_k=top_k,
                    relevance_threshold=0.5
                )
            )

            self.logger.info("Query with retrieval processed successfully")
            return response

        except Exception as e:
            self.logger.error(f"Error processing query with retrieval: {str(e)}")
            raise QueryProcessingError(f"Failed to process query with retrieval: {str(e)}")

    async def _retrieve_sources(self, query_embedding: List[float], top_k: int) -> List[SourceReference]:
        """Retrieve relevant sources based on query embedding"""
        try:
            self.logger.debug(f"Retrieving sources with top_k={top_k}")

            # Use Qdrant client to search for similar embeddings
            search_results = await self.qdrant_client.search_similar(query_embedding, top_k)

            # Convert search results to SourceReference objects
            sources = []
            for result in search_results:
                payload = result.get('payload', {})

                source_ref = SourceReference(
                    document_id=payload.get("source_url", "") or payload.get("document_id", ""),
                    title=payload.get("page_title", "") or payload.get("document_title", "Untitled"),
                    url=payload.get("source_url"),
                    page=payload.get("chunk_index"),
                    relevance_score=result.get('score', 0.0),
                    text_snippet=payload.get("raw_content", "")[:200] + "..." if len(payload.get("raw_content", "")) > 200 else payload.get("raw_content", "")
                )
                sources.append(source_ref)

            self.logger.debug(f"Retrieved {len(sources)} sources")
            return sources

        except Exception as e:
            self.logger.error(f"Error retrieving sources: {str(e)}")
            raise RetrievalError(f"Failed to retrieve sources: {str(e)}")

    async def _generate_response_with_context(self, query: str, sources: List[SourceReference]) -> tuple[str, TokenUsage]:
        """Generate response using the RAG agent with context"""
        try:
            self.logger.debug(f"Generating response with {len(sources)} sources")

            # Use the RAG agent to generate response with context
            response_text, token_usage = await self.rag_agent.generate_response_with_context(query, sources)

            self.logger.debug(f"Generated response with {len(response_text)} characters")
            return response_text, token_usage

        except Exception as e:
            self.logger.error(f"Error generating response with context: {str(e)}")
            raise GenerationError(f"Failed to generate response: {str(e)}")

    async def process_batch_queries(self, queries: List[QueryRequest]) -> List[Optional[QueryResponse]]:
        """Process multiple queries in a batch"""
        try:
            self.logger.info(f"Processing batch of {len(queries)} queries")

            results = []
            for query_request in queries:
                try:
                    result = await self.process_query(query_request)
                    results.append(result)
                except Exception as e:
                    self.logger.error(f"Error processing query in batch: {str(e)}")
                    results.append(None)  # Add None for failed queries

            success_count = sum(1 for r in results if r is not None)
            self.logger.info(f"Successfully processed {success_count}/{len(queries)} queries in batch")

            return results

        except Exception as e:
            self.logger.error(f"Error processing batch queries: {str(e)}")
            raise QueryProcessingError(f"Failed to process batch queries: {str(e)}")

    async def get_query_statistics(self) -> Dict[str, Any]:
        """Get statistics about query processing"""
        try:
            stats = {
                "total_queries_processed": 0,  # Would need to track this in a real implementation
                "average_response_time_ms": 0.0,  # Would need to track this
                "success_rate": 1.0,  # Would need to track this
                "active_sessions": 0,  # Would need to track this
                "top_k_default": 5,  # Default value
                "max_query_length": 1000,  # Max length allowed
                "model_used": getattr(self.rag_agent, 'model', 'unknown')
            }

            self.logger.debug("Retrieved query statistics")
            return stats

        except Exception as e:
            self.logger.error(f"Error getting query statistics: {str(e)}")
            return {}

    async def health_check(self) -> bool:
        """Check if the query handler service is healthy"""
        try:
            # Test the RAG agent
            rag_healthy = await self.rag_agent.health_check()
            if not rag_healthy:
                return False

            # Test the Qdrant client
            qdrant_healthy = await self.qdrant_client.health_check()
            if not qdrant_healthy:
                return False

            # Test the embedding service
            test_embedding = await self.embedding_service.generate_embedding("test")
            embedding_healthy = test_embedding is not None

            overall_healthy = rag_healthy and qdrant_healthy and embedding_healthy
            self.logger.info(f"Query handler health check: {overall_healthy}")
            return overall_healthy

        except Exception as e:
            self.logger.error(f"Query handler health check failed: {str(e)}")
            return False


class QueryProcessingState:
    """Class to track the state of query processing"""

    def __init__(self):
        self.state = "initial"
        self.start_time = None
        self.end_time = None
        self.error = None
        self.result = None

    def set_state(self, state: str):
        """Set the current state of query processing"""
        self.state = state
        if state == "started":
            import time
            self.start_time = time.time()

    def set_result(self, result: Any):
        """Set the result of query processing"""
        self.result = result
        import time
        self.end_time = time.time()

    def set_error(self, error: Exception):
        """Set an error that occurred during query processing"""
        self.error = error
        import time
        self.end_time = time.time()

    def get_processing_time(self) -> float:
        """Get the total processing time in seconds"""
        if self.start_time and self.end_time:
            return self.end_time - self.start_time
        return 0.0