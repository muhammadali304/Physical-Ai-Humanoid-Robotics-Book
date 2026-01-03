"""
RAG Agent service for the RAG Agent Backend with OpenAI Agents SDK using multiple LLM providers.

This module provides the core RAG agent functionality for processing user queries
using multiple LLM providers (Gemini and Qwen) through OpenAI-compatible APIs,
with retrieval and generation capabilities.
"""

import asyncio
from typing import List, Optional, Dict, Any
from openai import AsyncOpenAI
from pydantic import BaseModel
import logging
from src.config.settings import settings
from src.models.query import QueryRequest, QueryResponse, SourceReference, TokenUsage, RetrievalInfo
from src.services.qdrant_client import QdrantClientService
from src.services.embedding_service import CohereEmbeddingService
from src.models.document import DocumentChunk
from src.services.multi_llm_router import MultiLLMRouterService


class RAGAgentService:
    """Service class for the RAG agent that processes queries using multiple LLM providers"""

    def __init__(self):
        """Initialize the RAG agent service with necessary clients"""
        # Initialize multi-LLM router
        self.llm_router = MultiLLMRouterService()

        # Use existing services for retrieval
        self.qdrant_client = QdrantClientService()
        self.embedding_service = CohereEmbeddingService()  # Using existing embedding service

        self.logger = logging.getLogger(__name__)

    async def process_query(self, query_request: QueryRequest) -> QueryResponse:
        """Process a user query and return a response with retrieved context"""
        try:
            self.logger.info(f"Processing query: {query_request.query[:50]}...")

            # Step 1: Generate embedding for the query
            query_embedding = await self.embedding_service.generate_embedding(query_request.query)
            if not query_embedding:
                raise Exception("Failed to generate query embedding")

            # Step 2: Retrieve relevant documents using the embedding
            retrieved_sources = await self.retrieve_context(query_embedding, top_k=5)

            # Step 3: Generate response using Gemini with retrieved context
            response_text, token_usage = await self.generate_response_with_context(
                query_request.query,
                retrieved_sources
            )

            # Step 4: Create and return the response
            # Convert token_usage to dict to avoid Pydantic v1/v2 compatibility issues
            token_usage_dict = None
            if token_usage:
                if hasattr(token_usage, "model_dump"):
                    token_usage_dict = token_usage.model_dump()
                elif hasattr(token_usage, "dict"):
                    token_usage_dict = token_usage.dict()

            response = QueryResponse(
                response=response_text,
                sources=retrieved_sources,
                session_id=query_request.session_id,
                tokens_used=token_usage_dict,
                retrieval_info=RetrievalInfo(
                    retrieved_chunks=len(retrieved_sources),
                    search_time_ms=0.0,  # Placeholder - would need actual timing
                    top_k=5,
                    relevance_threshold=0.5
                )
            )

            self.logger.info(f"Successfully processed query, response length: {len(response_text)}")
            return response

        except Exception as e:
            self.logger.error(f"Error processing query: {str(e)}")
            raise

    async def retrieve_context(self, query_embedding: List[float], top_k: int = 5) -> List[SourceReference]:
        """Retrieve relevant document chunks based on query embedding"""
        try:
            self.logger.info(f"Retrieving context with top_k={top_k}")

            # Use the existing Qdrant client to search for similar embeddings
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

            self.logger.info(f"Retrieved {len(sources)} context sources")
            return sources

        except Exception as e:
            self.logger.error(f"Error retrieving context: {str(e)}")
            return []

    async def generate_response_with_context(self, query: str, sources: List[SourceReference]) -> tuple[str, TokenUsage]:
        """Generate a response using the multi-LLM router with the provided context"""
        try:
            self.logger.info(f"Generating response with {len(sources)} sources")

            # Prepare the context from sources
            context_text = ""
            for i, source in enumerate(sources):
                context_text += f"Source {i+1}: {source.title}\n"
                context_text += f"Content: {source.text_snippet}\n\n"

            # Prepare the system message with instructions for using context
            system_message = """You are a helpful AI assistant that answers questions based on provided context.
            Use the provided context to answer the user's question accurately.
            If the context doesn't contain the information needed to answer the question, say so.
            Always cite the sources you used to answer the question."""

            # Prepare the user message with query and context
            user_message = f"Context:\n{context_text}\n\nQuestion: {query}"

            # Create a temporary query request to pass to the router
            from src.models.query import QueryRequest
            temp_query_request = QueryRequest(query=user_message)

            # Use the multi-LLM router to generate response
            # This will automatically choose the appropriate LLM based on available API keys
            result = await self.llm_router.route_query(temp_query_request)

            # Calculate token usage - since we're not getting usage from the router yet, we'll estimate
            response_text = result.response
            input_tokens = len(user_message.split())
            output_tokens = len(response_text.split())
            total_tokens = input_tokens + output_tokens

            token_usage = {
                "input_tokens": input_tokens,
                "output_tokens": output_tokens,
                "total_tokens": total_tokens
            }


            self.logger.info(f"Generated response with {len(response_text)} characters")
            return response_text, token_usage

        except Exception as e:
            self.logger.error(f"Error generating response with context: {str(e)}")
            # Return a default response in case of error
            return "I encountered an error while processing your request. Please try again.", {
                "input_tokens": 0,
                "output_tokens": 0,
                "total_tokens": 0
            }

    async def process_query_with_session(self, query_request: QueryRequest, session_id: str) -> QueryResponse:
        """Process a query within a session context (for multi-turn conversations)"""
        try:
            self.logger.info(f"Processing query in session: {session_id}")

            # For now, just call the regular process_query method
            # In a full implementation, this would maintain conversation context
            response = await self.process_query(query_request)

            # Update session with the new query-response pair
            # This would require session management implementation
            await self._update_session_history(session_id, query_request, response)

            return response

        except Exception as e:
            self.logger.error(f"Error processing query with session: {str(e)}")
            raise

    async def _update_session_history(self, session_id: str, query_request: QueryRequest, response: QueryResponse):
        """Update session history with the query-response pair"""
        # Placeholder implementation
        # In a real system, this would store the conversation in a database or cache
        self.logger.debug(f"Updating session history for session: {session_id}")

    async def health_check(self) -> bool:
        """Check if the RAG agent service is healthy"""
        try:
            # Test embedding generation
            test_embedding = await self.embedding_service.generate_embedding("test")
            if not test_embedding:
                return False

            # Test Qdrant connection
            qdrant_healthy = await self.qdrant_client.health_check()
            if not qdrant_healthy:
                return False

            # Test Gemini connection by making a simple request
            try:
                response = await self.gemini_client.chat.completions.create(
                    model=self.model,
                    messages=[{"role": "user", "content": "Hello"}],
                    max_tokens=10
                )
                gemini_healthy = response is not None
            except:
                gemini_healthy = False

            return test_embedding is not None and qdrant_healthy and gemini_healthy
        except Exception as e:
            self.logger.error(f"RAG agent health check failed: {str(e)}")
            return False


# For now, since the actual Gemini OpenAI-compatible API might not be available in the standard OpenAI SDK,
# we'll create a mock implementation that uses the existing Cohere service for embedding
# and a placeholder for the Gemini response generation
class MockRAGAgentService(RAGAgentService):
    """Mock implementation for testing when actual Gemini API is not available"""

    async def generate_response_with_context(self, query: str, sources: List[SourceReference]) -> tuple[str, TokenUsage]:
        """Mock response generation"""
        import random

        # Create a mock response based on the query and sources
        mock_responses = [
            f"I found information about '{query}' in the provided documents. ",
            f"Based on the context, here's what I know about '{query}': ",
            f"After reviewing the documents, I can tell you about '{query}'. "
        ]

        base_response = random.choice(mock_responses)

        # Add some information from the sources
        for source in sources[:2]:  # Use first 2 sources
            base_response += f"According to '{source.title}', {source.text_snippet} "

        base_response += "Let me know if you need more details."

        # Create mock token usage
        token_usage = {
            "input_tokens": len(query.split()),
            "output_tokens": len(base_response.split()),
            "total_tokens": len(query.split()) + len(base_response.split())
        }


        return base_response, token_usage