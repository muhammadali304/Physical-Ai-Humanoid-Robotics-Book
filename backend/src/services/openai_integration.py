"""
OpenAI integration service for the RAG Agent Backend with OpenAI Agents SDK using multiple LLM providers.

This module provides functionality for integrating with OpenAI-compatible APIs,
specifically configured to work with multiple LLM providers (Google's Gemini and Alibaba's Qwen) as specified in the requirements.
"""

import asyncio
from typing import List, Optional, Dict, Any, Union
from openai import AsyncOpenAI
from pydantic import BaseModel
import logging
from src.config.settings import settings
from src.models.query import QueryRequest, QueryResponse, SourceReference, TokenUsage
from src.services.rag_agent import RAGAgentService
from src.services.context_injector import ContextInjector
from src.utils.errors import QueryProcessingError
from src.services.multi_llm_router import MultiLLMRouterService


class OpenAIIntegrationService:
    """Service class for OpenAI-compatible API integration with multiple LLM providers"""

    def __init__(self):
        """Initialize the OpenAI integration service"""
        self.logger = logging.getLogger(__name__)

        # Initialize multi-LLM router
        self.llm_router = MultiLLMRouterService()

        self.model = getattr(settings, 'qwen_model', "qwen3-max")  # Default Qwen model
        self.default_temperature = 0.7
        self.default_max_tokens = 1024

        # Initialize related services
        self.rag_agent = RAGAgentService()
        self.context_injector = ContextInjector()

    async def process_query_with_openai(self,
                                      query_request: QueryRequest,
                                      temperature: Optional[float] = None,
                                      max_tokens: Optional[int] = None) -> QueryResponse:
        """
        Process a query using the OpenAI-compatible API with Gemini.

        Args:
            query_request: The query request containing the user's question
            temperature: Sampling temperature for response generation
            max_tokens: Maximum number of tokens to generate

        Returns:
            QueryResponse: The response containing the answer and source references
        """
        try:
            self.logger.info(f"Processing query with OpenAI integration: {query_request.query[:50]}...")

            # Use default values if not provided
            temperature = temperature or self.default_temperature
            max_tokens = max_tokens or self.default_max_tokens

            # Get context from RAG if available
            if hasattr(query_request, 'sources') and query_request.sources:
                # Use provided sources for context
                sources = query_request.sources
            else:
                # Use the RAG agent to retrieve context (this will use embeddings and Qdrant)
                query_embedding = await self.rag_agent.embedding_service.generate_embedding(query_request.query)
                if query_embedding:
                    sources = await self.rag_agent.retrieve_context(query_embedding, top_k=5)
                else:
                    sources = []

            # Inject context into the prompt
            if sources:
                prompt, citations = self.context_injector.inject_context_with_citations(
                    query_request.query,
                    sources
                )
            else:
                # No context available, create a simple prompt
                prompt = f"You are a helpful assistant. Question: {query_request.query}"
                citations = []

            # Create a temporary query request to pass to the router
            temp_query_request = QueryRequest(
                query=prompt,
                session_id=query_request.session_id
            )

            # Use the multi-LLM router to generate response
            # This will automatically choose the appropriate LLM based on available API keys
            result = await self.llm_router.route_query(temp_query_request)

            # Extract the response text
            response_text = result.response

            # Format response with citations if available
            if citations:
                response_text = self.context_injector.format_response_with_citations(response_text, citations)

            # Calculate token usage - since we're not getting usage from the router yet, we'll estimate
            input_tokens = len(prompt.split())
            output_tokens = len(response_text.split())
            total_tokens = input_tokens + output_tokens

            token_usage = {
                "input_tokens": input_tokens,
                "output_tokens": output_tokens,
                "total_tokens": total_tokens
            }


            # Create the response object
            # Convert token_usage to dict to avoid Pydantic v1/v2 compatibility issues
            token_usage_dict = None
            if token_usage:
                if hasattr(token_usage, "model_dump"):
                    token_usage_dict = token_usage.model_dump()
                elif hasattr(token_usage, "dict"):
                    token_usage_dict = token_usage.dict()

            query_response = QueryResponse(
                response=response_text,
                sources=sources,
                session_id=query_request.session_id,
                tokens_used=token_usage_dict
            )

            self.logger.info(f"Successfully processed query with OpenAI integration, response length: {len(response_text)}")
            return query_response

        except Exception as e:
            self.logger.error(f"Error processing query with OpenAI integration: {str(e)}")
            raise QueryProcessingError(f"Failed to process query with OpenAI integration: {str(e)}")

    async def generate_text(self,
                           prompt: str,
                           temperature: Optional[float] = None,
                           max_tokens: Optional[int] = None,
                           top_p: Optional[float] = None) -> str:
        """
        Generate text using the OpenAI-compatible API with Gemini.

        Args:
            prompt: The input prompt for text generation
            temperature: Sampling temperature for response generation
            max_tokens: Maximum number of tokens to generate
            top_p: Nucleus sampling parameter

        Returns:
            Generated text response
        """
        try:
            self.logger.info(f"Generating text with OpenAI integration: {prompt[:50]}...")

            # Use default values if not provided
            temperature = temperature or self.default_temperature
            max_tokens = max_tokens or self.default_max_tokens

            # Create a temporary query request to pass to the router
            temp_query_request = QueryRequest(
                query=prompt,
                session_id="temp_session"
            )

            # Use the multi-LLM router to generate response
            # This will automatically choose the appropriate LLM based on available API keys
            result = await self.llm_router.route_query(temp_query_request)

            # Extract the response text
            response_text = result.response

            self.logger.info(f"Successfully generated text, length: {len(response_text)}")
            return response_text

        except Exception as e:
            self.logger.error(f"Error generating text with OpenAI integration: {str(e)}")
            raise QueryProcessingError(f"Failed to generate text with OpenAI integration: {str(e)}")

    async def embed_text(self, text: str) -> List[float]:
        """
        Generate embeddings for text using the OpenAI-compatible API.
        Note: This would use an embedding endpoint if available, otherwise using existing embedding service.

        Args:
            text: The text to generate embeddings for

        Returns:
            List of embedding values
        """
        try:
            self.logger.info(f"Generating embeddings for text: {text[:50]}...")

            # For now, use the existing embedding service (Cohere) as Gemini may not have an OpenAI-compatible embedding API
            # In a real implementation with proper OpenAI embedding support, this would call the OpenAI embeddings API
            from src.services.embedding_service import CohereEmbeddingService
            embedding_service = CohereEmbeddingService()
            embedding = await embedding_service.generate_embedding(text)

            if embedding:
                self.logger.info(f"Successfully generated embedding with {len(embedding)} dimensions")
                return embedding
            else:
                self.logger.error("Failed to generate embedding")
                raise QueryProcessingError("Failed to generate embedding")

        except Exception as e:
            self.logger.error(f"Error generating embedding: {str(e)}")
            raise QueryProcessingError(f"Failed to generate embedding: {str(e)}")

    async def chat_completion(self,
                             messages: List[Dict[str, str]],
                             temperature: Optional[float] = None,
                             max_tokens: Optional[int] = None) -> str:
        """
        Perform a chat completion using the OpenAI-compatible API with Gemini.

        Args:
            messages: List of messages in the conversation (role and content)
            temperature: Sampling temperature for response generation
            max_tokens: Maximum number of tokens to generate

        Returns:
            Generated response text
        """
        try:
            self.logger.info(f"Performing chat completion with {len(messages)} messages")

            # Use default values if not provided
            temperature = temperature or self.default_temperature
            max_tokens = max_tokens or self.default_max_tokens

            # Convert messages to a single prompt for the router
            # In a real implementation, we might need to handle conversation history differently
            prompt_parts = []
            for msg in messages:
                role = msg.get("role", "user")
                content = msg.get("content", "")
                prompt_parts.append(f"{role.capitalize()}: {content}")

            full_prompt = "\n".join(prompt_parts)

            # Create a temporary query request to pass to the router
            temp_query_request = QueryRequest(
                query=full_prompt,
                session_id="temp_session"
            )

            # Use the multi-LLM router to generate response
            # This will automatically choose the appropriate LLM based on available API keys
            result = await self.llm_router.route_query(temp_query_request)

            # Extract the response text
            response_text = result.response

            self.logger.info(f"Successfully completed chat, response length: {len(response_text)}")
            return response_text

        except Exception as e:
            self.logger.error(f"Error in chat completion: {str(e)}")
            raise QueryProcessingError(f"Failed in chat completion: {str(e)}")

    async def validate_api_connection(self) -> bool:
        """
        Validate the connection to the OpenAI-compatible API.

        Returns:
            True if connection is valid, False otherwise
        """
        try:
            self.logger.info("Validating OpenAI API connection")

            # Create a temporary query request to test the router
            temp_query_request = QueryRequest(
                query="Hello",
                session_id="test_session"
            )

            # Test the API with a simple request through the router
            result = await self.llm_router.route_query(temp_query_request)

            is_valid = result is not None and result.response is not None and len(result.response) > 0
            self.logger.info(f"OpenAI API connection validation: {is_valid}")
            return is_valid

        except Exception as e:
            self.logger.error(f"OpenAI API connection validation failed: {str(e)}")
            return False

    async def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the configured model.

        Returns:
            Dictionary containing model information
        """
        try:
            self.logger.info(f"Getting model info for: {self.model}")

            # Get info about the current configuration
            model_info = {
                "model": self.model,
                "qwen_model": getattr(settings, 'qwen_model', "qwen3-max"),
                "gemini_model": getattr(settings, 'gemini_model', "gemini-3-flash-preview"),
                "mulerouter_base_url": getattr(settings, 'mulerouter_base_url', "https://api.mulerouter.ai/v1/"),
                "supports_chat": True,
                "supports_embeddings": False,  # Assuming embeddings are handled separately
                "default_temperature": self.default_temperature,
                "default_max_tokens": self.default_max_tokens
            }

            self.logger.info("Retrieved model information")
            return model_info

        except Exception as e:
            self.logger.error(f"Error getting model info: {str(e)}")
            return {
                "model": self.model,
                "error": str(e)
            }

    async def batch_process_queries(self,
                                  queries: List[QueryRequest],
                                  temperature: Optional[float] = None,
                                  max_tokens: Optional[int] = None) -> List[Optional[QueryResponse]]:
        """
        Process multiple queries in batch using the OpenAI-compatible API.

        Args:
            queries: List of query requests to process
            temperature: Sampling temperature for response generation
            max_tokens: Maximum number of tokens to generate

        Returns:
            List of QueryResponse objects (None for failed queries)
        """
        try:
            self.logger.info(f"Processing batch of {len(queries)} queries with OpenAI integration")

            results = []
            for query_request in queries:
                try:
                    result = await self.process_query_with_openai(
                        query_request,
                        temperature=temperature,
                        max_tokens=max_tokens
                    )
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

    async def health_check(self) -> bool:
        """Check if the OpenAI integration service is healthy"""
        try:
            # Test the API connection
            api_healthy = await self.validate_api_connection()
            if not api_healthy:
                return False

            # Test with a simple generation
            try:
                response = await self.generate_text("test", max_tokens=10)
                generation_healthy = response is not None and len(response) > 0
            except:
                generation_healthy = False

            overall_healthy = api_healthy and generation_healthy
            self.logger.info(f"OpenAI integration service health check: {overall_healthy}")

            return overall_healthy

        except Exception as e:
            self.logger.error(f"OpenAI integration service health check failed: {str(e)}")
            return False


# Global OpenAI integration service instance
openai_integration_service = OpenAIIntegrationService()


def get_openai_integration_service() -> OpenAIIntegrationService:
    """Get the global OpenAI integration service instance"""
    return openai_integration_service