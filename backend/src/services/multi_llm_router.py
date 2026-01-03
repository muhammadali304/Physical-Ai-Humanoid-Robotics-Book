"""
Multi-LLM Router service for the RAG Agent Backend.

This module provides a router that can handle the Groq LLM provider.
"""
import asyncio
from typing import List, Optional, Dict, Any
from openai import AsyncOpenAI
import logging
from src.config.settings import settings
from src.models.query import QueryRequest, QueryResponse, SourceReference, TokenUsage, RetrievalInfo


class MultiLLMRouterService:
    """Service class that uses Groq API for LLM calls"""

    def __init__(self):
        """Initialize the router service with Groq client"""
        self.logger = logging.getLogger(__name__)

        # Initialize OpenAI client for Groq API
        self.groq_client = AsyncOpenAI(
            api_key=settings.groq_api_key,
            base_url="https://api.groq.com/openai/v1",
        )

        # Set default model
        self.model = "openai/gpt-oss-20b"

    async def route_query(self, query_request: QueryRequest, model_preference: Optional[str] = None) -> QueryResponse:
        """Route the query to Groq API"""
        try:
            return await self._call_groq(query_request)
        except Exception as e:
            self.logger.error(f"Error calling Groq API: {str(e)}")
            raise

    async def _call_groq(self, query_request: QueryRequest) -> QueryResponse:
        """Call the Groq API"""
        try:
            self.logger.info(f"Step 3a: Calling Groq API with query of length {len(query_request.query)} characters")
            self.logger.debug(f"Step 3a: Query content preview: {query_request.query[:200]}...")

            response = await self.groq_client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": query_request.query}],
                max_tokens=getattr(settings, 'default_max_tokens', 1024),
                temperature=getattr(settings, 'default_temperature', 0.7)
            )

            self.logger.info("Step 3b: Groq API call completed successfully")

            # Extract token usage from API response and ensure they are proper non-negative integers
            # For Groq API, ensure values are consistent with its reporting
            if hasattr(response.usage, 'prompt_tokens'):
                input_tokens = max(0, int(response.usage.prompt_tokens))
            else:
                input_tokens = max(0, len(query_request.query.split()) if query_request.query else 0)

            if hasattr(response.usage, 'completion_tokens'):
                output_tokens = max(0, int(response.usage.completion_tokens))
            else:
                output_tokens = max(0, len(response.choices[0].message.content.split()) if response.choices[0].message.content else 0)

            # Use the total_tokens as reported by the API to avoid validation issues
            # Some APIs include additional tokens (cache, system, etc.) in total that aren't just input+output
            if hasattr(response.usage, 'total_tokens'):
                total_tokens = max(0, int(response.usage.total_tokens))
            else:
                total_tokens = input_tokens + output_tokens

            # Create token usage object
            token_usage = {
                "input_tokens": input_tokens,
                "output_tokens": output_tokens,
                "total_tokens": total_tokens
            }

            self.logger.info(f"Step 3c: Response generated with {len(response.choices[0].message.content)} characters, tokens: input={input_tokens}, output={output_tokens}, total={total_tokens}")

            # Create a QueryResponse with token usage as dict to avoid Pydantic v1/v2 compatibility issues
            token_usage_dict = None
            if token_usage:
                if hasattr(token_usage, "model_dump"):
                    token_usage_dict = token_usage.model_dump()
                elif hasattr(token_usage, "dict"):
                    token_usage_dict = token_usage.dict()

            return QueryResponse(
                response=response.choices[0].message.content,
                sources=[],  # Now allowed to be empty after schema fix
                session_id=query_request.session_id,
                tokens_used=token_usage_dict  # Use token usage as dict to avoid validation issues
            )
        except Exception as e:
            self.logger.error(f"Error calling Groq API: {str(e)}", exc_info=True)
            raise