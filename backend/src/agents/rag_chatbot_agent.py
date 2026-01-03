"""
RAG Chatbot Agent using OpenAI Agents SDK with multiple LLM providers.

This module implements a dedicated conversational chatbot agent that uses the OpenAI Agents SDK
configured to work with multiple LLM providers (Google's Gemini and Alibaba's Qwen) via OpenAI-compatible APIs, with
Retrieval-Augmented Generation powered by Qdrant.
"""

import asyncio
from typing import List, Optional, Dict, Any
from openai import AsyncOpenAI
from pydantic import BaseModel
import logging
from datetime import datetime
from src.config.settings import settings
from src.models.query import QueryRequest, QueryResponse, SourceReference, TokenUsage
from src.models.session import QueryRecord
from src.services.qdrant_client import QdrantClientService
from src.services.embedding_service import CohereEmbeddingService
from src.services.context_injector import ContextInjector
from src.utils.errors import QueryProcessingError
from src.services.multi_llm_router import MultiLLMRouterService


class ConversationContext(BaseModel):
    """Model to maintain conversation context for multi-turn interactions"""
    session_id: str
    messages: List[Dict[str, str]]
    last_accessed: datetime
    context_window: int = 10  # Number of messages to maintain in context


class RAGChatbotAgent:
    """Dedicated conversational chatbot agent using OpenAI Agents SDK with multiple LLM providers"""

    def __init__(self):
        """Initialize the RAG Chatbot Agent"""
        self.logger = logging.getLogger(__name__)

        # Initialize multi-LLM router
        self.llm_router = MultiLLMRouterService()

        self.model = getattr(settings, 'qwen_model', "qwen3-max")  # Default Qwen model
        self.default_temperature = 0.7
        self.default_max_tokens = 1024

        # Initialize related services
        self.qdrant_client = QdrantClientService()
        self.embedding_service = CohereEmbeddingService()
        self.context_injector = ContextInjector()

        # Maintain conversation sessions
        self.conversation_sessions: Dict[str, ConversationContext] = {}

    async def process_query(self,
                          query_request: QueryRequest,
                          temperature: Optional[float] = None,
                          max_tokens: Optional[int] = None) -> QueryResponse:
        """
        Process a query through the RAG chatbot agent.

        Args:
            query_request: The query request containing the user's question
            temperature: Sampling temperature for response generation
            max_tokens: Maximum number of tokens to generate

        Returns:
            QueryResponse: The response containing the answer and source references
        """
        try:
            self.logger.info(f"Processing query through RAG Chatbot Agent: {query_request.query[:50]}...")

            # Use default values if not provided
            temperature = temperature or self.default_temperature
            max_tokens = max_tokens or self.default_max_tokens

            # Step 1: Retrieve relevant context from Qdrant
            self.logger.info("Step 1: Starting context retrieval from Qdrant...")
            sources = await self._retrieve_context(query_request.query)
            self.logger.info(f"Step 1: Retrieved {len(sources)} sources from Qdrant")

            # Build conversation history if session exists
            conversation_history = await self._get_conversation_history(query_request.session_id)

            # Step 2: Inject context, selected text, and conversation history into the prompt
            self.logger.info("Step 2: Building agent prompt with retrieved context...")
            prompt = await self._build_agent_prompt(query_request.query, sources, conversation_history, query_request.selected_text)
            self.logger.info(f"Step 2: Agent prompt built with {len(sources)} sources and context")

            # Create a temporary query request to pass to the router
            temp_query_request = QueryRequest(
                query=prompt,
                session_id=query_request.session_id
            )

            # Step 3: Use the multi-LLM router to generate response
            self.logger.info("Step 3: Routing query to LLM via multi-LLM router...")
            # This will automatically choose the appropriate LLM based on available API keys
            result = await self.llm_router.route_query(temp_query_request)
            self.logger.info("Step 3: LLM response received successfully")

            # Extract the response text
            response_text = result.response

            # Format response for chatbot consumption
            formatted_response = await self._format_chatbot_response(response_text, sources)

            # Use the router's response but update with our context and session info
            # Create a fresh TokenUsage object to ensure proper validation
            token_usage_obj = None
            if result.tokens_used:
                token_usage_obj = {
                    "input_tokens": result.tokens_used["input_tokens"],
                    "output_tokens": result.tokens_used["output_tokens"],
                    "total_tokens": result.tokens_used["total_tokens"]
                }


            query_response = QueryResponse(
                response=formatted_response,
                sources=sources if sources else [],  # Ensure sources is an empty list if None
                session_id=query_request.session_id,
                tokens_used=token_usage_obj,  # Use fresh TokenUsage object to ensure validation
                retrieval_info=None  # Explicitly set to None if not available
            )

            # Update conversation history if session exists
            if query_request.session_id:
                await self._update_conversation_history(
                    query_request.session_id,
                    query_request.query,
                    formatted_response
                )

            self.logger.info(f"Successfully processed query through RAG Chatbot Agent, response length: {len(formatted_response)}")
            return query_response

        except Exception as e:
            self.logger.error(f"Error processing query through RAG Chatbot Agent: {str(e)}")
            raise QueryProcessingError(f"Failed to process query through RAG Chatbot Agent: {str(e)}")

    async def _retrieve_context(self, query: str) -> List[SourceReference]:
        """Retrieve relevant context from Qdrant based on the query"""
        try:
            self.logger.info(f"Step 1a: Retrieving context for query: {query[:50]}...")

            # Step 1a: Generate embedding for the query
            self.logger.info("Step 1b: Generating query embedding...")
            query_embedding = await self.embedding_service.generate_embedding(query)
            if not query_embedding:
                self.logger.warning("Step 1b: Failed to generate query embedding")
                return []
            else:
                self.logger.info(f"Step 1b: Successfully generated query embedding with {len(query_embedding.vector_data)} dimensions")

            # Step 1c: Search for similar embeddings in Qdrant
            self.logger.info("Step 1c: Searching for similar embeddings in Qdrant...")
            search_results = await self.qdrant_client.search_similar(query_embedding.vector_data, top_k=5)
            self.logger.info(f"Step 1c: Found {len(search_results)} similar embeddings in Qdrant")

            # Step 1d: Convert search results to SourceReference objects
            self.logger.info("Step 1d: Converting search results to SourceReference objects...")
            sources = []
            for i, result in enumerate(search_results):
                payload = result.get('payload', {})

                self.logger.debug(f"Processing search result {i+1}: score={result.get('score', 0.0):.4f}, payload keys={list(payload.keys())}")

                source_ref = SourceReference(
                    document_id=payload.get("document_id", ""),
                    title=payload.get("title", payload.get("page_title", "Untitled")),
                    url=payload.get("source_url"),
                    page=payload.get("page"),
                    relevance_score=result.get('score', 0.0),
                    text_snippet=payload.get("raw_content", "")[:200] + "..." if len(payload.get("raw_content", "")) > 200 else payload.get("raw_content", "")
                )
                sources.append(source_ref)

            self.logger.info(f"Step 1d: Converted {len(sources)} search results to SourceReference objects")
            self.logger.info(f"Step 1: Completed context retrieval - {len(sources)} sources retrieved")
            return sources

        except Exception as e:
            self.logger.error(f"Error retrieving context: {str(e)}", exc_info=True)
            return []

    async def _get_conversation_history(self, session_id: Optional[str]) -> List[Dict[str, str]]:
        """Get conversation history for multi-turn context"""
        if not session_id:
            return []

        try:
            if session_id in self.conversation_sessions:
                context = self.conversation_sessions[session_id]
                # Return the most recent messages up to the context window
                recent_messages = context.messages[-context.context_window:]
                return recent_messages
            else:
                # Create a new conversation context if it doesn't exist
                new_context = ConversationContext(
                    session_id=session_id,
                    messages=[],
                    last_accessed=datetime.now()
                )
                self.conversation_sessions[session_id] = new_context
                return []

        except Exception as e:
            self.logger.error(f"Error getting conversation history for session {session_id}: {str(e)}")
            return []

    async def _update_conversation_history(self, session_id: str, user_query: str, agent_response: str):
        """Update conversation history with the latest interaction"""
        try:
            if session_id not in self.conversation_sessions:
                # Create a new conversation context if it doesn't exist
                self.conversation_sessions[session_id] = ConversationContext(
                    session_id=session_id,
                    messages=[],
                    last_accessed=datetime.now()
                )

            context = self.conversation_sessions[session_id]

            # Add the user query and agent response to the conversation history
            context.messages.append({
                "role": "user",
                "content": user_query
            })
            context.messages.append({
                "role": "assistant",
                "content": agent_response
            })

            # Update the last accessed time
            context.last_accessed = datetime.now()

            # Trim the conversation history if it exceeds the context window
            if len(context.messages) > context.context_window * 2:  # *2 because we add both user and assistant messages
                context.messages = context.messages[-context.context_window * 2:]

            self.logger.debug(f"Updated conversation history for session {session_id}")

        except Exception as e:
            self.logger.error(f"Error updating conversation history for session {session_id}: {str(e)}")

    async def _build_agent_prompt(self, query: str, sources: List[SourceReference], conversation_history: List[Dict[str, str]], selected_text: Optional[str] = None) -> str:
        """Build the prompt for the agent with context, selected text, and conversation history"""
        try:
            self.logger.info(f"Step 2a: Building agent prompt with {len(sources)} sources, {len(conversation_history)} conversation history items, and selected text: {'yes' if selected_text else 'no'}")

            # Build context from sources
            context_text = ""
            if sources:
                context_text = "Relevant documentation context:\n"
                for i, source in enumerate(sources):
                    self.logger.debug(f"Step 2a: Adding source {i+1}: {source.title[:50]}... (score: {source.relevance_score:.4f})")
                    context_text += f"Source {i+1}: {source.title}\n"
                    context_text += f"Content: {source.text_snippet}\n\n"
            else:
                context_text = "No relevant documentation was found for this query.\n"
                self.logger.info("Step 2a: No sources found, using fallback message")

            # Add selected text context if provided
            if selected_text:
                context_text += f"Selected text from page: {selected_text}\n\n"

            # Add the user query
            prompt = f"{context_text}\nUser Query: {query}"

            self.logger.info(f"Step 2b: Final prompt length: {len(prompt)} characters")
            self.logger.debug(f"Step 2b: Final prompt preview: {prompt[:200]}...")

            self.logger.info(f"Step 2: Completed agent prompt building")
            return prompt

        except Exception as e:
            self.logger.error(f"Error building agent prompt: {str(e)}", exc_info=True)
            return f"User Query: {query}"

    async def _format_chatbot_response(self, response_text: str, sources: List[SourceReference]) -> str:
        """Format the response for frontend chatbot consumption"""
        try:
            # Add source citations to the response for frontend display
            formatted_response = response_text

            if sources:
                # Add a sources section to the response
                formatted_response += "\n\n📚 **Sources:**\n"
                for i, source in enumerate(sources):
                    # Create a formatted citation
                    citation = f"[{i+1}] [{source.title}]"
                    if source.url:
                        citation = f"[{i+1}] [{source.title}]({source.url})"
                    else:
                        citation = f"[{i+1}] {source.title}"

                    formatted_response += f"- {citation}\n"

            self.logger.debug(f"Formatted response for chatbot consumption with {len(sources)} sources")
            return formatted_response

        except Exception as e:
            self.logger.error(f"Error formatting chatbot response: {str(e)}")
            # Return the original response if formatting fails
            return response_text

    async def start_new_conversation(self, session_id: str) -> bool:
        """Start a new conversation session"""
        try:
            self.conversation_sessions[session_id] = ConversationContext(
                session_id=session_id,
                messages=[],
                last_accessed=datetime.now()
            )
            self.logger.info(f"Started new conversation session: {session_id}")
            return True

        except Exception as e:
            self.logger.error(f"Error starting new conversation session {session_id}: {str(e)}")
            return False

    async def clear_conversation_history(self, session_id: str) -> bool:
        """Clear conversation history for a session"""
        try:
            if session_id in self.conversation_sessions:
                self.conversation_sessions[session_id].messages = []
                self.conversation_sessions[session_id].last_accessed = datetime.now()
                self.logger.info(f"Cleared conversation history for session: {session_id}")
            return True

        except Exception as e:
            self.logger.error(f"Error clearing conversation history for session {session_id}: {str(e)}")
            return False

    async def get_conversation_stats(self, session_id: str) -> Dict[str, Any]:
        """Get statistics about a conversation session"""
        try:
            if session_id in self.conversation_sessions:
                context = self.conversation_sessions[session_id]
                stats = {
                    "session_id": session_id,
                    "message_count": len(context.messages),
                    "last_accessed": context.last_accessed.isoformat(),
                    "active": True
                }
            else:
                stats = {
                    "session_id": session_id,
                    "message_count": 0,
                    "last_accessed": None,
                    "active": False
                }

            self.logger.debug(f"Retrieved conversation stats for session {session_id}")
            return stats

        except Exception as e:
            self.logger.error(f"Error getting conversation stats for session {session_id}: {str(e)}")
            return {
                "session_id": session_id,
                "error": str(e),
                "active": False
            }

    async def validate_api_connection(self) -> bool:
        """Validate the connection to the OpenAI-compatible API"""
        try:
            self.logger.info("Validating OpenAI API connection for RAG Chatbot Agent")

            # Create a temporary query request to test the router
            temp_query_request = QueryRequest(
                query="Hello",
                session_id="test_session"
            )

            # Test the API with a simple request through the router
            result = await self.llm_router.route_query(temp_query_request)

            is_valid = result is not None and result.response is not None and len(result.response) > 0
            self.logger.info(f"OpenAI API connection validation for RAG Chatbot Agent: {is_valid}")
            return is_valid

        except Exception as e:
            self.logger.error(f"OpenAI API connection validation for RAG Chatbot Agent failed: {str(e)}")
            return False

    async def health_check(self) -> bool:
        """Check if the RAG Chatbot Agent service is healthy"""
        try:
            # Test the API connection
            api_healthy = await self.validate_api_connection()
            if not api_healthy:
                return False

            # Test Qdrant connection
            qdrant_healthy = await self.qdrant_client.health_check()
            if not qdrant_healthy:
                return False

            # Test embedding service
            test_embedding = await self.embedding_service.generate_embedding("test")
            embedding_healthy = test_embedding is not None

            overall_healthy = api_healthy and qdrant_healthy and embedding_healthy
            self.logger.info(f"RAG Chatbot Agent health check: {overall_healthy}")

            return overall_healthy

        except Exception as e:
            self.logger.error(f"RAG Chatbot Agent health check failed: {str(e)}")
            return False


# Global RAG Chatbot Agent instance
rag_chatbot_agent = RAGChatbotAgent()


def get_rag_chatbot_agent() -> RAGChatbotAgent:
    """Get the global RAG Chatbot Agent instance"""
    return rag_chatbot_agent