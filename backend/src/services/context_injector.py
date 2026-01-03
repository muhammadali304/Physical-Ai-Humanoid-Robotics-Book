"""
Context injection service for the RAG Agent Backend.

This module provides functionality for injecting retrieved context into prompts
to ensure grounded responses from the Gemini model.
"""

import asyncio
from typing import List, Optional, Dict, Any
from pydantic import BaseModel
import logging
from src.models.query import SourceReference
from src.models.document import DocumentChunk


class ContextInjector:
    """Service class for injecting context into prompts for grounded responses"""

    def __init__(self):
        """Initialize the context injection service"""
        self.logger = logging.getLogger(__name__)
        self.max_context_length = 4000  # Max context length in tokens (approximately)
        self.citation_format = "numbered"  # Options: "numbered", "url", "title"

    def inject_context(self,
                     query: str,
                     sources: List[SourceReference],
                     system_instructions: Optional[str] = None) -> str:
        """
        Inject retrieved context into the prompt for grounded responses.

        Args:
            query: The original user query
            sources: List of sources to inject into the prompt
            system_instructions: Optional system instructions to include

        Returns:
            Formatted prompt with injected context
        """
        try:
            self.logger.info(f"Injecting context from {len(sources)} sources")

            # Build the system message
            if system_instructions:
                system_msg = system_instructions
            else:
                system_msg = ("You are a helpful AI assistant that answers questions based on provided context. "
                             "Use the provided context to answer the user's question accurately. "
                             "Always cite the sources you used to answer the question. "
                             "If the context doesn't contain the information needed to answer the question, say so.")

            # Build the context from sources
            context_text = self._build_context_text(sources)

            # Create the user message with query and context
            user_message = f"Context:\n{context_text}\n\nQuestion: {query}"

            # Format the complete prompt
            formatted_prompt = f"{system_msg}\n\n{user_message}"

            self.logger.info("Context injection completed successfully")
            return formatted_prompt

        except Exception as e:
            self.logger.error(f"Error injecting context: {str(e)}")
            # Return a minimal prompt in case of error
            return f"You are a helpful assistant. Question: {query}"

    def _build_context_text(self, sources: List[SourceReference]) -> str:
        """Build context text from sources with proper formatting"""
        try:
            context_parts = []

            for i, source in enumerate(sources):
                # Format the source based on the citation format
                if self.citation_format == "numbered":
                    source_header = f"[{i+1}] {source.title}"
                elif self.citation_format == "url":
                    source_header = f"{source.url or f'Source {i+1}'}"
                else:  # title format
                    source_header = f"{source.title}"

                # Add the source information
                source_text = f"Source: {source_header}\n"
                source_text += f"Content: {source.text_snippet}\n"

                # Add page information if available
                if source.page is not None:
                    source_text += f"Page: {source.page}\n"

                # Add relevance score if available
                if source.relevance_score is not None:
                    source_text += f"Relevance: {source.relevance_score:.2f}\n"

                source_text += "\n"
                context_parts.append(source_text)

            context_text = "".join(context_parts)
            self.logger.debug(f"Built context text from {len(sources)} sources")
            return context_text

        except Exception as e:
            self.logger.error(f"Error building context text: {str(e)}")
            return ""

    def inject_context_with_citations(self,
                                    query: str,
                                    sources: List[SourceReference],
                                    system_instructions: Optional[str] = None) -> tuple[str, List[str]]:
        """
        Inject context with numbered citations for proper source attribution.

        Args:
            query: The original user query
            sources: List of sources to inject into the prompt
            system_instructions: Optional system instructions to include

        Returns:
            Tuple of (formatted prompt, list of citation references)
        """
        try:
            self.logger.info(f"Injecting context with citations from {len(sources)} sources")

            # Build the system message
            if system_instructions:
                system_msg = system_instructions
            else:
                system_msg = ("You are a helpful AI assistant that answers questions based on provided context. "
                             "Use the provided context to answer the user's question accurately. "
                             "When answering, cite the sources using numbered references like [1], [2], etc. "
                             "Provide a references section at the end with full source information. "
                             "If the context doesn't contain the information needed to answer the question, say so.")

            # Build the context with numbered citations
            context_text, citations = self._build_context_with_citations(sources)

            # Create the user message with query and context
            user_message = f"Context:\n{context_text}\n\nQuestion: {query}"

            # Format the complete prompt
            formatted_prompt = f"{system_msg}\n\n{user_message}"

            self.logger.info("Context injection with citations completed successfully")
            return formatted_prompt, citations

        except Exception as e:
            self.logger.error(f"Error injecting context with citations: {str(e)}")
            return f"You are a helpful assistant. Question: {query}", []

    def _build_context_with_citations(self, sources: List[SourceReference]) -> tuple[str, List[str]]:
        """Build context text with numbered citations"""
        try:
            context_parts = []
            citations = []

            for i, source in enumerate(sources):
                # Create the context part with numbered reference
                context_part = f"[{i+1}] {source.title}\n"
                context_part += f"Content: {source.text_snippet}\n"

                # Add page information if available
                if source.page is not None:
                    context_part += f"Page: {source.page}\n"

                context_part += "\n"
                context_parts.append(context_part)

                # Create the citation reference
                citation = f"[{i+1}] {source.title}"
                if source.url:
                    citation += f" ({source.url})"
                citations.append(citation)

            context_text = "".join(context_parts)
            self.logger.debug(f"Built context with {len(sources)} citations")
            return context_text, citations

        except Exception as e:
            self.logger.error(f"Error building context with citations: {str(e)}")
            return "", []

    def format_response_with_citations(self, response: str, citations: List[str]) -> str:
        """Format the response with proper citation references"""
        try:
            # Add the references section to the response
            formatted_response = response

            if citations:
                formatted_response += "\n\nReferences:\n"
                for citation in citations:
                    formatted_response += f"{citation}\n"

            self.logger.info("Response formatted with citations")
            return formatted_response

        except Exception as e:
            self.logger.error(f"Error formatting response with citations: {str(e)}")
            return response

    def truncate_context_if_needed(self, context: str, max_length: Optional[int] = None) -> str:
        """Truncate context if it exceeds the maximum length"""
        try:
            max_len = max_length or self.max_context_length

            if len(context) > max_len:
                # Truncate to the nearest sentence boundary
                truncated = context[:max_len]
                last_period = truncated.rfind('.')
                if last_period != -1:
                    truncated = truncated[:last_period + 1]
                else:
                    # If no period found, truncate to max_len
                    truncated = truncated[:max_len]

                self.logger.warning(f"Context truncated from {len(context)} to {len(truncated)} characters")
                return truncated

            return context

        except Exception as e:
            self.logger.error(f"Error truncating context: {str(e)}")
            return context[:max_length] if max_length else context

    def inject_context_for_query_type(self,
                                   query: str,
                                   sources: List[SourceReference],
                                   query_type: str = "general") -> str:
        """
        Inject context optimized for specific query types.

        Args:
            query: The original user query
            sources: List of sources to inject into the prompt
            query_type: Type of query ("general", "factual", "analytical", "comparative")

        Returns:
            Formatted prompt optimized for the query type
        """
        try:
            self.logger.info(f"Injecting context for {query_type} query type")

            # Define system instructions based on query type
            system_instructions_map = {
                "general": ("You are a helpful AI assistant that answers questions based on provided context. "
                           "Use the provided context to answer the user's question accurately. "
                           "Always cite the sources you used to answer the question."),
                "factual": ("You are a factual AI assistant. Answer the question based strictly on the provided context. "
                           "Provide specific facts and figures from the sources. Cite your sources with numbered references."),
                "analytical": ("You are an analytical AI assistant. Analyze the provided context to answer the question. "
                              "Provide reasoning and insights based on the sources. Cite your sources with numbered references."),
                "comparative": ("You are a comparative AI assistant. Compare and contrast the information in the provided context. "
                               "Answer the question by highlighting differences and similarities. Cite your sources with numbered references.")
            }

            system_instructions = system_instructions_map.get(query_type, system_instructions_map["general"])

            # Use the standard injection method with the appropriate system instructions
            formatted_prompt = self.inject_context(query, sources, system_instructions)

            self.logger.info(f"Context injection for {query_type} query completed")
            return formatted_prompt

        except Exception as e:
            self.logger.error(f"Error injecting context for query type {query_type}: {str(e)}")
            return self.inject_context(query, sources)  # Fall back to standard injection


# Global context injector instance
context_injector = ContextInjector()


def inject_context_for_grounding(query: str, sources: List[SourceReference]) -> str:
    """
    Convenience function to inject context for grounded responses.

    Args:
        query: The user's query
        sources: List of sources to inject

    Returns:
        Formatted prompt with injected context
    """
    return context_injector.inject_context(query, sources)


def inject_context_with_citations_for_grounding(query: str, sources: List[SourceReference]) -> tuple[str, List[str]]:
    """
    Convenience function to inject context with numbered citations.

    Args:
        query: The user's query
        sources: List of sources to inject

    Returns:
        Tuple of (formatted prompt, list of citations)
    """
    return context_injector.inject_context_with_citations(query, sources)