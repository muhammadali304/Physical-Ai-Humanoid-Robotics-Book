"""
Content chunking service for the RAG Ingestion Pipeline.
Implements semantic chunking algorithms to split content into meaningful segments.
"""

import re
import time
from typing import List, Optional, Tuple, Dict, Any
from dataclasses import dataclass
from collections import deque
from datetime import datetime

from src.models.chunk import ContentChunk
from src.config.settings import settings
from src.config.constants import (
    DEFAULT_MAX_CHUNK_SIZE,
    DEFAULT_CHUNK_OVERLAP,
    DEFAULT_MIN_CONTENT_LENGTH
)
from src.utils.logging import get_logger
from src.utils.preprocessor import ContentPreprocessor


@dataclass
class ChunkMetrics:
    """Metrics for chunking operations."""
    content_length: int = 0
    processing_time_ms: float = 0.0
    chunks_created: int = 0
    total_tokens: int = 0
    avg_chunk_size: float = 0.0
    avg_chunk_tokens: int = 0
    min_chunk_size: int = 0
    max_chunk_size: int = 0
    min_chunk_tokens: int = 0
    max_chunk_tokens: int = 0
    chunks_with_issues: int = 0
    validation_issues: List[str] = None
    start_time: datetime = None
    end_time: datetime = None

    def __post_init__(self):
        if self.validation_issues is None:
            self.validation_issues = []


@dataclass
class ChunkResult:
    """Result of a chunking operation."""
    chunks: List[ContentChunk]
    total_chunks: int = 0
    total_tokens: int = 0
    average_chunk_size: float = 0.0
    metrics: Optional[ChunkMetrics] = None

    def __post_init__(self):
        if self.chunks:
            self.total_chunks = len(self.chunks)
            self.total_tokens = sum(len(chunk.content.split()) for chunk in self.chunks)
            self.average_chunk_size = self.total_tokens / self.total_chunks if self.total_chunks > 0 else 0


class ChunkerService:
    """Service to chunk content into semantically meaningful segments."""

    def __init__(self):
        self.logger = get_logger("chunker")
        self.max_chunk_size = settings.max_chunk_size
        self.chunk_overlap = settings.chunk_overlap
        self.min_content_length = settings.min_content_length
        self.preprocessor = ContentPreprocessor()

    def chunk_content(
        self,
        content: str,
        source_url: str,
        page_title: str,
        chunk_index_start: int = 0,
        section_heading: Optional[str] = None
    ) -> ChunkResult:
        """
        Chunk content into semantically meaningful segments.

        Args:
            content: Content to chunk
            source_url: URL of the source content
            page_title: Title of the source page
            chunk_index_start: Starting index for chunk numbering
            section_heading: Optional section heading for context

        Returns:
            ChunkResult with the created chunks and metrics
        """
        start_time = datetime.now()
        start_timestamp = time.time()

        if not content or len(content.strip()) < self.min_content_length:
            self.logger.warning(
                f"Content too short to chunk: {len(content)} characters",
                source_url=source_url,
                content_length=len(content)
            )
            # Return with metrics even for empty results
            processing_time = (time.time() - start_timestamp) * 1000  # Convert to milliseconds
            metrics = ChunkMetrics(
                content_length=len(content),
                processing_time_ms=processing_time,
                start_time=start_time,
                end_time=datetime.now()
            )
            return ChunkResult(chunks=[], metrics=metrics)

        # Validate content quality before chunking
        is_valid, validation_issues = self.validate_content_quality(content)
        chunks_with_issues = 0
        if not is_valid:
            self.logger.warning(
                f"Content quality validation failed for {source_url}: {len(validation_issues)} issues",
                source_url=source_url,
                validation_issues=validation_issues
            )
            chunks_with_issues = 1  # Count the entire content as having issues
            # Continue processing but log the issues

        self.logger.info(
            f"Chunking content from {source_url}",
            source_url=source_url,
            content_length=len(content),
            max_chunk_size=self.max_chunk_size,
            chunk_overlap=self.chunk_overlap
        )

        # Use semantic chunking based on document structure
        chunks = self.semantic_chunking(
            content,
            source_url,
            page_title,
            chunk_index_start,
            section_heading
        )

        # Calculate metrics
        processing_time = (time.time() - start_timestamp) * 1000  # Convert to milliseconds
        total_tokens = sum(chunk.token_count for chunk in chunks)
        avg_chunk_tokens = total_tokens / len(chunks) if chunks else 0

        chunk_sizes = [len(chunk.content) for chunk in chunks]
        min_chunk_size = min(chunk_sizes) if chunk_sizes else 0
        max_chunk_size = max(chunk_sizes) if chunk_sizes else 0

        chunk_tokens = [chunk.token_count for chunk in chunks]
        min_chunk_tokens = min(chunk_tokens) if chunk_tokens else 0
        max_chunk_tokens = max(chunk_tokens) if chunk_tokens else 0

        metrics = ChunkMetrics(
            content_length=len(content),
            processing_time_ms=processing_time,
            chunks_created=len(chunks),
            total_tokens=total_tokens,
            avg_chunk_size=sum(chunk_sizes) / len(chunk_sizes) if chunk_sizes else 0,
            avg_chunk_tokens=avg_chunk_tokens,
            min_chunk_size=min_chunk_size,
            max_chunk_size=max_chunk_size,
            min_chunk_tokens=min_chunk_tokens,
            max_chunk_tokens=max_chunk_tokens,
            chunks_with_issues=chunks_with_issues,
            validation_issues=validation_issues if not is_valid else [],
            start_time=start_time,
            end_time=datetime.now()
        )

        result = ChunkResult(chunks=chunks, metrics=metrics)

        self.logger.info(
            f"Completed chunking: {result.total_chunks} chunks created in {processing_time:.2f}ms",
            total_chunks=result.total_chunks,
            total_tokens=result.total_tokens,
            average_chunk_size=result.average_chunk_size,
            processing_time_ms=processing_time
        )

        return result

    def validate_content_quality(self, content: str) -> Tuple[bool, List[str]]:
        """
        Validate the quality of content before chunking.

        Args:
            content: Content to validate

        Returns:
            Tuple of (is_valid, list_of_issues)
        """
        issues = []

        # Check content length
        if len(content.strip()) < self.min_content_length:
            issues.append(f"Content too short: {len(content)} chars (min: {self.min_content_length})")

        # Check for meaningful content (not just whitespace or special characters)
        content_without_whitespace = re.sub(r'\s+', '', content)
        if len(content_without_whitespace) == 0:
            issues.append("Content contains only whitespace")

        # Check for excessive special characters
        special_char_ratio = len(re.findall(r'[^\w\s]', content)) / len(content) if content else 0
        if special_char_ratio > 0.5:  # More than 50% special characters
            issues.append(f"Excessive special characters: {special_char_ratio:.2%} of content")

        # Check for duplicated content
        if len(content) > 100:  # Only check for reasonably long content
            # Check if the content contains repeated sentences
            sentences = re.split(r'[.!?]+', content)
            unique_sentences = set(s.strip() for s in sentences if s.strip())
            if len(unique_sentences) < len(sentences) * 0.5:  # Less than 50% unique sentences
                issues.append(f"Highly repetitive content: only {len(unique_sentences)}/{len(sentences)} unique sentences")

        # Check for proper sentence structure
        sentence_count = len([s for s in re.split(r'[.!?]+', content) if s.strip()])
        word_count = len(re.findall(r'\b\w+\b', content))
        if sentence_count > 0 and word_count / sentence_count < 3:  # Avg less than 3 words per sentence
            issues.append(f"Unusual sentence structure: avg {word_count / sentence_count:.1f} words per sentence")

        # Check for excessive numbers or special formatting that might indicate non-text content
        numbers_ratio = len(re.findall(r'\d', content)) / len(content) if content else 0
        if numbers_ratio > 0.3:  # More than 30% numbers
            issues.append(f"High proportion of numbers: {numbers_ratio:.2%} of content")

        is_valid = len(issues) == 0
        return is_valid, issues

    def aggregate_metrics(self, chunk_results: List[ChunkResult]) -> Dict[str, Any]:
        """
        Aggregate metrics across multiple chunking operations.

        Args:
            chunk_results: List of ChunkResult objects to aggregate

        Returns:
            Dictionary with aggregated metrics
        """
        if not chunk_results:
            return {}

        # Filter out results without metrics
        results_with_metrics = [result for result in chunk_results if result.metrics]

        if not results_with_metrics:
            return {}

        total_processing_time = sum(result.metrics.processing_time_ms for result in results_with_metrics)
        total_chunks_created = sum(result.metrics.chunks_created for result in results_with_metrics)
        total_tokens = sum(result.metrics.total_tokens for result in results_with_metrics)

        # Calculate averages
        avg_processing_time = total_processing_time / len(results_with_metrics) if results_with_metrics else 0
        avg_tokens_per_result = total_tokens / len(results_with_metrics) if results_with_metrics else 0
        avg_chunks_per_result = total_chunks_created / len(results_with_metrics) if results_with_metrics else 0

        # Calculate overall rates
        total_content_length = sum(result.metrics.content_length for result in results_with_metrics)
        avg_content_per_result = total_content_length / len(results_with_metrics) if results_with_metrics else 0

        # Tokenization efficiency: tokens per second
        total_time_seconds = sum(result.metrics.processing_time_ms for result in results_with_metrics) / 1000.0
        tokens_per_second = total_tokens / total_time_seconds if total_time_seconds > 0 else 0

        # Aggregate validation issues
        all_validation_issues = []
        total_issues = 0
        for result in results_with_metrics:
            all_validation_issues.extend(result.metrics.validation_issues)
            total_issues += len(result.metrics.validation_issues)

        # Find min/max across all results
        if results_with_metrics:
            min_processing_time = min(result.metrics.processing_time_ms for result in results_with_metrics)
            max_processing_time = max(result.metrics.processing_time_ms for result in results_with_metrics)
            min_tokens = min(result.metrics.total_tokens for result in results_with_metrics if result.metrics.total_tokens > 0) if any(r.metrics.total_tokens > 0 for r in results_with_metrics) else 0
            max_tokens = max(result.metrics.total_tokens for result in results_with_metrics) if results_with_metrics else 0
        else:
            min_processing_time = max_processing_time = min_tokens = max_tokens = 0

        aggregated_metrics = {
            "total_operations": len(results_with_metrics),
            "total_processing_time_ms": total_processing_time,
            "avg_processing_time_ms": avg_processing_time,
            "min_processing_time_ms": min_processing_time,
            "max_processing_time_ms": max_processing_time,
            "total_chunks_created": total_chunks_created,
            "avg_chunks_per_operation": avg_chunks_per_result,
            "total_tokens_processed": total_tokens,
            "avg_tokens_per_operation": avg_tokens_per_result,
            "total_content_length_processed": total_content_length,
            "avg_content_length_per_operation": avg_content_per_result,
            "tokens_per_second": tokens_per_second,
            "total_validation_issues": total_issues,
            "unique_validation_issues": list(set(all_validation_issues)),
            "min_tokens_per_operation": min_tokens,
            "max_tokens_per_operation": max_tokens,
            "start_time": min((result.metrics.start_time for result in results_with_metrics), default=None),
            "end_time": max((result.metrics.end_time for result in results_with_metrics), default=None),
        }

        return aggregated_metrics

    def log_metrics_summary(self, metrics: Dict[str, Any], operation_name: str = "chunking"):
        """
        Log a summary of aggregated metrics.

        Args:
            metrics: Dictionary of aggregated metrics
            operation_name: Name of the operation for logging purposes
        """
        if not metrics:
            self.logger.info(f"No metrics to log for {operation_name}")
            return

        self.logger.info(
            f"{operation_name.capitalize()} metrics summary",
            total_operations=metrics.get("total_operations", 0),
            total_processing_time_ms=metrics.get("total_processing_time_ms", 0),
            avg_processing_time_ms=metrics.get("avg_processing_time_ms", 0),
            total_chunks_created=metrics.get("total_chunks_created", 0),
            avg_chunks_per_operation=metrics.get("avg_chunks_per_operation", 0),
            total_tokens_processed=metrics.get("total_tokens_processed", 0),
            tokens_per_second=metrics.get("tokens_per_second", 0),
            total_validation_issues=metrics.get("total_validation_issues", 0)
        )

    def semantic_chunking(
        self,
        content: str,
        source_url: str,
        page_title: str,
        chunk_index_start: int = 0,
        section_heading: Optional[str] = None
    ) -> List[ContentChunk]:
        """
        Perform semantic chunking based on document structure (headings, paragraphs).

        Args:
            content: Content to chunk
            source_url: URL of the source content
            page_title: Title of the source page
            chunk_index_start: Starting index for chunk numbering
            section_heading: Optional section heading for context

        Returns:
            List of ContentChunk objects
        """
        # Preprocess content to maintain structure
        preprocessed_content = self.preprocessor.preprocess_for_chunking(content)

        # Split content by semantic boundaries (headings, sections)
        sections = self.split_by_headings_with_context(preprocessed_content, content)

        chunks = []
        current_index = chunk_index_start

        for i, (section, context_heading) in enumerate(sections):
            # Determine the appropriate section heading for this chunk
            effective_heading = context_heading or section_heading

            # Use the new validation method to ensure chunks meet size requirements
            section_chunks = self.adjust_chunk_to_valid_size(
                section, source_url, page_title, current_index, effective_heading
            )
            chunks.extend(section_chunks)
            current_index += len(section_chunks)

        # Apply overlap between consecutive chunks
        chunks_with_overlap = self.apply_overlap(chunks)

        return chunks_with_overlap

    def split_by_headings(self, content: str) -> List[str]:
        """
        Split content by headings (h1, h2, h3, etc.).

        Args:
            content: Content to split

        Returns:
            List of content sections
        """
        # Split by markdown-style headings (# ## ###) or HTML headings
        # Use separate patterns to avoid backreference issues
        # Match markdown headings: # ## ### etc.
        markdown_pattern = r'(?:^|\n)#{1,6}\s+.*?(?=\n|$)'
        # Match HTML headings: <h1>, <h2>, etc.
        html_pattern = r'<h[1-6][^>]*>.*?</h[1-6]>'
        # Combine with non-capturing group
        heading_pattern = f'(?:{markdown_pattern}|{html_pattern})'
        splits = re.split(heading_pattern, content, flags=re.MULTILINE | re.IGNORECASE)

        sections = []
        current_section = ""

        for i, part in enumerate(splits):
            # Check if this part is a heading
            if re.match(r'(#{1,6}\s+|<(h[1-6])[^>]*>)', part.strip(), re.IGNORECASE):
                # If we have accumulated content, save it as a section
                if current_section.strip():
                    sections.append(current_section.strip())
                    current_section = ""

                # Add the heading as a separate section
                current_section += part.strip() + "\n"
            else:
                # Add non-heading content to current section
                current_section += part

        # Add the last section if it exists
        if current_section.strip():
            sections.append(current_section.strip())

        # Filter out empty sections and merge small sections with adjacent ones
        sections = [s for s in sections if s.strip()]
        sections = self.merge_small_sections(sections)

        return sections

    def split_by_headings_with_context(self, preprocessed_content: str, original_content: str) -> List[Tuple[str, Optional[str]]]:
        """
        Split content by headings and preserve context by associating content with its headings.

        Args:
            preprocessed_content: Preprocessed content to split
            original_content: Original content for reference

        Returns:
            List of tuples containing (section_content, associated_heading)
        """
        # Split by markdown-style headings (# ## ###) or HTML headings
        # Use separate patterns to avoid backreference issues
        # Match markdown headings: # ## ### etc.
        markdown_pattern = r'(?:^|\n)#{1,6}\s+.*?(?=\n|$)'
        # Match HTML headings: <h1>, <h2>, etc.
        html_pattern = r'<h[1-6][^>]*>.*?</h[1-6]>'
        # Combine with non-capturing group
        heading_pattern = f'(?:{markdown_pattern}|{html_pattern})'
        splits = re.split(heading_pattern, preprocessed_content, flags=re.MULTILINE | re.IGNORECASE)

        sections_with_context = []
        current_section = ""
        current_heading = None

        for i, part in enumerate(splits):
            # Check if this part is a heading
            if re.match(r'(#{1,6}\s+|<(h[1-6])[^>]*>)', part.strip(), re.IGNORECASE):
                # If we have accumulated content, save it with the previous heading
                if current_section.strip():
                    sections_with_context.append((current_section.strip(), current_heading))
                    current_section = ""

                # Extract the heading text (remove markdown/HTML tags)
                heading_match = re.match(r'(#{1,6}\s+|<(h[1-6])[^>]*>)(.*?)(?=\n|$)', part.strip(), re.IGNORECASE)
                if heading_match:
                    heading_text = heading_match.group(3).strip()
                    current_heading = heading_text
            else:
                # Add non-heading content to current section
                current_section += part

        # Add the last section if it exists
        if current_section.strip():
            sections_with_context.append((current_section.strip(), current_heading))

        # Filter out empty sections but keep the heading context
        sections_with_context = [(s, h) for s, h in sections_with_context if s.strip()]

        # Merge small sections with adjacent ones while preserving heading context
        sections_with_context = self.merge_small_sections_with_context(sections_with_context)

        return sections_with_context

    def merge_small_sections_with_context(self, sections_with_context: List[Tuple[str, Optional[str]]]) -> List[Tuple[str, Optional[str]]]:
        """
        Merge small sections with adjacent ones while preserving heading context.

        Args:
            sections_with_context: List of tuples (section_content, associated_heading)

        Returns:
            List of merged sections with context preserved
        """
        if len(sections_with_context) <= 1:
            return sections_with_context

        merged = []
        current_content, current_heading = sections_with_context[0]

        for i in range(1, len(sections_with_context)):
            next_content, next_heading = sections_with_context[i]

            # If current section is too small, merge with the next one
            if self.estimate_token_count(current_content) < self.max_chunk_size * 0.3:  # 30% of max size
                # Merge content
                current_content += "\n\n" + next_content

                # Preserve the most relevant heading - use next heading if it's more specific
                if next_heading and not current_heading:
                    current_heading = next_heading
                elif not next_heading and not current_heading:
                    current_heading = None
                # If both have headings, keep the first one (it's the section title)
            else:
                # Add the current section to merged list
                merged.append((current_content, current_heading))
                # Start a new section with the next content and heading
                current_content, current_heading = next_content, next_heading

        # Add the last section
        merged.append((current_content, current_heading))

        return merged

    def split_large_section(self, section: str, source_url: str, page_title: str,
                           start_index: int, section_heading: Optional[str]) -> List[ContentChunk]:
        """
        Split a large section into smaller chunks using paragraph boundaries.
        This method is now deprecated in favor of the new validation-based approach,
        but kept for backward compatibility.

        Args:
            section: Large section to split
            source_url: URL of the source content
            page_title: Title of the source page
            start_index: Starting index for chunk numbering
            section_heading: Optional section heading for context

        Returns:
            List of ContentChunk objects
        """
        # Use the new validation-based approach for splitting
        return self.split_large_content(section, source_url, page_title, start_index, section_heading)

    def split_paragraph_by_sentences(self, paragraph: str) -> List[str]:
        """
        Split a paragraph by sentences while respecting sentence boundaries.

        Args:
            paragraph: Paragraph to split

        Returns:
            List of sentence chunks
        """
        # Split by sentence endings followed by whitespace and capital letter
        sentences = re.split(r'(?<=[.!?])\s+(?=[A-Z])', paragraph)

        # Further refine to ensure chunks aren't too large
        refined_chunks = []
        for sent in sentences:
            if self.estimate_token_count(sent) > self.max_chunk_size:
                # If sentence is still too long, split by clauses
                sub_chunks = re.split(r'(?<=[,;:])\s+(?=[a-z])', sent)
                refined_chunks.extend(sub_chunks)
            else:
                refined_chunks.append(sent)

        # Filter out empty chunks
        return [chunk.strip() for chunk in refined_chunks if chunk.strip()]

    def apply_overlap(self, chunks: List[ContentChunk]) -> List[ContentChunk]:
        """
        Apply overlap between consecutive chunks to maintain context.
        Implements 20% overlap as specified in requirements.

        Args:
            chunks: List of chunks to apply overlap to

        Returns:
            List of chunks with overlap applied
        """
        if len(chunks) <= 1:
            return chunks

        # Use 20% overlap as specified in requirements
        overlap_percentage = 0.20
        overlapped_chunks = []

        for i, chunk in enumerate(chunks):
            new_content = chunk.content

            # If not the last chunk, add overlap from the next chunk
            if i < len(chunks) - 1:
                next_chunk = chunks[i + 1]

                # Calculate 20% overlap based on the next chunk's content
                overlap_size = max(1, int(len(next_chunk.content) * overlap_percentage))

                if overlap_size > 0:
                    # Get the overlap content from the beginning of the next chunk
                    overlap_content = next_chunk.content[:overlap_size]

                    # Add the overlap content to the current chunk
                    # Use a clear separator to distinguish original content from overlap
                    new_content = chunk.content + "\n\n[CONTINUATION: " + overlap_content + "]\n"

            # Create a new chunk with the updated content
            overlapped_chunk = ContentChunk(
                id=chunk.id,  # Keep the same ID
                source_url=chunk.source_url,
                page_title=chunk.page_title,
                section_heading=chunk.section_heading,
                chunk_index=chunk.chunk_index,
                content=new_content,
                token_count=self.estimate_token_count(new_content),
                metadata=chunk.metadata,
                created_at=chunk.created_at,
                updated_at=chunk.updated_at
            )

            overlapped_chunks.append(overlapped_chunk)

        return overlapped_chunks

    def estimate_token_count(self, text: str) -> int:
        """
        Estimate the number of tokens in text (rough approximation using words).

        Args:
            text: Text to count tokens for

        Returns:
            Estimated number of tokens
        """
        if not text:
            return 0

        # Simple word-based token estimation (1 word ≈ 1 token for English)
        # In practice, you might want to use a proper tokenizer like tiktoken
        words = re.findall(r'\b\w+\b', text.lower())
        return len(words)

    def validate_chunk_size(self, content: str) -> Tuple[bool, int, List[str]]:
        """
        Validate that the chunk size is within the acceptable range (50-1000 tokens).

        Args:
            content: Content to validate

        Returns:
            Tuple of (is_valid, token_count, list_of_issues)
        """
        token_count = self.estimate_token_count(content)
        issues = []

        if token_count < 50:
            issues.append(f"Content has only {token_count} tokens, which is below minimum of 50")
        elif token_count > 1000:
            issues.append(f"Content has {token_count} tokens, which exceeds maximum of 1000")

        is_valid = len(issues) == 0
        return is_valid, token_count, issues

    def adjust_chunk_to_valid_size(self, content: str, source_url: str, page_title: str,
                                 chunk_index: int, section_heading: Optional[str] = None) -> List[ContentChunk]:
        """
        Adjust a chunk to ensure it meets size requirements by splitting if too large
        or potentially merging with adjacent chunks if too small.

        Args:
            content: Content to adjust
            source_url: URL of the source content
            page_title: Title of the source page
            chunk_index: Index for the chunk
            section_heading: Optional section heading for context

        Returns:
            List of properly sized ContentChunk objects
        """
        is_valid, token_count, issues = self.validate_chunk_size(content)

        if is_valid:
            # Content is already valid size
            chunk = ContentChunk(
                source_url=source_url,
                page_title=page_title,
                section_heading=section_heading,
                chunk_index=chunk_index,
                content=content.strip(),
                token_count=token_count
            )
            return [chunk]

        if token_count > 1000:
            # Content is too large, split it into smaller chunks
            return self.split_large_content(content, source_url, page_title, chunk_index, section_heading)
        else:
            # Content is too small, skip creating a chunk to avoid validation errors
            self.logger.warning(
                f"Content chunk is too small ({token_count} tokens, minimum 50), skipping",
                source_url=source_url,
                chunk_index=chunk_index,
                token_count=token_count
            )
            return []  # Return empty list instead of invalid chunk

    def split_large_content(self, content: str, source_url: str, page_title: str,
                          start_index: int, section_heading: Optional[str]) -> List[ContentChunk]:
        """
        Split large content into smaller chunks that meet size requirements.

        Args:
            content: Large content to split
            source_url: URL of the source content
            page_title: Title of the source page
            start_index: Starting index for chunk numbering
            section_heading: Optional section heading for context

        Returns:
            List of ContentChunk objects that meet size requirements
        """
        # First try to split by paragraphs
        paragraphs = re.split(r'\n\s*\n', content)

        chunks = []
        current_chunk_content = ""
        current_index = start_index

        for para in paragraphs:
            para_token_count = self.estimate_token_count(para)

            if para_token_count > 1000:
                # Paragraph is still too big, split by sentences
                sentence_chunks = self.split_paragraph_by_sentences(para)
                for sent_chunk in sentence_chunks:
                    sent_token_count = self.estimate_token_count(sent_chunk)

                    if sent_token_count > 1000:
                        # Sentence is still too big, split by clauses
                        clause_chunks = re.split(r'(?<=,)|(?<=;)|(?<=:)', sent_chunk)
                        for clause_chunk in clause_chunks:
                            clause_chunk = clause_chunk.strip()
                            if clause_chunk:
                                clause_token_count = self.estimate_token_count(clause_chunk)
                                if clause_token_count <= 1000:
                                    chunk = ContentChunk(
                                        source_url=source_url,
                                        page_title=page_title,
                                        section_heading=section_heading,
                                        chunk_index=current_index,
                                        content=clause_chunk,
                                        token_count=clause_token_count
                                    )
                                    chunks.append(chunk)
                                    current_index += 1
                                else:
                                    # If still too big, force split by character count
                                    sub_chunks = self.force_split_content(clause_chunk, source_url, page_title, current_index, section_heading)
                                    chunks.extend(sub_chunks)
                                    current_index += len(sub_chunks)
                    elif sent_token_count <= 1000:
                        # Check if adding to current chunk would exceed limit
                        current_token_count = self.estimate_token_count(current_chunk_content)
                        if current_token_count + sent_token_count <= 1000 and current_chunk_content:
                            current_chunk_content += sent_chunk + "\n\n"
                        else:
                            # Save current chunk if it meets requirements
                            if current_chunk_content and self.estimate_token_count(current_chunk_content) >= 50:
                                chunk = ContentChunk(
                                    source_url=source_url,
                                    page_title=page_title,
                                    section_heading=section_heading,
                                    chunk_index=current_index,
                                    content=current_chunk_content.strip(),
                                    token_count=self.estimate_token_count(current_chunk_content)
                                )
                                chunks.append(chunk)
                                current_index += 1
                                current_chunk_content = sent_chunk + "\n\n"
                            elif current_chunk_content and self.estimate_token_count(current_chunk_content) < 50:
                                # Current chunk is too small, add sentence and continue
                                current_chunk_content += sent_chunk + "\n\n"
                            else:
                                # Start new chunk with this sentence
                                current_chunk_content = sent_chunk + "\n\n"
            else:
                # Check if adding this paragraph would exceed the size limit
                current_token_count = self.estimate_token_count(current_chunk_content)

                if current_token_count + para_token_count > 1000 and current_chunk_content.strip():
                    # Save current chunk and start a new one
                    chunk = ContentChunk(
                        source_url=source_url,
                        page_title=page_title,
                        section_heading=section_heading,
                        chunk_index=current_index,
                        content=current_chunk_content.strip(),
                        token_count=current_token_count
                    )
                    chunks.append(chunk)
                    current_index += 1

                    current_chunk_content = para + "\n\n"
                else:
                    current_chunk_content += para + "\n\n"

        # Add the last chunk if there's content left
        if current_chunk_content.strip():
            final_token_count = self.estimate_token_count(current_chunk_content)
            if final_token_count >= 50:  # Only create chunk if it meets minimum size
                chunk = ContentChunk(
                    source_url=source_url,
                    page_title=page_title,
                    section_heading=section_heading,
                    chunk_index=current_index,
                    content=current_chunk_content.strip(),
                    token_count=final_token_count
                )
                chunks.append(chunk)

        return chunks

    def force_split_content(self, content: str, source_url: str, page_title: str,
                          start_index: int, section_heading: Optional[str]) -> List[ContentChunk]:
        """
        Force split content by character count when other methods fail.

        Args:
            content: Content to force split
            source_url: URL of the source content
            page_title: Title of the source page
            start_index: Starting index for chunk numbering
            section_heading: Optional section heading for context

        Returns:
            List of ContentChunk objects
        """
        chunks = []
        current_index = start_index

        # Estimate tokens per character for rough calculation
        estimated_chars_per_token = len(content) / max(self.estimate_token_count(content), 1)
        max_chars_per_chunk = int(900 * estimated_chars_per_token)  # Use 900 to stay under 1000 token limit

        for i in range(0, len(content), max_chars_per_chunk):
            chunk_content = content[i:i + max_chars_per_chunk]
            chunk_token_count = self.estimate_token_count(chunk_content)

            chunk = ContentChunk(
                source_url=source_url,
                page_title=page_title,
                section_heading=section_heading,
                chunk_index=current_index,
                content=chunk_content,
                token_count=chunk_token_count
            )
            chunks.append(chunk)
            current_index += 1

        return chunks

    def merge_small_sections(self, sections: List[str]) -> List[str]:
        """
        Merge small sections with adjacent ones to avoid overly small chunks.

        Args:
            sections: List of sections to merge

        Returns:
            List of merged sections
        """
        if len(sections) <= 1:
            return sections

        merged = []
        current = sections[0]

        for i in range(1, len(sections)):
            # If current section is too small, merge with the next one
            if self.estimate_token_count(current) < self.max_chunk_size * 0.3:  # 30% of max size
                current += "\n\n" + sections[i]
            else:
                merged.append(current)
                current = sections[i]

        # Add the last section
        merged.append(current)

        return merged

    def validate_chunk_quality(self, chunk: ContentChunk) -> Tuple[bool, List[str]]:
        """
        Validate the quality of a chunk.

        Args:
            chunk: Content chunk to validate

        Returns:
            Tuple of (is_valid, list_of_issues)
        """
        issues = []

        # Check content length
        if len(chunk.content.strip()) < self.min_content_length:
            issues.append(f"Content too short: {len(chunk.content)} chars (min: {self.min_content_length})")

        # Check token count range (50-1000 tokens as per requirements)
        if chunk.token_count < 50:
            issues.append(f"Too few tokens: {chunk.token_count} (min: 50)")
        elif chunk.token_count > 1000:
            issues.append(f"Too many tokens: {chunk.token_count} (max: 1000)")

        # Check for meaningful content (not just whitespace or special characters)
        content_without_whitespace = re.sub(r'\s+', '', chunk.content)
        if len(content_without_whitespace) == 0:
            issues.append("Content contains only whitespace")

        # Check for excessive special characters
        special_char_ratio = len(re.findall(r'[^\w\s]', chunk.content)) / len(chunk.content) if chunk.content else 0
        if special_char_ratio > 0.5:  # More than 50% special characters
            issues.append(f"Excessive special characters: {special_char_ratio:.2%} of content")

        # Check for duplicated content
        if len(chunk.content) > 100:  # Only check for reasonably long content
            # Check if the chunk contains repeated sentences
            sentences = re.split(r'[.!?]+', chunk.content)
            unique_sentences = set(s.strip() for s in sentences if s.strip())
            if len(unique_sentences) < len(sentences) * 0.5:  # Less than 50% unique sentences
                issues.append(f"Highly repetitive content: only {len(unique_sentences)}/{len(sentences)} unique sentences")

        # Check for proper sentence structure
        sentence_count = len([s for s in re.split(r'[.!?]+', chunk.content) if s.strip()])
        word_count = len(re.findall(r'\b\w+\b', chunk.content))
        if sentence_count > 0 and word_count / sentence_count < 3:  # Avg less than 3 words per sentence
            issues.append(f"Unusual sentence structure: avg {word_count / sentence_count:.1f} words per sentence")

        # Check for proper heading context if available
        if chunk.section_heading and len(chunk.section_heading.strip()) > 100:  # Unusually long heading
            issues.append("Unusually long section heading (>100 chars)")

        is_valid = len(issues) == 0
        return is_valid, issues

    def chunk_document_structure(
        self,
        content: str,
        source_url: str,
        page_title: str,
        heading_hierarchy: Optional[Dict[str, str]] = None
    ) -> List[ContentChunk]:
        """
        Chunk content respecting document structure and heading hierarchy.

        Args:
            content: Content to chunk
            source_url: URL of the source content
            page_title: Title of the source page
            heading_hierarchy: Optional mapping of headings to their hierarchy levels

        Returns:
            List of ContentChunk objects
        """
        # This method would implement more sophisticated document structure-aware chunking
        # For now, we'll use the semantic chunking method
        return self.semantic_chunking(content, source_url, page_title)


# Convenience function to create a default chunker
def create_default_chunker() -> ChunkerService:
    """Create a default chunker service instance."""
    return ChunkerService()