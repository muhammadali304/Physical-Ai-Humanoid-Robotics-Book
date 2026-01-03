"""
Token counting utilities for the RAG Ingestion Pipeline.
Provides functions to estimate and count tokens in text content.
"""

import re
from typing import Union, List
from transformers import AutoTokenizer


class TokenCounter:
    """
    Utility class for counting tokens in text content.
    Provides multiple methods for token estimation and counting.
    """

    def __init__(self, model_name: str = "gpt2"):
        """
        Initialize the token counter with a specific model tokenizer.

        Args:
            model_name: Name of the model to use for tokenization (default: gpt2)
        """
        try:
            self.tokenizer = AutoTokenizer.from_pretrained(model_name)
            self.use_transformer_tokenizer = True
        except Exception:
            # Fallback to simple word-based counting
            self.tokenizer = None
            self.use_transformer_tokenizer = False

    def count_tokens(self, text: str) -> int:
        """
        Count the number of tokens in the given text.

        Args:
            text: Text to count tokens for

        Returns:
            Number of tokens
        """
        if not text:
            return 0

        if self.use_transformer_tokenizer:
            try:
                tokens = self.tokenizer.encode(text, add_special_tokens=False)
                return len(tokens)
            except Exception:
                # Fallback to word-based counting
                pass

        # Fallback: simple word-based token estimation
        return self.estimate_tokens_simple(text)

    def estimate_tokens_simple(self, text: str) -> int:
        """
        Estimate the number of tokens using simple heuristics.

        Args:
            text: Text to estimate tokens for

        Returns:
            Estimated number of tokens
        """
        if not text:
            return 0

        # Remove extra whitespace
        text = re.sub(r'\s+', ' ', text.strip())

        # Count words (common approach: 1 word ≈ 1.3 tokens for English)
        words = re.findall(r'\b\w+\b', text)
        word_count = len(words)

        # Count punctuation that often creates separate tokens
        punct_tokens = len(re.findall(r'[.!?,:;()\[\]{}\-\'\"]', text))

        # Estimate: words + some punctuation + special characters
        # This is a rough approximation; real tokenizers vary significantly
        estimated_tokens = int(word_count * 1.3) + punct_tokens // 2

        return max(estimated_tokens, 1)  # Ensure at least 1 token

    def count_tokens_batch(self, texts: List[str]) -> List[int]:
        """
        Count tokens for a batch of texts.

        Args:
            texts: List of texts to count tokens for

        Returns:
            List of token counts
        """
        return [self.count_tokens(text) for text in texts]

    def get_token_density(self, text: str) -> float:
        """
        Calculate the token density (tokens per character) of the text.

        Args:
            text: Text to analyze

        Returns:
            Token density (tokens per character)
        """
        if not text:
            return 0.0

        token_count = self.count_tokens(text)
        char_count = len(text)

        return token_count / char_count if char_count > 0 else 0.0

    def split_by_tokens(self, text: str, max_tokens: int, overlap: float = 0.0) -> List[str]:
        """
        Split text into chunks by token count.

        Args:
            text: Text to split
            max_tokens: Maximum number of tokens per chunk
            overlap: Fraction of overlap between chunks (0.0 to 1.0)

        Returns:
            List of text chunks
        """
        if not text or max_tokens <= 0:
            return [text] if text else []

        tokens = self.tokenize(text) if self.use_transformer_tokenizer else self.tokenize_simple(text)
        chunks = []

        if self.use_transformer_tokenizer:
            # Use transformer tokenizer
            chunk_size = int(max_tokens * (1 - overlap)) if overlap > 0 else max_tokens
            stride = max(int(max_tokens * overlap), 1) if overlap > 0 else max_tokens

            for i in range(0, len(tokens), chunk_size):
                end_idx = min(i + max_tokens, len(tokens))
                chunk_tokens = tokens[i:end_idx]

                # Decode tokens back to text
                chunk_text = self.tokenizer.decode(chunk_tokens, skip_special_tokens=True)
                chunks.append(chunk_text)

                # If overlap is specified and not at the end, continue with stride
                if overlap > 0 and end_idx < len(tokens):
                    continue
                elif overlap > 0:
                    break
        else:
            # Use simple word-based splitting
            words = re.findall(r'\S+|\s+', text)  # Include whitespace
            current_chunk = ""
            current_token_count = 0

            for word in words:
                word_token_count = self.estimate_tokens_simple(word)

                if current_token_count + word_token_count > max_tokens:
                    # If adding this word would exceed the limit, save current chunk
                    if current_chunk.strip():
                        chunks.append(current_chunk.strip())

                    # Start a new chunk with this word
                    current_chunk = word
                    current_token_count = word_token_count
                else:
                    # Add word to current chunk
                    current_chunk += word
                    current_token_count += word_token_count

            # Add the last chunk if it exists
            if current_chunk.strip():
                chunks.append(current_chunk.strip())

        return chunks

    def tokenize(self, text: str) -> List[int]:
        """
        Tokenize text into token IDs.

        Args:
            text: Text to tokenize

        Returns:
            List of token IDs
        """
        if not text:
            return []

        if self.use_transformer_tokenizer:
            return self.tokenizer.encode(text, add_special_tokens=False)
        else:
            # Fallback: return word indices
            words = re.findall(r'\b\w+\b', text)
            return list(range(len(words)))

    def tokenize_simple(self, text: str) -> List[str]:
        """
        Simple tokenization by splitting on whitespace and punctuation.

        Args:
            text: Text to tokenize

        Returns:
            List of simple tokens
        """
        if not text:
            return []

        # Split on whitespace and common punctuation
        tokens = re.findall(r'\b\w+\b|[.!?,:;()\[\]{}\-\'\"]', text)
        return [token for token in tokens if token.strip()]


class CohereTokenCounter(TokenCounter):
    """
    Specialized token counter for Cohere models.
    Uses Cohere's specific tokenization approach.
    """

    def __init__(self):
        """Initialize the Cohere token counter."""
        # For now, inherit from TokenCounter
        # In a real implementation, this would use Cohere's specific tokenizer
        super().__init__(model_name="gpt2")  # Using GPT-2 as a reasonable default

    def count_tokens(self, text: str) -> int:
        """
        Count tokens specifically for Cohere models.
        In practice, this would use Cohere's tokenizer.

        Args:
            text: Text to count tokens for

        Returns:
            Number of tokens
        """
        # For now, use the parent method
        # In a real implementation, this would use Cohere's tokenizer
        return super().count_tokens(text)


# Global token counter instances
_default_token_counter: TokenCounter = None
_cohere_token_counter: CohereTokenCounter = None


def get_default_token_counter() -> TokenCounter:
    """Get the default token counter instance."""
    global _default_token_counter
    if _default_token_counter is None:
        _default_token_counter = TokenCounter()
    return _default_token_counter


def get_cohere_token_counter() -> CohereTokenCounter:
    """Get the Cohere-specific token counter instance."""
    global _cohere_token_counter
    if _cohere_token_counter is None:
        _cohere_token_counter = CohereTokenCounter()
    return _cohere_token_counter


# Convenience functions
def count_tokens(text: str) -> int:
    """Convenience function to count tokens in text."""
    counter = get_default_token_counter()
    return counter.count_tokens(text)


def estimate_tokens_simple(text: str) -> int:
    """Convenience function to estimate tokens using simple heuristics."""
    counter = get_default_token_counter()
    return counter.estimate_tokens_simple(text)


def split_by_tokens(text: str, max_tokens: int, overlap: float = 0.0) -> List[str]:
    """Convenience function to split text by token count."""
    counter = get_default_token_counter()
    return counter.split_by_tokens(text, max_tokens, overlap)


def get_token_density(text: str) -> float:
    """Convenience function to get token density of text."""
    counter = get_default_token_counter()
    return counter.get_token_density(text)