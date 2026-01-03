import cohere
from typing import List, Optional
import asyncio
import logging
from ..config import settings


logger = logging.getLogger(__name__)


class EmbeddingService:
    def __init__(self):
        self.client = cohere.Client(settings.cohere_api_key)
        self.model = settings.cohere_model
        # Simple in-memory cache for embeddings to avoid redundant API calls
        self._cache = {}

    async def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for validation queries using Cohere API
        """
        try:
            # Check cache first
            uncached_texts = []
            uncached_indices = []

            for i, text in enumerate(texts):
                cache_key = f"{text}:{self.model}"
                if cache_key in self._cache:
                    # Placeholder - we'll fill this in later
                    pass
                else:
                    uncached_texts.append(text)
                    uncached_indices.append(i)

            # Generate embeddings for uncached texts
            if uncached_texts:
                response = self.client.embed(
                    texts=uncached_texts,
                    model=self.model
                )

                # Cache the results
                for i, embedding in enumerate(response.embeddings):
                    text_idx = uncached_indices[i]
                    cache_key = f"{texts[text_idx]}:{self.model}"
                    self._cache[cache_key] = embedding

            # Return all embeddings in order
            result = []
            for text in texts:
                cache_key = f"{text}:{self.model}"
                result.append(self._cache[cache_key])

            return result

        except Exception as e:
            logger.error(f"Failed to generate embeddings: {str(e)}")
            raise

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single validation query
        """
        embeddings = await self.generate_embeddings([text])
        return embeddings[0]

    def clear_cache(self):
        """
        Clear the embedding cache
        """
        self._cache.clear()