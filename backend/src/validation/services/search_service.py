from typing import List, Optional, Dict, Any
import logging
from .embedding_service import EmbeddingService
from .qdrant_service import QdrantService
from ..models.validation_models import SearchQuery, SearchResult


logger = logging.getLogger(__name__)


class SearchService:
    def __init__(self):
        self.embedding_service = EmbeddingService()
        self.qdrant_service = QdrantService()

    async def search_validation(
        self,
        query_text: str,
        top_k: int = 5,
        min_score: float = 0.0,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[SearchResult]:
        """
        Perform validation search: generate embedding and search in Qdrant
        """
        try:
            # Generate embedding for the query
            query_embedding = await self.embedding_service.generate_embedding(query_text)

            # Perform semantic search in Qdrant
            search_results = await self.qdrant_service.search(
                query_vector=query_embedding,
                top_k=top_k,
                min_score=min_score,
                filters=filters
            )

            # Convert to SearchResult objects
            result_objects = []
            for result in search_results:
                search_result = SearchResult(
                    id=result["id"],
                    score=result["score"],
                    content=result["content"],
                    metadata=result["metadata"],
                    query_id=result.get("query_id", "")
                )
                result_objects.append(search_result)

            return result_objects

        except Exception as e:
            logger.error(f"Search validation failed: {str(e)}")
            raise

    async def validate_url_filtering(
        self,
        query_text: str,
        source_url: str,
        top_k: int = 5
    ) -> List[SearchResult]:
        """
        Validate URL filtering functionality
        """
        try:
            results = await self.search_validation(
                query_text=query_text,
                top_k=top_k,
                filters={"source_url": source_url}
            )

            # Verify all results are from the specified URL
            for result in results:
                if result.metadata.url != source_url:
                    logger.warning(f"Filtering issue: result from {result.metadata.url}, expected {source_url}")

            return results
        except Exception as e:
            logger.error(f"URL filtering validation failed: {str(e)}")
            raise

    async def validate_section_filtering(
        self,
        query_text: str,
        section: str,
        top_k: int = 5
    ) -> List[SearchResult]:
        """
        Validate section filtering functionality
        """
        try:
            results = await self.search_validation(
                query_text=query_text,
                top_k=top_k,
                filters={"section": section}
            )

            # Verify all results are from the specified section
            for result in results:
                if result.metadata.section != section:
                    logger.warning(f"Filtering issue: result from {result.metadata.section}, expected {section}")

            return results
        except Exception as e:
            logger.error(f"Section filtering validation failed: {str(e)}")
            raise