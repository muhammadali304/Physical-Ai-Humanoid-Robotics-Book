from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Optional, Dict, Any
import logging
from ..config import settings


logger = logging.getLogger(__name__)


class QdrantService:
    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False  # Using REST API for validation
        )
        self.collection_name = settings.qdrant_collection_name

    async def search(
        self,
        query_vector: List[float],
        top_k: int = 5,
        min_score: float = 0.0,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Perform semantic search in Qdrant collection
        """
        try:
            # Build filters if provided
            qdrant_filters = None
            if filters:
                filter_conditions = []

                if "source_url" in filters and filters["source_url"]:
                    filter_conditions.append(
                        models.FieldCondition(
                            key="metadata.url",
                            match=models.MatchValue(value=filters["source_url"])
                        )
                    )

                if "section" in filters and filters["section"]:
                    filter_conditions.append(
                        models.FieldCondition(
                            key="metadata.section",
                            match=models.MatchValue(value=filters["section"])
                        )
                    )

                if filter_conditions:
                    qdrant_filters = models.Filter(must=filter_conditions)

            # Perform search
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                score_threshold=min_score,
                query_filter=qdrant_filters
            )

            # Format results
            formatted_results = []
            for hit in search_results:
                formatted_result = {
                    "id": hit.id,
                    "score": hit.score,
                    "content": hit.payload.get("content", ""),
                    "metadata": hit.payload.get("metadata", {}),
                    "query_id": None  # Will be set by caller
                }
                formatted_results.append(formatted_result)

            return formatted_results

        except Exception as e:
            logger.error(f"Qdrant search failed: {str(e)}")
            raise

    async def get_point(self, point_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific point from Qdrant by ID
        """
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[point_id]
            )

            if records:
                record = records[0]
                return {
                    "id": record.id,
                    "content": record.payload.get("content", ""),
                    "metadata": record.payload.get("metadata", {})
                }
            return None
        except Exception as e:
            logger.error(f"Failed to retrieve point {point_id}: {str(e)}")
            raise

    async def validate_collection_exists(self) -> bool:
        """
        Check if the collection exists and is accessible
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return True
        except Exception as e:
            logger.error(f"Collection {self.collection_name} not accessible: {str(e)}")
            return False