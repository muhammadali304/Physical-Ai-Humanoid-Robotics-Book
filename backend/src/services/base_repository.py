"""
Base repository pattern for the RAG Ingestion Pipeline.
Provides a generic interface for data access operations.
"""

from abc import ABC, abstractmethod
from typing import TypeVar, Generic, Optional, List, Dict, Any, Type
from uuid import UUID
from datetime import datetime
from pydantic import BaseModel


# Type variable for the model
T = TypeVar('T', bound=BaseModel)
ID = TypeVar('ID', bound=UUID)


class BaseRepository(ABC, Generic[T, ID]):
    """Abstract base repository defining common data access operations."""

    @abstractmethod
    async def create(self, model: T) -> T:
        """Create a new instance of the model."""
        pass

    @abstractmethod
    async def get_by_id(self, id: ID) -> Optional[T]:
        """Get a model instance by its ID."""
        pass

    @abstractmethod
    async def update(self, id: ID, model: T) -> Optional[T]:
        """Update an existing model instance."""
        pass

    @abstractmethod
    async def delete(self, id: ID) -> bool:
        """Delete a model instance by its ID."""
        pass

    @abstractmethod
    async def list(self, filters: Optional[Dict[str, Any]] = None, limit: Optional[int] = None, offset: Optional[int] = None) -> List[T]:
        """List model instances with optional filters, limit, and offset."""
        pass

    @abstractmethod
    async def count(self, filters: Optional[Dict[str, Any]] = None) -> int:
        """Count the number of model instances matching filters."""
        pass


class InMemoryRepository(BaseRepository[T, ID]):
    """
    In-memory implementation of the base repository pattern.
    Useful for testing and development.
    """

    def __init__(self, model_class: Type[T]):
        self.model_class = model_class
        self._data: Dict[ID, T] = {}
        self._next_id: int = 1

    async def create(self, model: T) -> T:
        """Create a new instance in memory."""
        # For UUID-based models, we assume the ID is already set
        # If not, we could generate one here
        self._data[model.id] = model
        return model

    async def get_by_id(self, id: ID) -> Optional[T]:
        """Get a model instance by its ID from memory."""
        return self._data.get(id)

    async def update(self, id: ID, model: T) -> Optional[T]:
        """Update an existing model instance in memory."""
        if id in self._data:
            self._data[id] = model
            return model
        return None

    async def delete(self, id: ID) -> bool:
        """Delete a model instance by its ID from memory."""
        if id in self._data:
            del self._data[id]
            return True
        return False

    async def list(self, filters: Optional[Dict[str, Any]] = None, limit: Optional[int] = None, offset: Optional[int] = None) -> List[T]:
        """List model instances from memory with optional filters."""
        items = list(self._data.values())

        # Apply filters if provided
        if filters:
            filtered_items = []
            for item in items:
                match = True
                for key, value in filters.items():
                    item_value = getattr(item, key, None)
                    if item_value != value:
                        match = False
                        break
                if match:
                    filtered_items.append(item)
            items = filtered_items

        # Apply offset
        if offset:
            items = items[offset:]

        # Apply limit
        if limit:
            items = items[:limit]

        return items

    async def count(self, filters: Optional[Dict[str, Any]] = None) -> int:
        """Count the number of model instances in memory."""
        if not filters:
            return len(self._data)

        # Count with filters applied
        count = 0
        for item in self._data.values():
            match = True
            for key, value in filters.items():
                item_value = getattr(item, key, None)
                if item_value != value:
                    match = False
                    break
            if match:
                count += 1

        return count


class QdrantRepository(BaseRepository[T, ID]):
    """
    Qdrant-based implementation of the base repository pattern.
    Used for vector storage operations.
    """

    def __init__(self, collection_name: str, model_class: Type[T]):
        self.collection_name = collection_name
        self.model_class = model_class
        # Note: Actual Qdrant client would be injected here
        # from src.services.qdrant_client import QdrantClient
        # self.qdrant_client = qdrant_client

    async def create(self, model: T) -> T:
        """
        Create a new instance in Qdrant.
        For vector data, this would upsert a point with the embedding.
        """
        # This would use the Qdrant client to store the embedding
        # Implementation would depend on the specific model type
        raise NotImplementedError("QdrantRepository.create not implemented yet")

    async def get_by_id(self, id: ID) -> Optional[T]:
        """
        Get a model instance by its ID from Qdrant.
        """
        # This would query Qdrant for the specific point by ID
        raise NotImplementedError("QdrantRepository.get_by_id not implemented yet")

    async def update(self, id: ID, model: T) -> Optional[T]:
        """
        Update an existing model instance in Qdrant.
        """
        # This would update the specific point in Qdrant
        raise NotImplementedError("QdrantRepository.update not implemented yet")

    async def delete(self, id: ID) -> bool:
        """
        Delete a model instance by its ID from Qdrant.
        """
        # This would delete the specific point from Qdrant
        raise NotImplementedError("QdrantRepository.delete not implemented yet")

    async def list(self, filters: Optional[Dict[str, Any]] = None, limit: Optional[int] = None, offset: Optional[int] = None) -> List[T]:
        """
        List model instances from Qdrant with optional filters.
        """
        # This would perform a scroll operation in Qdrant
        raise NotImplementedError("QdrantRepository.list not implemented yet")

    async def count(self, filters: Optional[Dict[str, Any]] = None) -> int:
        """
        Count the number of model instances in Qdrant.
        """
        # This would use Qdrant's count API
        raise NotImplementedError("QdrantRepository.count not implemented yet")


class RepositoryFactory:
    """Factory for creating repository instances."""

    @staticmethod
    def create_repository(model_class: Type[T], repository_type: str = "memory", **kwargs) -> BaseRepository[T, UUID]:
        """
        Create a repository instance based on the specified type.

        Args:
            model_class: The Pydantic model class for the repository
            repository_type: Type of repository ('memory' or 'qdrant')
            **kwargs: Additional arguments for repository initialization
        """
        if repository_type == "memory":
            return InMemoryRepository(model_class)
        elif repository_type == "qdrant":
            collection_name = kwargs.get("collection_name")
            if not collection_name:
                raise ValueError("collection_name is required for Qdrant repository")
            return QdrantRepository(collection_name, model_class)
        else:
            raise ValueError(f"Unsupported repository type: {repository_type}")