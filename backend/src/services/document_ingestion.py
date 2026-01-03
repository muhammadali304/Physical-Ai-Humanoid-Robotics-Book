"""
Document ingestion service for the RAG Agent Backend.

This module provides functionality for ingesting technical documentation
following the implementation plan requirements for User Story 2.
"""

import asyncio
import hashlib
from typing import List, Optional, Dict, Any
from pydantic import BaseModel
import logging
from pathlib import Path
import aiofiles
from src.models.document import DocumentChunk, DocumentMetadata
from src.services.document_storage import DocumentStorageService
from src.services.qdrant_client import QdrantClientService
from src.services.embedding_service import CohereEmbeddingService
from src.utils.errors import DocumentProcessingError


class DocumentIngestionService:
    """Service class for ingesting documents into the RAG system"""

    def __init__(self):
        """Initialize the document ingestion service"""
        self.logger = logging.getLogger(__name__)
        self.document_storage = DocumentStorageService()
        self.qdrant_client = QdrantClientService()
        self.embedding_service = CohereEmbeddingService()
        self.max_file_size = 10 * 1024 * 1024  # 10MB max file size
        self.supported_formats = ['.pdf', '.txt', '.docx', '.html', '.md']

    async def ingest_document(self,
                            file_path: str,
                            document_metadata: Optional[DocumentMetadata] = None,
                            chunk_size: int = 1000,
                            overlap: int = 200) -> bool:
        """
        Ingest a document and store it in the RAG system.

        Args:
            file_path: Path to the document file to ingest
            document_metadata: Optional metadata about the document
            chunk_size: Size of text chunks for processing
            overlap: Overlap between chunks for context continuity

        Returns:
            True if ingestion was successful, False otherwise
        """
        try:
            self.logger.info(f"Starting ingestion of document: {file_path}")

            # Validate file
            if not await self._validate_file(file_path):
                raise DocumentProcessingError(f"Invalid file: {file_path}")

            # Extract text from document
            text_content = await self._extract_text_from_file(file_path)
            if not text_content or len(text_content.strip()) == 0:
                raise DocumentProcessingError(f"Document is empty: {file_path}")

            # Generate document ID
            document_id = self._generate_document_id(file_path, text_content)

            # Create metadata if not provided
            if document_metadata is None:
                document_metadata = DocumentMetadata(
                    document_id=document_id,
                    source_url=file_path,
                    title=Path(file_path).stem,
                    file_size=len(text_content.encode('utf-8')),
                    page_count=1,  # Will be updated based on actual content
                    content_type=Path(file_path).suffix,
                    created_at=None,
                    updated_at=None
                )

            # Chunk the document
            chunks = await self._chunk_document(
                text_content,
                document_id,
                document_metadata,
                chunk_size,
                overlap
            )

            # Generate embeddings for chunks
            chunk_embeddings = await self._generate_embeddings_for_chunks(chunks)

            # Store chunks in the document storage
            success = await self.document_storage.store_document_chunks(chunks, chunk_embeddings)

            if success:
                self.logger.info(f"Successfully ingested document: {document_id} with {len(chunks)} chunks")
            else:
                self.logger.error(f"Failed to store document chunks for: {document_id}")

            return success

        except DocumentProcessingError:
            # Re-raise document processing errors
            raise
        except Exception as e:
            self.logger.error(f"Error ingesting document {file_path}: {str(e)}")
            raise DocumentProcessingError(f"Failed to ingest document: {str(e)}")

    async def _validate_file(self, file_path: str) -> bool:
        """Validate the file before ingestion"""
        try:
            path = Path(file_path)

            # Check if file exists
            if not path.exists():
                self.logger.error(f"File does not exist: {file_path}")
                return False

            # Check file size
            if path.stat().st_size > self.max_file_size:
                self.logger.error(f"File too large: {file_path} ({path.stat().st_size} bytes)")
                return False

            # Check file extension
            if path.suffix.lower() not in self.supported_formats:
                self.logger.error(f"Unsupported file format: {path.suffix} for {file_path}")
                return False

            return True

        except Exception as e:
            self.logger.error(f"Error validating file {file_path}: {str(e)}")
            return False

    async def _extract_text_from_file(self, file_path: str) -> str:
        """Extract text content from various file formats"""
        try:
            path = Path(file_path)
            file_ext = path.suffix.lower()

            if file_ext == '.txt':
                async with aiofiles.open(file_path, 'r', encoding='utf-8') as f:
                    content = await f.read()
            elif file_ext == '.pdf':
                # For PDF files, we would use a PDF library like PyPDF2 or pdfplumber
                # Since we don't have those installed, we'll simulate the functionality
                content = await self._extract_text_from_pdf(file_path)
            elif file_ext == '.docx':
                # For DOCX files, we would use python-docx
                content = await self._extract_text_from_docx(file_path)
            elif file_ext in ['.html', '.htm']:
                # For HTML files, we would use BeautifulSoup
                content = await self._extract_text_from_html(file_path)
            elif file_ext == '.md':
                async with aiofiles.open(file_path, 'r', encoding='utf-8') as f:
                    content = await f.read()
            else:
                raise DocumentProcessingError(f"Unsupported file format: {file_ext}")

            return content

        except Exception as e:
            self.logger.error(f"Error extracting text from {file_path}: {str(e)}")
            raise DocumentProcessingError(f"Failed to extract text from document: {str(e)}")

    async def _extract_text_from_pdf(self, file_path: str) -> str:
        """Extract text from PDF file (simulated implementation)"""
        # This is a simulated implementation - in a real system you would use PyPDF2 or similar
        # For now, we'll just read the file and return its content as text
        # In a real implementation, you would properly parse the PDF
        async with aiofiles.open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            content = await f.read()
        return content

    async def _extract_text_from_docx(self, file_path: str) -> str:
        """Extract text from DOCX file (simulated implementation)"""
        # This is a simulated implementation - in a real system you would use python-docx
        # For now, we'll just read the file and return its content as text
        async with aiofiles.open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            content = await f.read()
        return content

    async def _extract_text_from_html(self, file_path: str) -> str:
        """Extract text from HTML file (simulated implementation)"""
        # This is a simulated implementation - in a real system you would use BeautifulSoup
        # For now, we'll just read the file and return its content as text
        async with aiofiles.open(file_path, 'r', encoding='utf-8') as f:
            content = await f.read()
        # In a real implementation, you would parse HTML and extract text content
        return content

    def _generate_document_id(self, file_path: str, content: str) -> str:
        """Generate a unique document ID based on file path and content"""
        content_hash = hashlib.sha256(content.encode('utf-8')).hexdigest()[:16]
        path_hash = hashlib.sha256(file_path.encode('utf-8')).hexdigest()[:16]
        return f"{path_hash}_{content_hash}"

    async def _chunk_document(self,
                             text_content: str,
                             document_id: str,
                             metadata: DocumentMetadata,
                             chunk_size: int,
                             overlap: int) -> List[DocumentChunk]:
        """Chunk the document text into smaller pieces"""
        try:
            self.logger.info(f"Chunking document: {document_id} with chunk_size={chunk_size}, overlap={overlap}")

            # Split the text into chunks
            chunks = []
            start = 0
            chunk_index = 0

            while start < len(text_content):
                # Determine the end position for this chunk
                end = start + chunk_size

                # If we're near the end, make sure to include the remainder
                if end > len(text_content):
                    end = len(text_content)

                # Extract the chunk text
                chunk_text = text_content[start:end]

                # Create a document chunk
                chunk = DocumentChunk(
                    chunk_id=f"{document_id}_chunk_{chunk_index}",
                    document_id=document_id,
                    content=chunk_text,
                    chunk_index=chunk_index,
                    metadata=metadata
                )

                chunks.append(chunk)

                # Move to the next chunk position (with overlap)
                start = end - overlap
                chunk_index += 1

            self.logger.info(f"Created {len(chunks)} chunks for document: {document_id}")
            return chunks

        except Exception as e:
            self.logger.error(f"Error chunking document {document_id}: {str(e)}")
            raise DocumentProcessingError(f"Failed to chunk document: {str(e)}")

    async def _generate_embeddings_for_chunks(self, chunks: List[DocumentChunk]) -> List[List[float]]:
        """Generate embeddings for document chunks"""
        try:
            self.logger.info(f"Generating embeddings for {len(chunks)} chunks")

            embeddings = []
            for chunk in chunks:
                # Generate embedding for the chunk content
                embedding = await self.embedding_service.generate_embedding(chunk.content)
                if embedding:
                    embeddings.append(embedding)
                else:
                    self.logger.warning(f"Failed to generate embedding for chunk: {chunk.chunk_id}")
                    # Use a zero vector as fallback
                    embeddings.append([0.0] * 1024)  # Assuming 1024-dim embedding

            self.logger.info(f"Generated embeddings for {len(embeddings)} chunks")
            return embeddings

        except Exception as e:
            self.logger.error(f"Error generating embeddings: {str(e)}")
            raise DocumentProcessingError(f"Failed to generate embeddings: {str(e)}")

    async def ingest_multiple_documents(self,
                                      file_paths: List[str],
                                      chunk_size: int = 1000,
                                      overlap: int = 200) -> Dict[str, bool]:
        """
        Ingest multiple documents at once.

        Args:
            file_paths: List of file paths to ingest
            chunk_size: Size of text chunks for processing
            overlap: Overlap between chunks for context continuity

        Returns:
            Dictionary mapping file paths to ingestion success status
        """
        try:
            self.logger.info(f"Starting ingestion of {len(file_paths)} documents")

            results = {}
            for file_path in file_paths:
                try:
                    success = await self.ingest_document(file_path, chunk_size=chunk_size, overlap=overlap)
                    results[file_path] = success
                    if success:
                        self.logger.info(f"Successfully ingested: {file_path}")
                    else:
                        self.logger.error(f"Failed to ingest: {file_path}")
                except Exception as e:
                    self.logger.error(f"Error ingesting {file_path}: {str(e)}")
                    results[file_path] = False

            success_count = sum(1 for success in results.values() if success)
            self.logger.info(f"Completed ingestion: {success_count}/{len(file_paths)} successful")

            return results

        except Exception as e:
            self.logger.error(f"Error in batch ingestion: {str(e)}")
            raise DocumentProcessingError(f"Failed to process batch ingestion: {str(e)}")

    async def get_ingestion_status(self, document_id: str) -> Dict[str, Any]:
        """Get the ingestion status for a specific document"""
        try:
            self.logger.info(f"Getting ingestion status for document: {document_id}")

            # Check if document chunks exist in storage
            chunks = await self.document_storage.get_document_chunks(document_id)

            status = {
                "document_id": document_id,
                "ingested": len(chunks) > 0,
                "chunk_count": len(chunks),
                "status": "completed" if len(chunks) > 0 else "not_found"
            }

            self.logger.info(f"Ingestion status for {document_id}: {status}")
            return status

        except Exception as e:
            self.logger.error(f"Error getting ingestion status for {document_id}: {str(e)}")
            return {
                "document_id": document_id,
                "ingested": False,
                "chunk_count": 0,
                "status": "error",
                "error": str(e)
            }

    async def delete_document(self, document_id: str) -> bool:
        """Delete a document and its chunks from the system"""
        try:
            self.logger.info(f"Deleting document: {document_id}")

            # Delete document chunks from storage
            success = await self.document_storage.delete_document_chunks(document_id)

            if success:
                self.logger.info(f"Successfully deleted document: {document_id}")
            else:
                self.logger.error(f"Failed to delete document: {document_id}")

            return success

        except Exception as e:
            self.logger.error(f"Error deleting document {document_id}: {str(e)}")
            return False

    async def health_check(self) -> bool:
        """Check if the document ingestion service is healthy"""
        try:
            # Test the embedding service
            test_embedding = await self.embedding_service.generate_embedding("test")
            embedding_healthy = test_embedding is not None

            # Test the document storage
            storage_healthy = await self.document_storage.health_check()

            # Test the Qdrant client
            qdrant_healthy = await self.qdrant_client.health_check()

            overall_healthy = embedding_healthy and storage_healthy and qdrant_healthy
            self.logger.info(f"Document ingestion service health check: {overall_healthy}")

            return overall_healthy

        except Exception as e:
            self.logger.error(f"Document ingestion service health check failed: {str(e)}")
            return False


# Global document ingestion service instance
document_ingestion_service = DocumentIngestionService()


def get_document_ingestion_service() -> DocumentIngestionService:
    """Get the global document ingestion service instance"""
    return document_ingestion_service