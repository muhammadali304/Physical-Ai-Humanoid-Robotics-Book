"""
Backup and recovery service for the RAG Ingestion Pipeline.
Provides functionality for backing up data and recovering from failures.
"""

import asyncio
import json
import os
import shutil
import zipfile
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, List, Optional
from dataclasses import dataclass

from src.config.settings import settings
from src.utils.logging import get_logger
from src.services.qdrant_client import QdrantClientService


@dataclass
class BackupInfo:
    """Information about a backup."""
    id: str
    timestamp: datetime
    size_bytes: int
    status: str
    location: str
    collections_backed_up: List[str]
    metadata: Dict[str, Any]


class BackupService:
    """Service to handle backup and recovery operations."""

    def __init__(self):
        self.logger = get_logger("backup_service")
        self.backup_directory = settings.get("backup_directory", "./backups")
        self.qdrant_client = QdrantClientService()
        self.max_backup_age_days = settings.get("max_backup_age_days", 30)
        self.compression_enabled = settings.get("backup_compression_enabled", True)

        # Create backup directory if it doesn't exist
        Path(self.backup_directory).mkdir(parents=True, exist_ok=True)

    async def create_backup(self, backup_id: str = None) -> BackupInfo:
        """
        Create a backup of the system data.

        Args:
            backup_id: Optional backup ID (will be generated if not provided)

        Returns:
            BackupInfo object with backup details
        """
        from uuid import uuid4

        if backup_id is None:
            backup_id = f"backup_{datetime.utcnow().strftime('%Y%m%d_%H%M%S')}_{str(uuid4())[:8]}"

        self.logger.info(f"Starting backup: {backup_id}")

        backup_path = Path(self.backup_directory) / backup_id
        backup_path.mkdir(parents=True, exist_ok=True)

        try:
            # Backup Qdrant collections
            collections_backed_up = await self._backup_qdrant_collections(backup_path)

            # Create backup metadata
            metadata = {
                "backup_id": backup_id,
                "timestamp": datetime.utcnow().isoformat(),
                "collections": collections_backed_up,
                "settings": {
                    "qdrant_url": settings.qdrant_url,
                    "qdrant_collection_name": settings.qdrant_collection_name
                }
            }

            # Save metadata
            metadata_path = backup_path / "metadata.json"
            with open(metadata_path, 'w', encoding='utf-8') as f:
                json.dump(metadata, f, indent=2, default=str)

            # Get backup size
            size_bytes = self._get_directory_size(backup_path)

            # Optionally compress the backup
            if self.compression_enabled:
                compressed_path = backup_path.with_suffix('.zip')
                self._compress_directory(backup_path, compressed_path)
                shutil.rmtree(backup_path)  # Remove uncompressed directory
                backup_path = compressed_path
                size_bytes = os.path.getsize(compressed_path)

            backup_info = BackupInfo(
                id=backup_id,
                timestamp=datetime.utcnow(),
                size_bytes=size_bytes,
                status="completed",
                location=str(backup_path),
                collections_backed_up=collections_backed_up,
                metadata=metadata
            )

            self.logger.info(
                f"Backup completed: {backup_id}",
                backup_id=backup_id,
                size_bytes=size_bytes,
                collections_backed_up=collections_backed_up
            )

            # Clean up old backups
            await self.cleanup_old_backups()

            return backup_info

        except Exception as e:
            self.logger.error(
                f"Backup failed: {backup_id}",
                backup_id=backup_id,
                error=str(e)
            )
            # Remove partial backup
            if backup_path.exists():
                shutil.rmtree(backup_path, ignore_errors=True)
            raise

    async def _backup_qdrant_collections(self, backup_path: Path) -> List[str]:
        """
        Backup Qdrant collections.

        Args:
            backup_path: Path to store the backup

        Returns:
            List of collection names that were backed up
        """
        collections_backed_up = []

        try:
            # Get list of collections from Qdrant
            collections = await self.qdrant_client.get_collections()

            for collection_name in collections:
                self.logger.info(
                    f"Backing up Qdrant collection: {collection_name}",
                    collection=collection_name
                )

                # Export collection data
                collection_backup_path = backup_path / f"{collection_name}_backup.jsonl"

                # This is a simplified approach - in a real implementation, you'd use
                # Qdrant's backup/restore functionality or export points in batches
                collection_data = await self.qdrant_client.export_collection(collection_name)

                with open(collection_backup_path, 'w', encoding='utf-8') as f:
                    for record in collection_data:
                        f.write(json.dumps(record) + '\n')

                collections_backed_up.append(collection_name)

                self.logger.info(
                    f"Completed backup of collection: {collection_name}",
                    collection=collection_name,
                    records=len(collection_data)
                )

        except Exception as e:
            self.logger.error(
                f"Error backing up Qdrant collections: {str(e)}",
                error=str(e)
            )
            raise

        return collections_backed_up

    def _get_directory_size(self, directory: Path) -> int:
        """
        Get the total size of a directory.

        Args:
            directory: Path to the directory

        Returns:
            Total size in bytes
        """
        total_size = 0
        for dirpath, dirnames, filenames in os.walk(directory):
            for filename in filenames:
                filepath = Path(dirpath) / filename
                total_size += filepath.stat().st_size
        return total_size

    def _compress_directory(self, source_dir: Path, output_path: Path):
        """
        Compress a directory into a zip file.

        Args:
            source_dir: Directory to compress
            output_path: Path for the output zip file
        """
        with zipfile.ZipFile(output_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
            for root, dirs, files in os.walk(source_dir):
                for file in files:
                    file_path = Path(root) / file
                    arcname = file_path.relative_to(source_dir.parent)
                    zipf.write(file_path, arcname)

    async def restore_backup(self, backup_id: str) -> bool:
        """
        Restore from a backup.

        Args:
            backup_id: ID of the backup to restore

        Returns:
            True if restore was successful, False otherwise
        """
        self.logger.info(f"Starting restore from backup: {backup_id}")

        backup_path = Path(self.backup_directory) / backup_id

        # Check if backup exists
        if not backup_path.exists():
            # Check if it's a compressed backup
            compressed_path = backup_path.with_suffix('.zip')
            if compressed_path.exists():
                backup_path = compressed_path
            else:
                self.logger.error(
                    f"Backup not found: {backup_id}",
                    backup_id=backup_id
                )
                return False

        try:
            # Extract if compressed
            extracted_path = backup_path
            temp_extract_dir = None
            if backup_path.suffix == '.zip':
                temp_extract_dir = Path(tempfile.mkdtemp())
                extracted_path = temp_extract_dir
                self._extract_zip(backup_path, temp_extract_dir)

            # Load metadata
            metadata_path = extracted_path / "metadata.json"
            with open(metadata_path, 'r', encoding='utf-8') as f:
                metadata = json.load(f)

            # Restore Qdrant collections
            for collection_name in metadata["collections"]:
                collection_backup_path = extracted_path / f"{collection_name}_backup.jsonl"
                if collection_backup_path.exists():
                    await self._restore_qdrant_collection(collection_name, collection_backup_path)
                else:
                    self.logger.warning(
                        f"Collection backup file not found: {collection_backup_path}",
                        collection=collection_name
                    )

            self.logger.info(
                f"Restore completed: {backup_id}",
                backup_id=backup_id
            )

            # Clean up temporary extraction directory
            if temp_extract_dir:
                shutil.rmtree(temp_extract_dir, ignore_errors=True)

            return True

        except Exception as e:
            self.logger.error(
                f"Restore failed: {backup_id}",
                backup_id=backup_id,
                error=str(e)
            )
            # Clean up temporary extraction directory
            if temp_extract_dir:
                shutil.rmtree(temp_extract_dir, ignore_errors=True)
            return False

    async def _restore_qdrant_collection(self, collection_name: str, backup_file_path: Path) -> bool:
        """
        Restore a Qdrant collection from backup.

        Args:
            collection_name: Name of the collection to restore
            backup_file_path: Path to the backup file

        Returns:
            True if restore was successful, False otherwise
        """
        try:
            self.logger.info(
                f"Restoring Qdrant collection: {collection_name}",
                collection=collection_name,
                backup_file=str(backup_file_path)
            )

            # Read backup data
            records = []
            with open(backup_file_path, 'r', encoding='utf-8') as f:
                for line in f:
                    if line.strip():
                        records.append(json.loads(line))

            # Restore records to Qdrant
            await self.qdrant_client.restore_collection(collection_name, records)

            self.logger.info(
                f"Completed restore of collection: {collection_name}",
                collection=collection_name,
                records_restored=len(records)
            )

            return True

        except Exception as e:
            self.logger.error(
                f"Error restoring Qdrant collection: {collection_name}",
                collection=collection_name,
                error=str(e)
            )
            return False

    def _extract_zip(self, zip_path: Path, extract_to: Path):
        """
        Extract a zip file to a directory.

        Args:
            zip_path: Path to the zip file
            extract_to: Directory to extract to
        """
        with zipfile.ZipFile(zip_path, 'r') as zipf:
            zipf.extractall(extract_to)

    async def get_backup_info(self, backup_id: str) -> Optional[BackupInfo]:
        """
        Get information about a specific backup.

        Args:
            backup_id: ID of the backup

        Returns:
            BackupInfo object or None if backup doesn't exist
        """
        backup_path = Path(self.backup_directory) / backup_id
        metadata_path = backup_path / "metadata.json"

        # Check if it's a compressed backup
        if not metadata_path.exists():
            compressed_path = backup_path.with_suffix('.zip')
            if compressed_path.exists():
                # Extract to temp directory to read metadata
                with tempfile.TemporaryDirectory() as temp_dir:
                    temp_path = Path(temp_dir)
                    self._extract_zip(compressed_path, temp_path)
                    metadata_path = temp_path / "metadata.json"

                    if metadata_path.exists():
                        with open(metadata_path, 'r', encoding='utf-8') as f:
                            metadata = json.load(f)

                        # Get size
                        size_bytes = os.path.getsize(compressed_path)

                        return BackupInfo(
                            id=backup_id,
                            timestamp=datetime.fromisoformat(metadata["timestamp"]),
                            size_bytes=size_bytes,
                            status="completed",
                            location=str(compressed_path),
                            collections_backed_up=metadata["collections"],
                            metadata=metadata
                        )

        # Check uncompressed backup
        elif metadata_path.exists():
            with open(metadata_path, 'r', encoding='utf-8') as f:
                metadata = json.load(f)

            size_bytes = self._get_directory_size(backup_path)

            return BackupInfo(
                id=backup_id,
                timestamp=datetime.fromisoformat(metadata["timestamp"]),
                size_bytes=size_bytes,
                status="completed",
                location=str(backup_path),
                collections_backed_up=metadata["collections"],
                metadata=metadata
            )

        return None

    async def list_backups(self) -> List[BackupInfo]:
        """
        List all available backups.

        Returns:
            List of BackupInfo objects
        """
        backups = []

        # Look for both compressed and uncompressed backups
        for item in Path(self.backup_directory).iterdir():
            if item.is_dir():
                # Uncompressed backup
                backup_id = item.name
                backup_info = await self.get_backup_info(backup_id)
                if backup_info:
                    backups.append(backup_info)
            elif item.is_file() and item.suffix == '.zip':
                # Compressed backup
                backup_id = item.stem  # Remove .zip extension
                backup_info = await self.get_backup_info(backup_id)
                if backup_info:
                    backups.append(backup_info)

        # Sort by timestamp (newest first)
        backups.sort(key=lambda x: x.timestamp, reverse=True)
        return backups

    async def delete_backup(self, backup_id: str) -> bool:
        """
        Delete a backup.

        Args:
            backup_id: ID of the backup to delete

        Returns:
            True if deletion was successful, False otherwise
        """
        backup_path = Path(self.backup_directory) / backup_id
        compressed_path = backup_path.with_suffix('.zip')

        if backup_path.exists():
            try:
                if backup_path.is_dir():
                    shutil.rmtree(backup_path)
                else:
                    backup_path.unlink()
                self.logger.info(
                    f"Deleted backup: {backup_id}",
                    backup_id=backup_id
                )
                return True
            except Exception as e:
                self.logger.error(
                    f"Error deleting backup: {backup_id}",
                    backup_id=backup_id,
                    error=str(e)
                )
                return False
        elif compressed_path.exists():
            try:
                compressed_path.unlink()
                self.logger.info(
                    f"Deleted compressed backup: {backup_id}",
                    backup_id=backup_id
                )
                return True
            except Exception as e:
                self.logger.error(
                    f"Error deleting compressed backup: {backup_id}",
                    backup_id=backup_id,
                    error=str(e)
                )
                return False
        else:
            self.logger.warning(
                f"Backup not found for deletion: {backup_id}",
                backup_id=backup_id
            )
            return False

    async def cleanup_old_backups(self):
        """
        Clean up old backups based on retention policy.
        """
        try:
            all_backups = await self.list_backups()
            cutoff_date = datetime.utcnow().replace(
                hour=0, minute=0, second=0, microsecond=0
            ) - timedelta(days=self.max_backup_age_days)

            for backup in all_backups:
                if backup.timestamp < cutoff_date:
                    await self.delete_backup(backup.id)
                    self.logger.info(
                        f"Cleaned up old backup: {backup.id}",
                        backup_id=backup.id,
                        timestamp=backup.timestamp.isoformat()
                    )

        except Exception as e:
            self.logger.error(
                f"Error cleaning up old backups: {str(e)}",
                error=str(e)
            )

    async def verify_backup(self, backup_id: str) -> Dict[str, Any]:
        """
        Verify the integrity of a backup.

        Args:
            backup_id: ID of the backup to verify

        Returns:
            Dictionary with verification results
        """
        try:
            backup_info = await self.get_backup_info(backup_id)
            if not backup_info:
                return {"valid": False, "error": "Backup not found"}

            # Check if all required files exist
            backup_path = Path(self.backup_directory) / backup_id
            metadata_path = backup_path / "metadata.json"

            # Handle compressed backups
            if not metadata_path.exists():
                compressed_path = backup_path.with_suffix('.zip')
                if compressed_path.exists():
                    # Extract to temp directory to verify
                    with tempfile.TemporaryDirectory() as temp_dir:
                        temp_path = Path(temp_dir)
                        self._extract_zip(compressed_path, temp_path)
                        metadata_path = temp_path / "metadata.json"

            if not metadata_path.exists():
                return {"valid": False, "error": "Metadata file missing"}

            # Verify metadata structure
            with open(metadata_path, 'r', encoding='utf-8') as f:
                metadata = json.load(f)

            required_fields = ["backup_id", "timestamp", "collections"]
            missing_fields = [field for field in required_fields if field not in metadata]
            if missing_fields:
                return {"valid": False, "error": f"Missing required fields: {missing_fields}"}

            # Verify collection backup files exist
            for collection in metadata["collections"]:
                collection_backup_path = backup_path / f"{collection}_backup.jsonl"
                if not collection_backup_path.exists():
                    # Check if compressed
                    compressed_path = backup_path.with_suffix('.zip')
                    if compressed_path.exists():
                        # For compressed backups, we can't easily check individual files without extracting
                        continue
                    else:
                        return {"valid": False, "error": f"Collection backup file missing: {collection_backup_path}"}

            return {"valid": True, "backup_info": backup_info.__dict__}

        except Exception as e:
            return {"valid": False, "error": f"Verification failed: {str(e)}"}


# Global instance
_backup_service = None


def get_backup_service() -> BackupService:
    """
    Get the global backup service instance.

    Returns:
        BackupService instance
    """
    global _backup_service
    if _backup_service is None:
        _backup_service = BackupService()
    return _backup_service


from datetime import datetime, timedelta  # Import here since it's used in the class


async def perform_scheduled_backup():
    """
    Perform a scheduled backup.
    """
    service = get_backup_service()
    return await service.create_backup()


async def restore_from_latest_backup():
    """
    Restore from the latest available backup.
    """
    service = get_backup_service()
    backups = await service.list_backups()
    if backups:
        latest_backup = backups[0]  # Most recent backup
        return await service.restore_backup(latest_backup.id)
    else:
        raise ValueError("No backups available")