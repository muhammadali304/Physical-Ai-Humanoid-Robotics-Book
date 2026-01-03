from typing import List, Dict, Any, Optional
from ..models.validation_models import SearchResult, ResultMetadata


class MetadataService:
    """
    Service for validating metadata integrity and completeness
    """

    def validate_url_integrity(self, results: List[SearchResult], expected_url: Optional[str] = None) -> Dict[str, float]:
        """
        Validate URL integrity in search results
        """
        if not results:
            return {
                "integrity_score": 0.0,
                "valid_count": 0,
                "total_count": 0
            }

        total_count = len(results)
        valid_count = 0

        for result in results:
            if result.metadata and result.metadata.url:
                if expected_url and result.metadata.url != expected_url:
                    continue  # Skip if URL doesn't match expected
                valid_count += 1

        integrity_score = valid_count / total_count if total_count > 0 else 0.0

        return {
            "integrity_score": integrity_score,
            "valid_count": valid_count,
            "total_count": total_count
        }

    def validate_section_integrity(self, results: List[SearchResult], expected_section: Optional[str] = None) -> Dict[str, float]:
        """
        Validate section integrity in search results
        """
        if not results:
            return {
                "integrity_score": 0.0,
                "valid_count": 0,
                "total_count": 0
            }

        total_count = len(results)
        valid_count = 0

        for result in results:
            if result.metadata and result.metadata.section:
                if expected_section and result.metadata.section != expected_section:
                    continue  # Skip if section doesn't match expected
                valid_count += 1

        integrity_score = valid_count / total_count if total_count > 0 else 0.0

        return {
            "integrity_score": integrity_score,
            "valid_count": valid_count,
            "total_count": total_count
        }

    def validate_chunk_index_integrity(self, results: List[SearchResult]) -> Dict[str, float]:
        """
        Validate chunk index integrity in search results
        """
        if not results:
            return {
                "integrity_score": 0.0,
                "valid_count": 0,
                "total_count": 0
            }

        total_count = len(results)
        valid_count = 0

        for result in results:
            if result.metadata and result.metadata.chunk_index is not None:
                # Validate that chunk_index is a valid integer
                if isinstance(result.metadata.chunk_index, int) and result.metadata.chunk_index >= 0:
                    valid_count += 1

        integrity_score = valid_count / total_count if total_count > 0 else 0.0

        return {
            "integrity_score": integrity_score,
            "valid_count": valid_count,
            "total_count": total_count
        }

    def validate_source_title_integrity(self, results: List[SearchResult]) -> Dict[str, float]:
        """
        Validate source title integrity in search results
        """
        if not results:
            return {
                "integrity_score": 0.0,
                "valid_count": 0,
                "total_count": 0
            }

        total_count = len(results)
        valid_count = 0

        for result in results:
            if result.metadata and result.metadata.source_title:
                # Validate that source_title is a non-empty string
                if isinstance(result.metadata.source_title, str) and result.metadata.source_title.strip():
                    valid_count += 1

        integrity_score = valid_count / total_count if total_count > 0 else 0.0

        return {
            "integrity_score": integrity_score,
            "valid_count": valid_count,
            "total_count": total_count
        }

    def calculate_metadata_completeness(self, results: List[SearchResult]) -> Dict[str, float]:
        """
        Calculate overall metadata completeness score
        """
        if not results:
            return {
                "completeness_score": 0.0,
                "required_fields_present": 0.0,
                "optional_fields_present": 0.0
            }

        total_results = len(results)
        total_required_fields = 0
        total_optional_fields = 0
        max_required_fields = 0
        max_optional_fields = 0

        # Define required and optional fields
        required_fields = ["url", "section", "chunk_index"]
        optional_fields = ["source_title"]

        for result in results:
            if result.metadata:
                for field in required_fields:
                    if hasattr(result.metadata, field) and getattr(result.metadata, field) is not None:
                        total_required_fields += 1
                max_required_fields += len(required_fields)

                for field in optional_fields:
                    if hasattr(result.metadata, field) and getattr(result.metadata, field) is not None:
                        total_optional_fields += 1
                max_optional_fields += len(optional_fields)

        completeness_score = 0.0
        if max_required_fields > 0:
            required_score = total_required_fields / max_required_fields
            optional_score = total_optional_fields / max_optional_fields if max_optional_fields > 0 else 0.0
            # Weight required fields more heavily (70%) vs optional fields (30%)
            completeness_score = (required_score * 0.7) + (optional_score * 0.3)

        return {
            "completeness_score": completeness_score,
            "required_fields_present": total_required_fields / max_required_fields if max_required_fields > 0 else 0.0,
            "optional_fields_present": total_optional_fields / max_optional_fields if max_optional_fields > 0 else 0.0
        }

    def validate_metadata_consistency(self, results: List[SearchResult]) -> Dict[str, Any]:
        """
        Validate consistency of metadata across results
        """
        if not results:
            return {
                "consistency_score": 0.0,
                "inconsistencies": []
            }

        inconsistencies = []
        urls = []
        sections = []

        for result in results:
            if result.metadata:
                if result.metadata.url:
                    urls.append(result.metadata.url)
                if result.metadata.section:
                    sections.append(result.metadata.section)

        # Check for consistency in source URLs
        unique_urls = set(urls) if urls else set()
        if len(unique_urls) > 1:
            inconsistencies.append(f"Multiple source URLs detected: {list(unique_urls)}")

        # Check for consistency in sections
        unique_sections = set(sections) if sections else set()
        if len(unique_sections) > 1:
            inconsistencies.append(f"Multiple sections detected: {list(unique_sections)}")

        # Calculate consistency score (simplified)
        consistency_score = 1.0
        if len(unique_urls) > 1:
            consistency_score -= 0.2  # Penalty for multiple URLs
        if len(unique_sections) > 1:
            consistency_score -= 0.2  # Penalty for multiple sections

        consistency_score = max(0.0, consistency_score)  # Ensure non-negative

        return {
            "consistency_score": consistency_score,
            "inconsistencies": inconsistencies
        }