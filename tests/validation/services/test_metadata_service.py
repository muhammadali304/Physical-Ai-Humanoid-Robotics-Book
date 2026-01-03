import pytest
from backend.src.validation.services.metadata_service import MetadataService
from backend.src.validation.models.validation_models import SearchResult, ResultMetadata


class TestMetadataService:
    """
    Unit tests for MetadataService
    """

    @pytest.fixture
    def metadata_service(self):
        return MetadataService()

    def test_validate_url_integrity(self, metadata_service):
        """
        Test validate_url_integrity method
        """
        # Create mock results
        metadata1 = ResultMetadata(
            url="https://example.com",
            section="Section 1",
            chunk_index=1
        )
        metadata2 = ResultMetadata(
            url="https://example.com",
            section="Section 2",
            chunk_index=2
        )
        result1 = SearchResult(
            id="result_1",
            score=0.85,
            content="content 1",
            metadata=metadata1,
            query_id="query_1"
        )
        result2 = SearchResult(
            id="result_2",
            score=0.75,
            content="content 2",
            metadata=metadata2,
            query_id="query_1"
        )

        results = [result1, result2]
        validation = metadata_service.validate_url_integrity(results)

        assert validation["integrity_score"] == 1.0  # Both have valid URLs
        assert validation["valid_count"] == 2
        assert validation["total_count"] == 2

    def test_validate_url_integrity_with_expected_url(self, metadata_service):
        """
        Test validate_url_integrity with expected URL filter
        """
        metadata1 = ResultMetadata(
            url="https://example.com",
            section="Section 1",
            chunk_index=1
        )
        metadata2 = ResultMetadata(
            url="https://other.com",  # Different URL
            section="Section 2",
            chunk_index=2
        )
        result1 = SearchResult(
            id="result_1",
            score=0.85,
            content="content 1",
            metadata=metadata1,
            query_id="query_1"
        )
        result2 = SearchResult(
            id="result_2",
            score=0.75,
            content="content 2",
            metadata=metadata2,
            query_id="query_1"
        )

        results = [result1, result2]
        validation = metadata_service.validate_url_integrity(results, expected_url="https://example.com")

        assert validation["integrity_score"] == 0.5  # Only 1 out of 2 matches expected URL
        assert validation["valid_count"] == 1
        assert validation["total_count"] == 2

    def test_validate_url_integrity_empty(self, metadata_service):
        """
        Test validate_url_integrity with empty results
        """
        validation = metadata_service.validate_url_integrity([])
        expected = {
            "integrity_score": 0.0,
            "valid_count": 0,
            "total_count": 0
        }
        assert validation == expected

    def test_validate_section_integrity(self, metadata_service):
        """
        Test validate_section_integrity method
        """
        metadata1 = ResultMetadata(
            url="https://example.com",
            section="Section 1",
            chunk_index=1
        )
        metadata2 = ResultMetadata(
            url="https://example.com",
            section="Section 2",
            chunk_index=2
        )
        result1 = SearchResult(
            id="result_1",
            score=0.85,
            content="content 1",
            metadata=metadata1,
            query_id="query_1"
        )
        result2 = SearchResult(
            id="result_2",
            score=0.75,
            content="content 2",
            metadata=metadata2,
            query_id="query_1"
        )

        results = [result1, result2]
        validation = metadata_service.validate_section_integrity(results)

        assert validation["integrity_score"] == 1.0  # Both have valid sections
        assert validation["valid_count"] == 2
        assert validation["total_count"] == 2

    def test_validate_chunk_index_integrity(self, metadata_service):
        """
        Test validate_chunk_index_integrity method
        """
        metadata1 = ResultMetadata(
            url="https://example.com",
            section="Section 1",
            chunk_index=1
        )
        metadata2 = ResultMetadata(
            url="https://example.com",
            section="Section 2",
            chunk_index=2
        )
        # Result with invalid chunk index
        metadata3 = ResultMetadata(
            url="https://example.com",
            section="Section 3",
            chunk_index=-1  # Invalid chunk index
        )
        result1 = SearchResult(
            id="result_1",
            score=0.85,
            content="content 1",
            metadata=metadata1,
            query_id="query_1"
        )
        result2 = SearchResult(
            id="result_2",
            score=0.75,
            content="content 2",
            metadata=metadata2,
            query_id="query_1"
        )
        result3 = SearchResult(
            id="result_3",
            score=0.80,
            content="content 3",
            metadata=metadata3,
            query_id="query_1"
        )

        results = [result1, result2, result3]
        validation = metadata_service.validate_chunk_index_integrity(results)

        assert validation["integrity_score"] == 1.0  # All have valid chunk indices (>= 0)
        assert validation["valid_count"] == 3
        assert validation["total_count"] == 3

    def test_validate_source_title_integrity(self, metadata_service):
        """
        Test validate_source_title_integrity method
        """
        metadata1 = ResultMetadata(
            url="https://example.com",
            section="Section 1",
            chunk_index=1,
            source_title="Title 1"
        )
        metadata2 = ResultMetadata(
            url="https://example.com",
            section="Section 2",
            chunk_index=2,
            source_title="Title 2"
        )
        # Result with empty source title
        metadata3 = ResultMetadata(
            url="https://example.com",
            section="Section 3",
            chunk_index=3,
            source_title=""
        )
        result1 = SearchResult(
            id="result_1",
            score=0.85,
            content="content 1",
            metadata=metadata1,
            query_id="query_1"
        )
        result2 = SearchResult(
            id="result_2",
            score=0.75,
            content="content 2",
            metadata=metadata2,
            query_id="query_1"
        )
        result3 = SearchResult(
            id="result_3",
            score=0.80,
            content="content 3",
            metadata=metadata3,
            query_id="query_1"
        )

        results = [result1, result2, result3]
        validation = metadata_service.validate_source_title_integrity(results)

        assert validation["integrity_score"] == 2/3  # 2 out of 3 have valid titles
        assert validation["valid_count"] == 2
        assert validation["total_count"] == 3

    def test_calculate_metadata_completeness(self, metadata_service):
        """
        Test calculate_metadata_completeness method
        """
        # Result with all required fields
        metadata1 = ResultMetadata(
            url="https://example.com",
            section="Section 1",
            chunk_index=1,
            source_title="Title 1"
        )
        # Result missing optional field (source_title)
        metadata2 = ResultMetadata(
            url="https://example.com",
            section="Section 2",
            chunk_index=2,
            source_title=None
        )
        result1 = SearchResult(
            id="result_1",
            score=0.85,
            content="content 1",
            metadata=metadata1,
            query_id="query_1"
        )
        result2 = SearchResult(
            id="result_2",
            score=0.75,
            content="content 2",
            metadata=metadata2,
            query_id="query_1"
        )

        results = [result1, result2]
        completeness = metadata_service.calculate_metadata_completeness(results)

        assert "completeness_score" in completeness
        assert "required_fields_present" in completeness
        assert "optional_fields_present" in completeness

        # Both results have all required fields (url, section, chunk_index)
        assert completeness["required_fields_present"] == 1.0
        # Only 1 out of 2 results has the optional source_title field
        assert completeness["optional_fields_present"] == 0.5

    def test_validate_metadata_consistency(self, metadata_service):
        """
        Test validate_metadata_consistency method
        """
        # Results with same URL and section (consistent)
        metadata1 = ResultMetadata(
            url="https://example.com",
            section="Section 1",
            chunk_index=1
        )
        metadata2 = ResultMetadata(
            url="https://example.com",  # Same URL
            section="Section 1",        # Same section
            chunk_index=2
        )
        result1 = SearchResult(
            id="result_1",
            score=0.85,
            content="content 1",
            metadata=metadata1,
            query_id="query_1"
        )
        result2 = SearchResult(
            id="result_2",
            score=0.75,
            content="content 2",
            metadata=metadata2,
            query_id="query_1"
        )

        results = [result1, result2]
        consistency = metadata_service.validate_metadata_consistency(results)

        assert consistency["consistency_score"] >= 0.6  # Should be high for consistent results
        assert len(consistency["inconsistencies"]) == 0  # No inconsistencies

    def test_validate_metadata_consistency_inconsistent(self, metadata_service):
        """
        Test validate_metadata_consistency with inconsistent results
        """
        # Results with different URLs and sections (inconsistent)
        metadata1 = ResultMetadata(
            url="https://example1.com",
            section="Section 1",
            chunk_index=1
        )
        metadata2 = ResultMetadata(
            url="https://example2.com",  # Different URL
            section="Section 2",        # Different section
            chunk_index=2
        )
        result1 = SearchResult(
            id="result_1",
            score=0.85,
            content="content 1",
            metadata=metadata1,
            query_id="query_1"
        )
        result2 = SearchResult(
            id="result_2",
            score=0.75,
            content="content 2",
            metadata=metadata2,
            query_id="query_1"
        )

        results = [result1, result2]
        consistency = metadata_service.validate_metadata_consistency(results)

        assert consistency["consistency_score"] < 0.8  # Should be lower for inconsistent results
        assert len(consistency["inconsistencies"]) > 0  # Should have inconsistencies