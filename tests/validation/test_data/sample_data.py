"""
Sample test data for validation tests
"""

# Sample search queries for testing
SAMPLE_SEARCH_QUERIES = [
    {
        "query_text": "What are the key features of this system?",
        "top_k": 5,
        "min_score": 0.3
    },
    {
        "query_text": "How does the authentication work?",
        "top_k": 3,
        "min_score": 0.4
    },
    {
        "query_text": "What are the performance requirements?",
        "top_k": 10,
        "min_score": 0.2
    }
]

# Sample validation results
SAMPLE_VALIDATION_RESULTS = [
    {
        "id": "vr_1",
        "query": {
            "query_text": "What are the key features?",
            "top_k": 5,
            "min_score": 0.3
        },
        "results": [
            {
                "id": "result_1",
                "score": 0.95,
                "content": "The system has several key features including authentication, authorization, and data validation.",
                "metadata": {
                    "url": "https://example.com/features",
                    "section": "Key Features",
                    "chunk_index": 1,
                    "source_title": "Features Documentation"
                },
                "query_id": "query_1"
            }
        ],
        "relevance_metrics": {
            "accuracy": 0.92,
            "precision": 0.88,
            "recall": 0.85
        },
        "metadata_validation": {
            "url_integrity": 1.0,
            "section_integrity": 1.0,
            "completeness": 1.0
        },
        "execution_time": 0.12,
        "status": "success"
    }
]

# Sample validation reports
SAMPLE_VALIDATION_REPORTS = [
    {
        "id": "report_1",
        "test_suite": "basic_accuracy_tests",
        "total_tests": 5,
        "passed_tests": 4,
        "failed_tests": 1,
        "results": SAMPLE_VALIDATION_RESULTS,
        "summary_metrics": {
            "avg_accuracy": 0.89,
            "avg_precision": 0.85,
            "avg_recall": 0.82,
            "avg_response_time": 0.15
        },
        "status": "warning",
        "created_at": "2023-01-01T00:00:00",
        "duration": 2.5
    }
]

# Sample test configurations
SAMPLE_TEST_CONFIGS = [
    {
        "batch_size": 10,
        "concurrency": 5,
        "timeout": 30.0,
        "min_score_threshold": 0.3,
        "max_execution_time": 300.0,
        "report_formats": ["json", "csv"]
    }
]

# Sample metadata for testing
SAMPLE_METADATA = [
    {
        "url": "https://example.com/docs/intro",
        "section": "Introduction",
        "chunk_index": 0,
        "source_title": "Introduction to the System"
    },
    {
        "url": "https://example.com/docs/features",
        "section": "Features",
        "chunk_index": 1,
        "source_title": "System Features"
    },
    {
        "url": "https://example.com/docs/api",
        "section": "API Reference",
        "chunk_index": 2,
        "source_title": "API Documentation"
    }
]

# Sample search results
SAMPLE_SEARCH_RESULTS = [
    {
        "id": "result_1",
        "score": 0.95,
        "content": "This is a highly relevant result for the search query.",
        "metadata": SAMPLE_METADATA[0],
        "query_id": "query_1"
    },
    {
        "id": "result_2",
        "score": 0.85,
        "content": "This is a moderately relevant result for the search query.",
        "metadata": SAMPLE_METADATA[1],
        "query_id": "query_1"
    },
    {
        "id": "result_3",
        "score": 0.75,
        "content": "This is a somewhat relevant result for the search query.",
        "metadata": SAMPLE_METADATA[2],
        "query_id": "query_1"
    }
]