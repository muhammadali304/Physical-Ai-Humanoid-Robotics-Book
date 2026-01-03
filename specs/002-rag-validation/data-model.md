# Data Model: RAG Retrieval & Pipeline Validation

## Core Entities

### SearchQuery
**Description**: Represents a validation query with parameters for semantic search validation

**Fields**:
- `id` (str): Unique identifier for the query
- `query_text` (str): The text to search for in the validation
- `top_k` (int): Number of results to return (default: 5)
- `min_score` (float): Minimum relevance score threshold (default: 0.0)
- `filters` (dict): Optional filtering parameters (URL, section, etc.)
- `created_at` (datetime): Timestamp when the query was created

**Validation Rules**:
- `query_text` must be non-empty
- `top_k` must be between 1 and 100
- `min_score` must be between 0.0 and 1.0

### SearchResult
**Description**: Individual result from a semantic search validation

**Fields**:
- `id` (str): Unique identifier for the result
- `score` (float): Relevance score between 0.0 and 1.0
- `content` (str): Raw content of the retrieved chunk
- `metadata` (dict): Contains source URL, section, chunk_index, etc.
- `query_id` (str): Reference to the original query

**Validation Rules**:
- `score` must be between 0.0 and 1.0
- `content` must be non-empty
- `metadata` must contain required fields (URL, section, chunk_index)

### ValidationResult
**Description**: Result of a single validation operation

**Fields**:
- `id` (str): Unique identifier for the validation result
- `query` (SearchQuery): The original validation query
- `results` (List[SearchResult]): Retrieved search results
- `relevance_metrics` (dict): Accuracy, precision, recall measurements
- `metadata_validation` (dict): Metadata integrity check results
- `execution_time` (float): Time taken for validation in seconds
- `status` (str): Status of validation (success, partial, failed)
- `created_at` (datetime): Timestamp of validation execution

**Validation Rules**:
- `results` list must not be empty
- `execution_time` must be positive
- `status` must be one of: success, partial, failed

### ValidationReport
**Description**: Comprehensive report of validation test suite execution

**Fields**:
- `id` (str): Unique identifier for the report
- `test_suite` (str): Name of the test suite executed
- `total_tests` (int): Total number of validation tests executed
- `passed_tests` (int): Number of tests that passed
- `failed_tests` (int): Number of tests that failed
- `results` (List[ValidationResult]): Individual validation results
- `summary_metrics` (dict): Overall performance and accuracy metrics
- `status` (str): Overall validation status (pass, fail, warning)
- `created_at` (datetime): Timestamp of report generation
- `duration` (float): Total time taken for validation suite execution

**Validation Rules**:
- `total_tests` must be greater than 0
- `passed_tests + failed_tests` must equal `total_tests`
- `status` must be one of: pass, fail, warning

### ValidationTest
**Description**: Predefined test case for pipeline validation

**Fields**:
- `id` (str): Unique identifier for the test
- `name` (str): Descriptive name for the test
- `description` (str): Detailed description of what the test validates
- `query` (str): Query text for the validation test
- `expected_results` (List[str]): Expected content or metadata identifiers
- `thresholds` (dict): Minimum acceptable values for metrics
- `category` (str): Category of validation (accuracy, consistency, etc.)
- `enabled` (bool): Whether the test is enabled for execution

**Validation Rules**:
- `name` and `description` must be non-empty
- `query` must be non-empty
- `thresholds` must contain valid metric names and values

### ValidationConfig
**Description**: Configuration for validation operations

**Fields**:
- `batch_size` (int): Number of queries to process in each batch
- `concurrency` (int): Number of concurrent validation workers
- `timeout` (float): Maximum time for individual validation requests
- `min_score_threshold` (float): Minimum acceptable relevance score
- `max_execution_time` (float): Maximum time for validation suite execution
- `report_formats` (List[str]): Output formats for validation reports

**Validation Rules**:
- `batch_size` must be between 1 and 1000
- `concurrency` must be between 1 and 50
- `timeout` must be positive

## Relationships

### SearchQuery → SearchResult
- One-to-many: A single query can return multiple results
- Reference: SearchResult.query_id → SearchQuery.id

### ValidationResult → SearchResult
- One-to-many: A validation result contains multiple search results
- Reference: SearchResult.query_id → ValidationResult.query.id

### ValidationReport → ValidationResult
- One-to-many: A validation report contains multiple validation results
- Reference: ValidationResult.id in ValidationReport.results

### ValidationTest → ValidationResult
- One-to-many: A validation test can produce multiple validation results over time
- Reference: Not directly stored, but test context can be included in ValidationResult

## State Transitions

### ValidationResult Status
- `created` → `processing` → `success` | `partial` | `failed`
- Transitions based on the success of the validation operation

### ValidationReport Status
- `created` → `executing` → `completed` → `pass` | `fail` | `warning`
- Final status determined by the aggregation of individual test results

## Indexes and Performance Considerations

### Required Indexes
- SearchQuery.created_at: For time-based queries
- ValidationResult.query_id: For linking results to queries
- ValidationReport.created_at: For report retrieval
- SearchResult.score: For sorting by relevance

### Performance Optimizations
- SearchResults should be stored with indexed metadata fields for efficient filtering
- ValidationReports should include pre-calculated summary metrics
- ValidationTest definitions should be cached for efficient retrieval