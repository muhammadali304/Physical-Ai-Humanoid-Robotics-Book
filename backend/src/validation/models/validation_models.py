from pydantic import BaseModel, Field, validator
from pydantic.functional_validators import model_validator
from typing import List, Optional, Dict, Any
from datetime import datetime
from enum import Enum


class ValidationStatus(str, Enum):
    SUCCESS = "success"
    PARTIAL = "partial"
    FAILED = "failed"


class ValidationReportStatus(str, Enum):
    PASS = "pass"
    FAIL = "fail"
    WARNING = "warning"


class SearchQuery(BaseModel):
    id: Optional[str] = None
    query_text: str = Field(..., min_length=1, description="The text to search for in the validation")
    top_k: int = Field(default=5, ge=1, le=100, description="Number of results to return")
    min_score: float = Field(default=0.0, ge=0.0, le=1.0, description="Minimum relevance score threshold")
    filters: Optional[Dict[str, Any]] = Field(default_factory=dict, description="Optional filtering parameters (URL, section, etc.)")
    created_at: Optional[datetime] = Field(default_factory=datetime.utcnow)

    @validator('query_text')
    def validate_query_text(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('query_text must be non-empty')
        return v

    @validator('top_k')
    def validate_top_k(cls, v):
        if v < 1 or v > 100:
            raise ValueError('top_k must be between 1 and 100')
        return v

    @validator('min_score')
    def validate_min_score(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('min_score must be between 0.0 and 1.0')
        return v


class ResultMetadata(BaseModel):
    url: str
    section: str
    chunk_index: int
    source_title: Optional[str] = None


class SearchResult(BaseModel):
    id: str
    score: float = Field(ge=0.0, le=1.0, description="Relevance score between 0.0 and 1.0")
    content: str = Field(..., min_length=1, description="Raw content of the retrieved chunk")
    metadata: ResultMetadata
    query_id: str

    @validator('score')
    def validate_score(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('score must be between 0.0 and 1.0')
        return v

    @validator('content')
    def validate_content(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('content must be non-empty')
        return v


class ValidationResult(BaseModel):
    id: str
    query: SearchQuery
    results: List[SearchResult] = Field(default_factory=list)
    relevance_metrics: Optional[Dict[str, float]] = Field(default_factory=dict)
    metadata_validation: Optional[Dict[str, float]] = Field(default_factory=dict)
    execution_time: float = Field(gt=0.0, description="Time taken for validation in seconds")
    status: ValidationStatus
    created_at: Optional[datetime] = Field(default_factory=datetime.utcnow)

    @validator('execution_time')
    def validate_execution_time(cls, v):
        if v <= 0:
            raise ValueError('execution_time must be positive')
        return v


class ValidationReport(BaseModel):
    id: str
    test_suite: str
    total_tests: int = Field(gt=0)
    passed_tests: int
    failed_tests: int
    results: List[ValidationResult]
    summary_metrics: Optional[Dict[str, float]] = Field(default_factory=dict)
    status: ValidationReportStatus
    created_at: Optional[datetime] = Field(default_factory=datetime.utcnow)
    duration: float

    @validator('total_tests')
    def validate_total_tests(cls, v):
        if v <= 0:
            raise ValueError('total_tests must be greater than 0')
        return v

    @validator('passed_tests', 'failed_tests')
    def validate_test_counts(cls, v):
        if v < 0:
            raise ValueError('test counts must be non-negative')
        return v

    @validator('status')
    def validate_status(cls, v):
        if v not in ['pass', 'fail', 'warning']:
            raise ValueError('status must be one of: pass, fail, warning')
        return v

    @model_validator(mode='after')
    def validate_counts_match(self):
        if self.total_tests is not None and self.passed_tests is not None and self.failed_tests is not None:
            if self.passed_tests + self.failed_tests != self.total_tests:
                raise ValueError('passed_tests + failed_tests must equal total_tests')
        return self


class ValidationTest(BaseModel):
    id: Optional[str] = None
    name: str = Field(..., min_length=1)
    description: str = Field(..., min_length=1)
    query: str = Field(..., min_length=1)
    expected_results: List[str] = Field(default_factory=list)
    thresholds: Optional[Dict[str, float]] = Field(default_factory=dict)
    category: str
    enabled: bool = True

    @validator('name', 'description')
    def validate_name_desc(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('name and description must be non-empty')
        return v

    @validator('query')
    def validate_query(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('query must be non-empty')
        return v

    @validator('thresholds')
    def validate_thresholds(cls, v):
        if v:
            for key, value in v.items():
                if not isinstance(value, (int, float)):
                    raise ValueError(f'threshold values must be numeric')
                if key not in ['accuracy', 'precision', 'recall', 'consistency']:
                    raise ValueError(f'threshold keys must be valid metric names')
        return v


class BatchValidationRequest(BaseModel):
    queries: List[SearchQuery]
    batch_size: int = Field(ge=1, le=1000, default=10)
    concurrency: int = Field(ge=1, le=50, default=5)
    timeout: float = Field(gt=0.0, default=30.0)
    min_score_threshold: float = Field(ge=0.0, le=1.0, default=0.3)
    max_execution_time: float = Field(gt=0.0, default=300.0)  # 5 minutes

    @validator('batch_size')
    def validate_batch_size(cls, v):
        if v < 1 or v > 1000:
            raise ValueError('batch_size must be between 1 and 1000')
        return v

    @validator('concurrency')
    def validate_concurrency(cls, v):
        if v < 1 or v > 50:
            raise ValueError('concurrency must be between 1 and 50')
        return v

    @validator('timeout', 'max_execution_time')
    def validate_positive_time(cls, v):
        if v <= 0:
            raise ValueError('time values must be positive')
        return v

    @validator('min_score_threshold')
    def validate_min_score_threshold(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('min_score_threshold must be between 0.0 and 1.0')
        return v

    @validator('queries')
    def validate_queries(cls, v):
        if not v:
            raise ValueError('queries list must not be empty')
        return v


class TestSuiteRequest(BaseModel):
    suite_name: str = Field(..., min_length=1, max_length=100, description="Name of the test suite")
    queries: List[SearchQuery] = Field(default_factory=list)
    batch_size: int = Field(ge=1, le=1000, default=10)
    concurrency: int = Field(ge=1, le=50, default=5)
    timeout: float = Field(gt=0.0, default=30.0)
    min_score_threshold: float = Field(ge=0.0, le=1.0, default=0.3)
    max_execution_time: float = Field(gt=0.0, default=300.0)  # 5 minutes
    test_config: Optional[Dict[str, Any]] = Field(default_factory=dict)

    @validator('suite_name')
    def validate_suite_name(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('suite_name must be non-empty')
        if len(v) > 100:
            raise ValueError('suite_name must be 100 characters or less')
        return v

    @validator('batch_size', 'concurrency', 'timeout', 'max_execution_time', 'min_score_threshold')
    def validate_common_fields(cls, v):
        if isinstance(v, int) and v <= 0:
            raise ValueError('numeric values must be positive')
        if isinstance(v, float) and v <= 0:
            raise ValueError('numeric values must be positive')
        return v


class ValidationConfig(BaseModel):
    batch_size: int = Field(ge=1, le=1000, default=10)
    concurrency: int = Field(ge=1, le=50, default=5)
    timeout: float = Field(gt=0.0, default=30.0)
    min_score_threshold: float = Field(ge=0.0, le=1.0, default=0.3)
    max_execution_time: float = Field(gt=0.0, default=300.0)  # 5 minutes
    report_formats: List[str] = Field(default=['json', 'csv'])

    @validator('batch_size')
    def validate_batch_size(cls, v):
        if v < 1 or v > 1000:
            raise ValueError('batch_size must be between 1 and 1000')
        return v

    @validator('concurrency')
    def validate_concurrency(cls, v):
        if v < 1 or v > 50:
            raise ValueError('concurrency must be between 1 and 50')
        return v

    @validator('timeout', 'max_execution_time')
    def validate_positive_time(cls, v):
        if v <= 0:
            raise ValueError('time values must be positive')
        return v

    @validator('min_score_threshold')
    def validate_min_score_threshold(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('min_score_threshold must be between 0.0 and 1.0')
        return v