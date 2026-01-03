from fastapi import APIRouter, Depends, HTTPException, status
from typing import Optional
import time
from starlette.requests import Request
from ...models.validation_models import (
    SearchQuery, ValidationResult, ValidationReport,
    BatchValidationRequest, TestSuiteRequest
)
from ...services.validation_service import ValidationService
from ...config import settings
from ...api.middleware import auth_middleware


router = APIRouter(prefix="/validation", tags=["validation"])


def get_validation_service():
    """Dependency to get validation service instance"""
    return ValidationService()


@router.post("/search", response_model=ValidationResult)
async def validate_search(
    request: Request,
    search_query: SearchQuery,
    validation_service: ValidationService = Depends(get_validation_service)
):
    """
    Validate semantic search functionality
    Performs semantic search and validates results
    """
    # Authenticate request
    await auth_middleware.authenticate(request)
    # Apply rate limiting
    await auth_middleware.rate_limit(request)

    try:
        start_time = time.time()
        result = await validation_service.validate_search(search_query)
        result.execution_time = time.time() - start_time
        return result
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Search validation failed: {str(e)}"
        )


@router.post("/batch", response_model=ValidationReport)
async def run_batch_validation(
    request: Request,
    batch_request: BatchValidationRequest,
    validation_service: ValidationService = Depends(get_validation_service)
):
    """
    Run batch validation
    Execute multiple validation queries in batch
    """
    # Authenticate request
    await auth_middleware.authenticate(request)
    # Apply rate limiting
    await auth_middleware.rate_limit(request)

    try:
        start_time = time.time()
        report = await validation_service.run_batch_validation(batch_request)
        report.duration = time.time() - start_time
        return report
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Batch validation failed: {str(e)}"
        )


@router.post("/test-suite", response_model=ValidationReport)
async def execute_test_suite(
    request: Request,
    test_suite_request: TestSuiteRequest,
    validation_service: ValidationService = Depends(get_validation_service)
):
    """
    Execute validation test suite
    Run a predefined set of validation tests
    """
    # Authenticate request
    await auth_middleware.authenticate(request)
    # Apply rate limiting
    await auth_middleware.rate_limit(request)

    try:
        start_time = time.time()
        report = await validation_service.execute_test_suite(test_suite_request)
        report.duration = time.time() - start_time
        return report
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Test suite execution failed: {str(e)}"
        )


@router.get("/reports", response_model=dict)
async def list_validation_reports(
    request: Request,
    limit: int = 20,
    offset: int = 0,
    validation_service: ValidationService = Depends(get_validation_service)
):
    """
    List validation reports
    Retrieve list of available validation reports
    """
    # Authenticate request
    await auth_middleware.authenticate(request)
    # Apply rate limiting
    await auth_middleware.rate_limit(request)

    try:
        reports = await validation_service.list_reports(limit=limit, offset=offset)
        return {
            "reports": reports,
            "total": len(reports)  # In a real implementation, this would come from a database
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to list reports: {str(e)}"
        )


@router.get("/reports/{report_id}", response_model=ValidationReport)
async def get_validation_report(
    request: Request,
    report_id: str,
    format: Optional[str] = "json",
    validation_service: ValidationService = Depends(get_validation_service)
):
    """
    Get validation report
    Retrieve a specific validation report
    """
    # Authenticate request
    await auth_middleware.authenticate(request)
    # Apply rate limiting
    await auth_middleware.rate_limit(request)

    try:
        report = await validation_service.get_report(report_id)
        if not report:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Report with ID {report_id} not found"
            )
        return report
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to retrieve report: {str(e)}"
        )