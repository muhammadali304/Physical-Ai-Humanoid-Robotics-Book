# Quickstart Guide: RAG Retrieval & Pipeline Validation

## Overview

This guide will help you quickly set up and run validation tests on your RAG pipeline. The validation system allows you to verify semantic search accuracy, metadata integrity, and overall pipeline consistency.

## Prerequisites

- Python 3.10+
- Access to Qdrant vector database with embedded content
- Cohere API key for embedding generation
- Valid credentials for your RAG pipeline

## Setup

### 1. Clone and Install Dependencies

```bash
git clone <repository-url>
cd <repository-name>
poetry install
```

### 2. Configure Environment Variables

Create a `.env` file in the project root with the following variables:

```bash
# Qdrant Configuration
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key

# Cohere Configuration
COHERE_API_KEY=your_cohere_api_key

# Validation API Configuration
VALIDATION_API_KEY=your_secure_api_key

# Optional: Redis for caching (if enabled)
REDIS_URL=redis://localhost:6379

# Logging Configuration
LOG_LEVEL=INFO
```

### 3. Verify Configuration

Run the configuration verification script:

```bash
python scripts/verify_config.py
```

## Running Basic Validation

### 1. Start the Validation Service

```bash
python -m src.validation.main
```

The service will start on `http://localhost:8000` by default.

### 2. Perform a Simple Search Validation

Test the semantic search functionality with a basic query:

```bash
curl -X POST http://localhost:8000/validation/search \
  -H "Authorization: Bearer $VALIDATION_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "What are the key features of this system?",
    "top_k": 5,
    "min_score": 0.3
  }'
```

### 3. Run a Predefined Test Suite

Execute a batch of validation tests:

```bash
curl -X POST http://localhost:8000/validation/test-suite \
  -H "Authorization: Bearer $VALIDATION_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "suite_name": "basic_accuracy_tests",
    "batch_size": 10,
    "concurrency": 5
  }'
```

## Running Batch Validation

### 1. Prepare Your Test Queries

Create a JSON file with your validation queries:

```json
{
  "queries": [
    {
      "query_text": "How does the system handle authentication?",
      "expected_sources": ["auth.md", "security.md"]
    },
    {
      "query_text": "What are the performance requirements?",
      "expected_sources": ["performance.md", "requirements.md"]
    }
  ]
}
```

### 2. Execute Batch Validation

```bash
curl -X POST http://localhost:8000/validation/batch \
  -H "Authorization: Bearer $VALIDATION_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "queries": [
      {
        "query_text": "How does the system handle authentication?",
        "top_k": 5,
        "filters": {"section": "security"}
      }
    ],
    "batch_size": 20,
    "concurrency": 4
  }'
```

## Viewing Validation Reports

### 1. List Available Reports

```bash
curl -X GET http://localhost:8000/validation/reports \
  -H "Authorization: Bearer $VALIDATION_API_KEY"
```

### 2. Get a Specific Report

```bash
curl -X GET http://localhost:8000/validation/reports/{report_id} \
  -H "Authorization: Bearer $VALIDATION_API_KEY"
```

### 3. Download Report in Different Formats

```bash
# JSON format
curl -X GET http://localhost:8000/validation/reports/{report_id}?format=json \
  -H "Authorization: Bearer $VALIDATION_API_KEY"

# CSV format
curl -X GET http://localhost:8000/validation/reports/{report_id}?format=csv \
  -H "Authorization: Bearer $VALIDATION_API_KEY"
```

## Filtering and Advanced Validation

### 1. Filter by Source URL

```bash
curl -X POST http://localhost:8000/validation/search \
  -H "Authorization: Bearer $VALIDATION_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "API documentation",
    "filters": {
      "source_url": "https://docs.example.com/api/"
    },
    "top_k": 5
  }'
```

### 2. Filter by Section

```bash
curl -X POST http://localhost:8000/validation/search \
  -H "Authorization: Bearer $VALIDATION_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "configuration settings",
    "filters": {
      "section": "Configuration"
    },
    "top_k": 3
  }'
```

## Interpreting Results

### Validation Report Structure

A validation report includes:

- `status`: Overall validation status (pass/fail/warning)
- `summary_metrics`: Key performance indicators
- `relevance_metrics`: Accuracy, precision, recall measurements
- `metadata_validation`: Integrity check results
- `execution_time`: Time taken for validation

### Success Criteria

A validation is considered successful if:

- Accuracy > 85% (configurable threshold)
- Metadata integrity 100%
- Response time < 500ms for 95% of requests (configurable)
- Consistency variance < 5% across repeated queries

## Troubleshooting

### Common Issues

1. **Authentication Errors**: Verify your API key is correct and has proper permissions
2. **Qdrant Connection Issues**: Check your QDRANT_URL and QDRANT_API_KEY
3. **Cohere API Errors**: Verify your COHERE_API_KEY and API access
4. **Empty Results**: Ensure your Qdrant database contains embedded content

### Checking System Health

```bash
curl -X GET http://localhost:8000/health
```

## Next Steps

1. Explore the API documentation at `/docs` endpoint
2. Set up automated validation tests with cron jobs
3. Configure alerting for validation failures
4. Customize validation thresholds for your specific use case