#!/usr/bin/env python3
"""
Test script to isolate the QueryResponse creation issue.
"""

from src.models.query import QueryResponse, TokenUsage
from src.services.multi_llm_router import MultiLLMRouterService
import asyncio
import uuid

async def test_query_response_creation():
    """Test creating QueryResponse with TokenUsage"""
    print("Testing QueryResponse creation with TokenUsage...")

    # Test 1: Create TokenUsage directly
    print("\n1. Creating TokenUsage directly...")
    token_usage = {
        "input_tokens": 10,
        "output_tokens": 20,
        "total_tokens": 30
    }

    print(f"TokenUsage created: {token_usage}")

    # Test 2: Create QueryResponse with the TokenUsage
    print("\n2. Creating QueryResponse with TokenUsage...")
    try:
        query_response = QueryResponse(
            response="Test response",
            sources=[],
            session_id=str(uuid.uuid4()),
            tokens_used=token_usage
        )
        print(f"QueryResponse created successfully: {query_response}")
        print(f"Token usage in response: {query_response.tokens_used}")
    except Exception as e:
        print(f"Error creating QueryResponse: {e}")

    # Test 3: Create QueryResponse with TokenUsage dict
    print("\n3. Creating QueryResponse with TokenUsage dict...")
    try:
        query_response2 = QueryResponse(
            response="Test response",
            sources=[],
            session_id=str(uuid.uuid4()),
            tokens_used={
                "input_tokens": 15,
                "output_tokens": 25,
                "total_tokens": 40
            }
        )
        print(f"QueryResponse created successfully with dict: {query_response2}")
    except Exception as e:
        print(f"Error creating QueryResponse with dict: {e}")

    # Test 4: Use the actual router to see what it returns
    print("\n4. Testing with actual router...")
    try:
        router = MultiLLMRouterService()
        result = await router._call_groq(type('MockQueryRequest', (), {
            'query': 'test',
            'session_id': str(uuid.uuid4())
        })())
        print(f"Router result tokens: {result.tokens_used}")

        # Now try to use this in a new QueryResponse
        new_response = QueryResponse(
            response="Modified response",
            sources=[],
            session_id=str(uuid.uuid4()),
            tokens_used=result.tokens_used
        )
        print(f"Successfully created new QueryResponse with router's tokens_used: {new_response.tokens_used}")
    except Exception as e:
        print(f"Error in router test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test_query_response_creation())