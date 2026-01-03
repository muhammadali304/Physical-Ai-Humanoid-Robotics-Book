#!/usr/bin/env python3
"""
Debug script to understand the TokenUsage serialization issue.
"""

import asyncio
from src.services.multi_llm_router import MultiLLMRouterService
from src.models.query import QueryRequest, QueryResponse
import uuid

async def debug_token_usage():
    """Debug the TokenUsage serialization issue"""
    print("Debugging TokenUsage serialization...")

    # Initialize the router
    router = MultiLLMRouterService()

    # Create a test query
    test_query = QueryRequest(
        query="Hello, how are you?",
        session_id=str(uuid.uuid4())
    )

    print("Calling router...")
    result = await router.route_query(test_query)

    print(f"Result type: {type(result)}")
    print(f"Result.tokens_used type: {type(result.tokens_used)}")
    print(f"Result.tokens_used value: {result.tokens_used}")

    # Check if it's serializable
    try:
        # Try to access its dict representation
        if hasattr(result.tokens_used, 'dict'):
            token_dict = result.tokens_used.dict()
            print(f"TokenUsage dict: {token_dict}")
        elif hasattr(result.tokens_used, '__dict__'):
            token_dict = result.tokens_used.__dict__
            print(f"TokenUsage __dict__: {token_dict}")
        else:
            print("No dict method found on TokenUsage")
    except Exception as e:
        print(f"Error getting dict from TokenUsage: {e}")

    # Check the full response
    try:
        if hasattr(result, 'dict'):
            response_dict = result.dict()
            print(f"QueryResponse dict created successfully")
            print(f"Keys in response dict: {list(response_dict.keys())}")
        else:
            print("No dict method found on QueryResponse")
    except Exception as e:
        print(f"Error getting dict from QueryResponse: {e}")

if __name__ == "__main__":
    asyncio.run(debug_token_usage())