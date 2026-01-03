#!/usr/bin/env python3
"""
Debug script to replicate the exact flow that happens in RAGChatbotAgent.
"""

import asyncio
import uuid
from src.services.multi_llm_router import MultiLLMRouterService
from src.models.query import QueryRequest, QueryResponse
from src.agents.rag_chatbot_agent import RAGChatbotAgent

async def debug_rag_flow():
    """Debug the exact flow that happens in RAGChatbotAgent"""
    print("Debugging the RAG flow...")

    # Step 1: Create a router and get a result (like in the service test)
    print("\n1. Creating router and getting result...")
    router = MultiLLMRouterService()

    test_query = QueryRequest(
        query="Hello, how are you?",
        session_id=str(uuid.uuid4())
    )

    result = await router.route_query(test_query)
    print(f"Router result: {result}")
    print(f"Router result.tokens_used: {result.tokens_used}")
    print(f"Type of tokens_used: {type(result.tokens_used)}")

    # Step 2: Simulate what happens in RAGChatbotAgent
    print("\n2. Simulating RAGChatbotAgent process...")

    # This is essentially what happens in the RAGChatbotAgent
    try:
        query_response = QueryResponse(
            response="Formatted response from RAG",
            sources=[],  # Empty sources like in RAGChatbotAgent
            session_id=test_query.session_id,
            tokens_used=result.tokens_used,  # This is the problematic line
            retrieval_info=None
        )
        print(f"SUCCESS: QueryResponse created successfully in RAG context")
        print(f"QueryResponse tokens_used: {query_response.tokens_used}")
    except Exception as e:
        print(f"ERROR: Failed to create QueryResponse in RAG context: {e}")
        import traceback
        traceback.print_exc()

    # Step 3: Test with a new TokenUsage object created from the values
    print("\n3. Testing with new TokenUsage object...")
    try:
        new_token_usage = None
        if result.tokens_used:
            new_token_usage = type(result.tokens_used)(
                input_tokens=result.tokens_used.input_tokens,
                output_tokens=result.tokens_used.output_tokens,
                total_tokens=result.tokens_used.total_tokens
            )

        query_response2 = QueryResponse(
            response="Formatted response from RAG",
            sources=[],
            session_id=test_query.session_id,
            tokens_used=new_token_usage,
            retrieval_info=None
        )
        print(f"SUCCESS: QueryResponse created with new TokenUsage object")
        print(f"QueryResponse2 tokens_used: {query_response2.tokens_used}")
    except Exception as e:
        print(f"ERROR: Failed with new TokenUsage object: {e}")
        import traceback
        traceback.print_exc()

    # Step 4: Test with dict
    print("\n4. Testing with dictionary...")
    try:
        token_usage_dict = None
        if result.tokens_used:
            token_usage_dict = {
                'input_tokens': result.tokens_used.input_tokens,
                'output_tokens': result.tokens_used.output_tokens,
                'total_tokens': result.tokens_used.total_tokens
            }

        query_response3 = QueryResponse(
            response="Formatted response from RAG",
            sources=[],
            session_id=test_query.session_id,
            tokens_used=token_usage_dict,
            retrieval_info=None
        )
        print(f"SUCCESS: QueryResponse created with dict")
        print(f"QueryResponse3 tokens_used: {query_response3.tokens_used}")
    except Exception as e:
        print(f"ERROR: Failed with dict: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(debug_rag_flow())