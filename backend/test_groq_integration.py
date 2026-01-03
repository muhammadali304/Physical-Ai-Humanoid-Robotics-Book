#!/usr/bin/env python3
"""
Test script to verify Groq API integration with the updated MultiLLMRouterService.
"""

import asyncio
import logging
import uuid
from src.services.multi_llm_router import MultiLLMRouterService
from src.models.query import QueryRequest

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def test_groq_connection():
    """Test the Groq API connection with the updated service"""
    try:
        logger.info("Initializing MultiLLMRouterService with Groq...")
        router = MultiLLMRouterService()

        logger.info("Testing Groq API connection...")

        # Create a test query with valid format
        test_query = QueryRequest(
            query="Hello, how are you?",
            session_id=str(uuid.uuid4())
        )

        # Test the Groq call (now the only method available)
        result = await router.route_query(test_query)

        logger.info(f"Groq response: {result.response}")
        logger.info(f"Token usage: {result.tokens_used}")
        logger.info(f"Session ID: {result.session_id}")
        logger.info(f"Sources: {result.sources}")

        return True

    except Exception as e:
        logger.error(f"Error testing Groq connection: {str(e)}")
        logger.error(f"Error type: {type(e).__name__}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
        return False

async def test_groq_with_different_queries():
    """Test Groq API with different types of queries"""
    try:
        logger.info("Testing Groq API with various queries...")
        router = MultiLLMRouterService()

        test_queries = [
            "What is the capital of France?",
            "Explain quantum computing in simple terms.",
            "Write a short poem about technology.",
            "How does machine learning work?"
        ]

        for i, query_text in enumerate(test_queries, 1):
            logger.info(f"Test {i}: {query_text}")

            test_query = QueryRequest(
                query=query_text,
                session_id=str(uuid.uuid4())
            )

            result = await router.route_query(test_query)

            logger.info(f"  Response length: {len(result.response)} characters")
            logger.info(f"  Tokens used: {result.tokens_used}")
            if result.tokens_used:
                logger.info(f"    Input: {result.tokens_used.input_tokens}, Output: {result.tokens_used.output_tokens}, Total: {result.tokens_used.total_tokens}")

        return True

    except Exception as e:
        logger.error(f"Error testing multiple queries: {str(e)}")
        logger.error(f"Error type: {type(e).__name__}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
        return False

async def main():
    """Main test function"""
    logger.info("Starting Groq API integration tests...")

    # Test basic Groq connection
    logger.info("\n1. Testing basic Groq API connection...")
    success1 = await test_groq_connection()

    if success1:
        logger.info("✓ Basic Groq API connection successful!")
    else:
        logger.error("✗ Basic Groq API connection failed!")
        return  # Stop if basic connection fails

    # Test with different queries
    logger.info("\n2. Testing Groq API with different queries...")
    success2 = await test_groq_with_different_queries()

    if success2:
        logger.info("✓ Multiple queries test successful!")
    else:
        logger.error("✗ Multiple queries test failed!")

    logger.info(f"\nFinal Test Results:")
    logger.info(f"Basic Groq connection: {'PASS' if success1 else 'FAIL'}")
    logger.info(f"Multiple queries: {'PASS' if success2 else 'FAIL'}")

    if success1 and success2:
        logger.info("\n🎉 All Groq API integration tests passed!")
        logger.info("The system should now be working correctly with Groq API and 400 errors should be resolved.")
    else:
        logger.error("\n❌ Some tests failed. Please check the logs above.")

if __name__ == "__main__":
    asyncio.run(main())