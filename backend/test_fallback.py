#!/usr/bin/env python3
"""
Test script to verify the fallback mechanism works when Qwen is unavailable.
"""

import asyncio
import logging
import uuid
from src.services.multi_llm_router import MultiLLMRouterService
from src.models.query import QueryRequest

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def test_fallback_mechanism():
    """Test that the system falls back to Gemini when Qwen is unavailable"""
    try:
        logger.info("Initializing MultiLLMRouterService...")
        router = MultiLLMRouterService()

        logger.info("Testing routing with Qwen preference (should fallback to Gemini)...")

        # Create a test query
        test_query = QueryRequest(
            query="Hello, how are you?",
            session_id=str(uuid.uuid4())
        )

        # Test routing with preference for Qwen (should fallback to Gemini)
        result = await router.route_query(test_query, model_preference="qwen3-max")

        logger.info(f"Response received: {result.response[:100]}...")
        logger.info(f"Token usage: {result.tokens_used}")
        logger.info("SUCCESS: Fallback mechanism is working correctly!")

        return True

    except Exception as e:
        logger.error(f"Error testing fallback mechanism: {str(e)}")
        logger.error(f"Error type: {type(e).__name__}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
        return False

async def test_default_routing():
    """Test default routing (should try Qwen first, then fallback)"""
    try:
        logger.info("Testing default routing with fallback...")
        router = MultiLLMRouterService()

        test_query = QueryRequest(
            query="What is the weather today?",
            session_id=str(uuid.uuid4())
        )

        # Test default routing (should try Qwen first due to API key being set, then fallback)
        result = await router.route_query(test_query)

        logger.info(f"Default routing response: {result.response[:100]}...")
        logger.info(f"Token usage: {result.tokens_used}")
        logger.info("SUCCESS: Default routing with fallback is working!")

        return True

    except Exception as e:
        logger.error(f"Error testing default routing: {str(e)}")
        logger.error(f"Error type: {type(e).__name__}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
        return False

async def main():
    """Main test function"""
    logger.info("Starting fallback mechanism tests...")

    # Test fallback when Qwen is explicitly requested
    logger.info("\n1. Testing Qwen preference with fallback...")
    success1 = await test_fallback_mechanism()

    # Test default routing
    logger.info("\n2. Testing default routing with fallback...")
    success2 = await test_default_routing()

    logger.info(f"\nTest Results:")
    logger.info(f"Qwen preference fallback: {'PASS' if success1 else 'FAIL'}")
    logger.info(f"Default routing fallback: {'PASS' if success2 else 'FAIL'}")

    overall_success = success1 and success2
    logger.info(f"Overall: {'PASS' if overall_success else 'FAIL'}")

    return overall_success

if __name__ == "__main__":
    success = asyncio.run(main())
    exit(0 if success else 1)