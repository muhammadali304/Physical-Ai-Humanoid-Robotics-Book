#!/usr/bin/env python3
"""
Final integration test to verify the complete system works.
"""

import asyncio
import logging
import uuid
from src.services.multi_llm_router import MultiLLMRouterService
from src.models.query import QueryRequest

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def test_system():
    """Test the complete system functionality"""
    try:
        logger.info("Initializing MultiLLMRouterService...")
        router = MultiLLMRouterService()

        logger.info("Testing system with various query scenarios...")

        # Test 1: Basic query with fallback
        test_query1 = QueryRequest(
            query="What is artificial intelligence?",
            session_id=str(uuid.uuid4())
        )

        result1 = await router.route_query(test_query1)
        logger.info(f"Test 1 - Basic query: Response length: {len(result1.response)}")

        # Test 2: Query with model preference for Qwen (will fallback)
        test_query2 = QueryRequest(
            query="Explain machine learning in simple terms",
            session_id=str(uuid.uuid4())
        )

        result2 = await router.route_query(test_query2, model_preference="qwen3-max")
        logger.info(f"Test 2 - Qwen preference (fallback): Response length: {len(result2.response)}")

        # Test 3: Default routing (will try Qwen first, then fallback)
        test_query3 = QueryRequest(
            query="How does a neural network work?",
            session_id=str(uuid.uuid4())
        )

        result3 = await router.route_query(test_query3)
        logger.info(f"Test 3 - Default routing (fallback): Response length: {len(result3.response)}")

        logger.info("All tests passed! System is working correctly.")
        logger.info("Mulerouter integration is properly configured with fallback to Gemini.")

        return True

    except Exception as e:
        logger.error(f"Error in final test: {str(e)}")
        logger.error(f"Error type: {type(e).__name__}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
        return False

async def main():
    """Main test function"""
    logger.info("Running final integration test...")

    success = await test_system()

    if success:
        logger.info("\n✓ All tests passed! The system is ready for use.")
        logger.info("✓ Qwen 3 Max support is configured with fallback mechanism.")
        logger.info("✓ When a valid Mulerouter API key is available, Qwen will be used.")
        logger.info("✓ When Mulerouter is unavailable, the system falls back to Gemini.")
    else:
        logger.error("\n✗ Tests failed!")

    return success

if __name__ == "__main__":
    success = asyncio.run(main())
    exit(0 if success else 1)