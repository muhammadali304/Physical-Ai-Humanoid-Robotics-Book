#!/usr/bin/env python3
"""
Test script to verify Mulerouter API connection with Qwen3-Max model.
"""

import asyncio
import logging
import uuid
from src.services.multi_llm_router import MultiLLMRouterService
from src.models.query import QueryRequest

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def test_mulerouter_connection():
    """Test the Mulerouter API connection with Qwen3-Max"""
    try:
        logger.info("Initializing MultiLLMRouterService...")
        router = MultiLLMRouterService()

        logger.info("Testing Qwen API via Mulerouter...")

        # Create a test query with valid UUID
        test_query = QueryRequest(
            query="Hello, how are you?",
            session_id=str(uuid.uuid4())
        )

        # Test the Qwen call specifically
        result = await router._call_qwen(test_query)

        logger.info(f"Qwen response: {result.response}")
        logger.info(f"Token usage: {result.tokens_used}")

        return True

    except Exception as e:
        logger.error(f"Error testing Mulerouter connection: {str(e)}")
        logger.error(f"Error type: {type(e).__name__}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
        return False

async def test_routing():
    """Test the routing logic"""
    try:
        logger.info("Testing multi-LLM routing...")
        router = MultiLLMRouterService()

        test_query = QueryRequest(
            query="Hello, how are you?",
            session_id=str(uuid.uuid4())
        )

        # Test routing with preference for Qwen
        result = await router.route_query(test_query, model_preference="qwen3-max")

        logger.info(f"Routed response: {result.response}")
        logger.info(f"Token usage: {result.tokens_used}")

        return True

    except Exception as e:
        logger.error(f"Error testing routing: {str(e)}")
        logger.error(f"Error type: {type(e).__name__}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
        return False

async def main():
    """Main test function"""
    logger.info("Starting Mulerouter API tests...")

    # Test direct Qwen call
    logger.info("\n1. Testing direct Qwen API call...")
    success1 = await test_mulerouter_connection()

    if success1:
        logger.info("Direct Qwen API call successful!")
    else:
        logger.error("Direct Qwen API call failed!")

    # Test routing
    logger.info("\n2. Testing multi-LLM routing...")
    success2 = await test_routing()

    if success2:
        logger.info("Multi-LLM routing successful!")
    else:
        logger.error("Multi-LLM routing failed!")

    logger.info(f"\nTest Results:")
    logger.info(f"Direct Qwen call: {'PASS' if success1 else 'FAIL'}")
    logger.info(f"Routing test: {'PASS' if success2 else 'FAIL'}")

if __name__ == "__main__":
    asyncio.run(main())