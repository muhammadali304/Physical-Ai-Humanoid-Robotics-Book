#!/usr/bin/env python3
"""
Direct test to Mulerouter API to understand the issue.
"""

import asyncio
import logging
import httpx
import json

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def test_direct_api_call():
    """Test the Mulerouter API with a direct HTTP call"""

    # Get the API key from environment
    import os
    from src.config.settings import settings

    api_key = settings.mulerouter_api_key
    if not api_key:
        logger.error("Mulerouter API key not found in settings")
        return False

    logger.info(f"Using API key: {'***' + api_key[-4:]}")

    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json"
    }

    # Test data
    data = {
        "model": "qwen3-max",
        "messages": [
            {"role": "user", "content": "Hello, how are you?"}
        ],
        "max_tokens": 100,
        "temperature": 0.7
    }

    url = "https://api.mulerouter.ai/v1/chat/completions"

    try:
        logger.info(f"Making request to: {url}")
        logger.info(f"Request data: {json.dumps(data, indent=2)}")

        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(url, headers=headers, json=data)

            logger.info(f"Response status: {response.status_code}")
            logger.info(f"Response headers: {dict(response.headers)}")
            logger.info(f"Response content: {response.text}")

            if response.status_code == 200:
                logger.info("SUCCESS: Direct API call worked!")
                return True
            else:
                logger.error(f"FAILED: Direct API call failed with status {response.status_code}")
                return False

    except Exception as e:
        logger.error(f"Error making direct API call: {str(e)}")
        logger.error(f"Error type: {type(e).__name__}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
        return False

async def main():
    """Main test function"""
    logger.info("Starting direct Mulerouter API test...")

    success = await test_direct_api_call()

    if success:
        logger.info("Direct API test: PASS")
    else:
        logger.error("Direct API test: FAIL")

if __name__ == "__main__":
    asyncio.run(main())