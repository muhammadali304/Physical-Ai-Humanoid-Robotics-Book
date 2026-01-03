#!/usr/bin/env python3
"""
Test OpenRouter API (since Mulerouter might be using OpenRouter as the base service).
"""

import asyncio
import logging
import httpx
import json

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def test_openrouter_api():
    """Test the OpenRouter API with the provided API key"""

    from src.config.settings import settings

    api_key = settings.mulerouter_api_key
    if not api_key:
        logger.error("Mulerouter API key not found in settings")
        return False

    logger.info(f"Testing with API key: {'***' + api_key[-4:]}")

    # Try the OpenRouter endpoint
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json"
    }

    # Test data for OpenRouter
    data = {
        "model": "qwen/qwen3-max:free",  # OpenRouter format for Qwen
        "messages": [
            {"role": "user", "content": "Hello, how are you?"}
        ],
        "max_tokens": 100,
        "temperature": 0.7
    }

    url = "https://openrouter.ai/api/v1/chat/completions"

    try:
        logger.info(f"Making request to: {url}")
        logger.info(f"Request data: {json.dumps(data, indent=2)}")

        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(url, headers=headers, json=data)

            logger.info(f"Response status: {response.status_code}")
            logger.info(f"Response headers: {dict(response.headers)}")

            response_text = response.text
            logger.info(f"Response content: {response_text[:500]}...")  # First 500 chars

            if response.status_code == 200:
                logger.info("SUCCESS: OpenRouter API call worked!")
                return True
            else:
                logger.error(f"FAILED: OpenRouter API call failed with status {response.status_code}")
                return False

    except Exception as e:
        logger.error(f"Error making OpenRouter API call: {str(e)}")
        logger.error(f"Error type: {type(e).__name__}")
        import traceback
        logger.error(f"Full traceback: {traceback.format_exc()}")
        return False

async def main():
    """Main test function"""
    logger.info("Starting OpenRouter API test...")

    success = await test_openrouter_api()

    if success:
        logger.info("OpenRouter API test: PASS")
    else:
        logger.error("OpenRouter API test: FAIL")

if __name__ == "__main__":
    asyncio.run(main())