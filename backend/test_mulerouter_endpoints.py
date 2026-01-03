#!/usr/bin/env python3
"""
Test different Mulerouter API endpoints to find the correct one.
"""

import asyncio
import logging
import httpx
import json

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def test_api_endpoint(endpoint, api_key):
    """Test a specific Mulerouter API endpoint"""

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

    try:
        logger.info(f"Testing endpoint: {endpoint}")
        logger.info(f"Request data: {json.dumps(data, indent=2)}")

        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(endpoint, headers=headers, json=data)

            logger.info(f"Response status: {response.status_code}")
            logger.info(f"Response content: {response.text[:200]}...")  # First 200 chars

            return response.status_code, response.text

    except Exception as e:
        logger.error(f"Error testing endpoint {endpoint}: {str(e)}")
        return None, str(e)

async def main():
    """Test multiple potential endpoints"""
    from src.config.settings import settings

    api_key = settings.mulerouter_api_key
    if not api_key:
        logger.error("Mulerouter API key not found in settings")
        return

    logger.info(f"Testing with API key: {'***' + api_key[-4:]}")

    # Common OpenAI-compatible API endpoints
    endpoints_to_test = [
        "https://api.mulerouter.ai/v1/chat/completions",  # Current
        "https://api.mulerouter.ai/chat/completions",     # Alternative
        "https://mulerouter.ai/v1/chat/completions",      # Alternative
        "https://api.mulerouter.ai/api/v1/chat/completions", # Alternative
        "https://api.mulerouter.ai/v1/openai/chat/completions", # Alternative
    ]

    results = {}
    for endpoint in endpoints_to_test:
        status, content = await test_api_endpoint(endpoint, api_key)
        results[endpoint] = (status, content)

        if status == 200:
            logger.info(f"SUCCESS: Found working endpoint: {endpoint}")
            break
        else:
            logger.info(f"---")

    logger.info("\nSummary of tests:")
    for endpoint, (status, content) in results.items():
        logger.info(f"{endpoint}: {status}")

if __name__ == "__main__":
    asyncio.run(main())