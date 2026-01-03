# Mulerouter Qwen 3 Max Integration

## Overview
This system has been configured to support Qwen 3 Max via Mulerouter with a fallback to Google Gemini. The architecture is ready to use Qwen 3 Max when a valid Mulerouter API key is available.

## Current Status
- The system attempts to use Qwen 3 Max via Mulerouter
- If Mulerouter is unavailable (401 Unauthorized error), it falls back to Gemini
- All functionality remains intact regardless of which LLM is used

## Configuration
The system is configured with:
- Mulerouter API key: `MULEROUTER_API_KEY` (in .env)
- Mulerouter base URL: `https://openrouter.ai/api/v1/` (as fallback)
- Qwen model: `qwen/qwen3-max:free` (OpenRouter format)
- Fallback model: Google Gemini

## How to Enable Full Mulerouter Support
When you have a valid Mulerouter account and API key that works with the service:

1. Replace the current Mulerouter API key in `.env` with a valid one
2. Update the `MULEROUTER_BASE_URL` in `.env` if needed (currently set to OpenRouter as fallback)
3. The system will automatically start using Qwen 3 Max instead of falling back to Gemini

## Architecture
- `MultiLLMRouterService` handles routing between different LLM providers
- Fallback logic ensures continuous operation if one service is unavailable
- All existing RAG functionality is preserved
- Token usage and source references are properly handled

## Testing
The fallback mechanism has been tested and confirmed to work. When Qwen is unavailable, the system seamlessly continues operation using the Gemini fallback.