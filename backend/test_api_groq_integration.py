#!/usr/bin/env python3
"""
Test script to verify the full API endpoint with Groq integration.
"""

import requests
import json
import uuid
import time

# Server configuration
BASE_URL = "http://localhost:8000"

def test_query_endpoint():
    """Test the main query endpoint with Groq API integration"""
    print("Testing query endpoint with Groq API integration...")

    # Prepare the query payload
    query_payload = {
        "query": "Hello, how are you?",
        "session_id": str(uuid.uuid4()),
        "user_id": str(uuid.uuid4()),
        "metadata": {
            "test": True,
            "source": "api_test"
        }
    }

    headers = {
        "Content-Type": "application/json"
    }

    # Make the request to the API
    try:
        start_time = time.time()
        response = requests.post(f"{BASE_URL}/api/v1/query", json=query_payload, headers=headers)
        end_time = time.time()

        print(f"Response Status Code: {response.status_code}")
        print(f"Response Time: {end_time - start_time:.2f} seconds")

        if response.status_code == 200:
            response_data = response.json()
            print("Response Data:")
            print(json.dumps(response_data, indent=2))

            print("\n[SUCCESS] Query endpoint is working with Groq API!")
            print(f"  - Response length: {len(response_data.get('response', ''))} characters")
            if response_data.get('tokens_used'):
                tokens = response_data['tokens_used']
                print(f"  - Tokens used: Input={tokens.get('input_tokens')}, Output={tokens.get('output_tokens')}, Total={tokens.get('total_tokens')}")
            print(f"  - Sources: {len(response_data.get('sources', []))}")

            return True
        else:
            print(f"[FAILED] Query endpoint returned status {response.status_code}")
            print(f"Response: {response.text}")
            return False

    except Exception as e:
        print(f"[ERROR] Failed to call query endpoint - {str(e)}")
        return False

def test_different_queries():
    """Test the API with different types of queries"""
    print("\nTesting API with different types of queries...")

    test_queries = [
        "What is the capital of France?",
        "Explain quantum computing in simple terms.",
        "Write a short poem about technology.",
        "How does machine learning work?"
    ]

    all_successful = True

    for i, query_text in enumerate(test_queries, 1):
        print(f"\nTest {i}: {query_text}")

        query_payload = {
            "query": query_text,
            "session_id": str(uuid.uuid4())
        }

        headers = {
            "Content-Type": "application/json"
        }

        try:
            response = requests.post(f"{BASE_URL}/api/v1/query", json=query_payload, headers=headers)

            if response.status_code == 200:
                response_data = response.json()
                print(f"  [SUCCESS] Response length {len(response_data.get('response', ''))} chars")
                if response_data.get('tokens_used'):
                    tokens = response_data['tokens_used']
                    print(f"    Tokens: {tokens.get('input_tokens')}/{tokens.get('output_tokens')}/{tokens.get('total_tokens')}")
            else:
                print(f"  [FAILED] Failed with status {response.status_code}: {response.text}")
                all_successful = False

        except Exception as e:
            print(f"  [FAILED] Error: {str(e)}")
            all_successful = False

    return all_successful

def test_health_endpoint():
    """Test the query health endpoint"""
    print("\nTesting query health endpoint...")

    try:
        response = requests.get(f"{BASE_URL}/api/v1/query/health")
        print(f"Health check status: {response.status_code}")

        if response.status_code == 200:
            health_data = response.json()
            print(f"Health status: {health_data}")
            print("[SUCCESS] Query service health check passed")
            return True
        else:
            print(f"[FAILED] Health check failed: {response.text}")
            return False

    except Exception as e:
        print(f"[FAILED] Error checking health: {str(e)}")
        return False

def main():
    """Main test function"""
    print("Starting API integration tests with Groq API...")
    print("="*60)

    # Test the main query endpoint
    success1 = test_query_endpoint()

    # Test health endpoint
    success2 = test_health_endpoint()

    # Test different queries
    success3 = test_different_queries()

    print("\n" + "="*60)
    print("FINAL RESULTS:")
    print(f"Basic query test: {'PASS' if success1 else 'FAIL'}")
    print(f"Health check test: {'PASS' if success2 else 'FAIL'}")
    print(f"Different queries test: {'PASS' if success3 else 'FAIL'}")

    overall_success = success1 and success2 and success3

    if overall_success:
        print("\n[SUCCESS] ALL API INTEGRATION TESTS PASSED!")
        print("[SUCCESS] The system is working correctly with Groq API")
        print("[SUCCESS] 400 BAD REQUEST errors should be resolved")
        print("[SUCCESS] Full API integration is functioning properly")
    else:
        print("\n[FAILED] SOME TESTS FAILED")
        print("Please check the errors above and investigate.")

    return overall_success

if __name__ == "__main__":
    main()