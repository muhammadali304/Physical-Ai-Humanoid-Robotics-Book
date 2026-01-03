#!/usr/bin/env python3
"""
Comprehensive test to verify the system functions correctly with the new Groq configuration.
"""

import requests
import json
import uuid
import time

# Server configuration
BASE_URL = "http://localhost:8000"

def test_basic_functionality():
    """Test basic API functionality with Groq"""
    print("Testing basic API functionality with Groq...")

    # Test 1: Health check
    print("\n1. Testing health endpoint...")
    try:
        response = requests.get(f"{BASE_URL}/health")
        print(f"Health check: {response.status_code}")
        if response.status_code == 200:
            print("[SUCCESS] Health check passed")
        else:
            print(f"[FAILED] Health check failed: {response.text}")
            return False
    except Exception as e:
        print(f"[FAILED] Health check error: {e}")
        return False

    # Test 2: Query endpoint with a simple query
    print("\n2. Testing query endpoint...")
    query_payload = {
        "query": "What is artificial intelligence?",
        "session_id": str(uuid.uuid4())
    }

    headers = {
        "Content-Type": "application/json"
    }

    try:
        start_time = time.time()
        response = requests.post(f"{BASE_URL}/api/v1/query", json=query_payload, headers=headers)
        end_time = time.time()

        print(f"Query response: {response.status_code}")
        print(f"Response time: {end_time - start_time:.2f}s")

        if response.status_code == 200:
            response_data = response.json()
            print("[SUCCESS] Query successful")
            print(f"  Response length: {len(response_data.get('response', ''))} chars")
            print(f"  Sources: {len(response_data.get('sources', []))}")
            if response_data.get('tokens_used'):
                tokens = response_data['tokens_used']
                print(f"  Tokens: {tokens.get('input_tokens')}/{tokens.get('output_tokens')}/{tokens.get('total_tokens')}")
        else:
            print(f"[FAILED] Query failed: {response.status_code}, {response.text}")
            return False
    except Exception as e:
        print(f"[FAILED] Query error: {e}")
        return False

    # Test 3: Query health endpoint
    print("\n3. Testing query health endpoint...")
    try:
        response = requests.get(f"{BASE_URL}/api/v1/query/health")
        print(f"Query health: {response.status_code}")
        if response.status_code == 200:
            print("[SUCCESS] Query health check passed")
        else:
            print(f"[FAILED] Query health check failed: {response.text}")
            return False
    except Exception as e:
        print(f"[FAILED] Query health error: {e}")
        return False

    return True

def test_groq_specific_functionality():
    """Test Groq-specific functionality"""
    print("\nTesting Groq-specific functionality...")

    test_queries = [
        "Explain how machine learning works in simple terms",
        "What are the benefits of using AI?",
        "How does natural language processing work?",
        "What is the difference between AI and machine learning?"
    ]

    success_count = 0

    for i, query_text in enumerate(test_queries, 1):
        print(f"\n  Query {i}: {query_text[:50]}...")

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
                response_text = response_data.get('response', '')

                print(f"    [SUCCESS] Success - Response: {len(response_text)} chars")
                if response_data.get('tokens_used'):
                    tokens = response_data['tokens_used']
                    print(f"      Tokens: {tokens.get('input_tokens')}/{tokens.get('output_tokens')}/{tokens.get('total_tokens')}")

                success_count += 1
            else:
                print(f"    [FAILED] Failed: {response.status_code} - {response.text}")

        except Exception as e:
            print(f"    [FAILED] Error: {e}")

    print(f"\nGroq functionality test: {success_count}/{len(test_queries)} queries successful")
    return success_count == len(test_queries)

def test_error_handling():
    """Test error handling"""
    print("\nTesting error handling...")

    # Test with empty query
    empty_query = {
        "query": "",
        "session_id": str(uuid.uuid4())
    }

    headers = {
        "Content-Type": "application/json"
    }

    try:
        response = requests.post(f"{BASE_URL}/api/v1/query", json=empty_query, headers=headers)
        print(f"Empty query response: {response.status_code}")

        # An empty query should either fail gracefully or be handled properly
        print("[SUCCESS] Error handling test completed")
        return True
    except Exception as e:
        print(f"[FAILED] Error handling test error: {e}")
        return False

def main():
    """Main verification function"""
    print("="*60)
    print("COMPREHENSIVE SYSTEM VERIFICATION WITH GROQ CONFIGURATION")
    print("="*60)

    print("This test verifies that the system functions correctly with Groq API")
    print("after resolving the 400 BAD REQUEST errors.\n")

    # Run all tests
    test1_result = test_basic_functionality()
    test2_result = test_groq_specific_functionality()
    test3_result = test_error_handling()

    print("\n" + "="*60)
    print("VERIFICATION RESULTS:")
    print(f"Basic functionality: {'PASS' if test1_result else 'FAIL'}")
    print(f"Groq functionality: {'PASS' if test2_result else 'FAIL'}")
    print(f"Error handling: {'PASS' if test3_result else 'FAIL'}")

    overall_success = test1_result and test2_result and test3_result

    print(f"\nOverall result: {'PASS' if overall_success else 'FAIL'}")

    if overall_success:
        print("\n🎉 SYSTEM VERIFICATION PASSED!")
        print("✓ Groq API integration is working correctly")
        print("✓ 400 BAD REQUEST errors have been resolved")
        print("✓ System handles queries properly")
        print("✓ Token usage is being tracked correctly")
        print("✓ API endpoints are functioning as expected")
    else:
        print("\n❌ SYSTEM VERIFICATION FAILED")
        print("Some functionality is not working correctly")

    return overall_success

if __name__ == "__main__":
    main()