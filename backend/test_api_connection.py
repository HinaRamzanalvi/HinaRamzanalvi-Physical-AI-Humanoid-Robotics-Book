"""
Test script to verify backend API is working properly
"""
import requests
import json

def test_api_connection():
    base_url = "http://localhost:8000"

    # Test health endpoint
    try:
        response = requests.get(f"{base_url}/api/v1/health")
        print(f"Health check: {response.status_code} - {response.json()}")
    except Exception as e:
        print(f"Health check failed: {e}")

    # Test chat endpoint
    try:
        payload = {
            "query_text": "Hello, are you working?",
            "query_mode": "AskBook"
        }
        response = requests.post(
            f"{base_url}/api/v1/chat/query",
            headers={"Content-Type": "application/json"},
            data=json.dumps(payload)
        )
        print(f"Chat endpoint: {response.status_code}")
        if response.status_code == 200:
            print(f"Response: {response.json()}")
        else:
            print(f"Error: {response.text}")
    except Exception as e:
        print(f"Chat endpoint test failed: {e}")

if __name__ == "__main__":
    test_api_connection()