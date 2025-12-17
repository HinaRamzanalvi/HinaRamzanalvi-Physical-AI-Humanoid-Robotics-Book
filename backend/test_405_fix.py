"""
Test script to verify the 405 Method Not Allowed error fix
"""
import requests
import json

def test_405_error_handling():
    base_url = "http://localhost:8000"

    print("Testing 405 Method Not Allowed error handling...")

    # Start the server in the background first
    print("Please start the server with: python -m src.main")
    input("Press Enter once the server is running...")

    # Test accessing a POST-only endpoint with GET method
    # This should trigger a 405 error that gets mapped to CHAT_007 instead of CHAT_405
    try:
        # Try to GET the chat query endpoint (which only accepts POST)
        response = requests.get(f"{base_url}/api/v1/chat/query")
        print(f"GET /api/v1/chat/query response: {response.status_code}")
        print(f"Response content: {response.text}")

        # Check if the error code is now CHAT_007 instead of CHAT_405
        try:
            response_json = response.json()
            error_code = response_json.get('error', {}).get('code', 'NO_CODE')
            print(f"Error code received: {error_code}")

            if error_code == "CHAT_007":
                print("✅ SUCCESS: 405 error is now properly mapped to CHAT_007")
                return True
            elif error_code == "CHAT_405":
                print("❌ FAILURE: 405 error is still mapped to CHAT_405")
                return False
            else:
                print(f"⚠️  UNEXPECTED: Error code is {error_code}, expected CHAT_007")
                return False
        except json.JSONDecodeError:
            print("⚠️  Response is not JSON format")
            return False

    except Exception as e:
        print(f"Test failed with exception: {e}")
        return False

if __name__ == "__main__":
    test_405_error_handling()