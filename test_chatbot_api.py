import requests
import json

# Base URL for the running API
BASE_URL = "http://localhost:8000"

def test_chatbot_api():
    print("Testing RAG Chatbot API...")
    print("="*50)

    # Step 1: Create a session
    print("1. Creating a new session...")
    session_response = requests.post(f"{BASE_URL}/api/session/sessions")
    if session_response.status_code == 200:
        session_data = session_response.json()
        session_id = session_data['id']
        print(f"   SUCCESS: Session created with ID: {session_id}")
    else:
        print(f"   ERROR: Failed to create session: {session_response.status_code}")
        return

    # Step 2: Send a test message to the chatbot
    print("\n2. Sending a test message to the chatbot...")
    chat_payload = {
        "session_id": session_id,
        "message": "What is Physical AI?",
        "mode": "full_book"
    }

    chat_response = requests.post(f"{BASE_URL}/api/chat/chat", json=chat_payload)
    if chat_response.status_code == 200:
        response_data = chat_response.json()
        print(f"   SUCCESS: Received response from chatbot")
        print(f"   Response: {response_data['response'][:100]}...")
        if 'references' in response_data:
            print(f"   References: {response_data['references']}")
    else:
        print(f"   ERROR: Failed to get response: {chat_response.status_code}")
        print(f"   Error details: {chat_response.text}")

    # Step 3: Test another query
    print("\n3. Sending another test message...")
    chat_payload2 = {
        "session_id": session_id,
        "message": "Explain humanoid robot design principles",
        "mode": "full_book"
    }

    chat_response2 = requests.post(f"{BASE_URL}/api/chat/chat", json=chat_payload2)
    if chat_response2.status_code == 200:
        response_data2 = chat_response2.json()
        print(f"   SUCCESS: Received second response from chatbot")
        print(f"   Response: {response_data2['response'][:100]}...")
    else:
        print(f"   ERROR: Failed to get second response: {chat_response2.status_code}")

    print("\n" + "="*50)
    print("Chatbot API test completed!")
    print(f"API is accessible at: {BASE_URL}")
    print(f"Documentation: {BASE_URL}/docs")

if __name__ == "__main__":
    test_chatbot_api()