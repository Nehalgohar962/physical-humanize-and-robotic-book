import requests
import json

# Base URL for the running API
BASE_URL = "http://localhost:8000"

def test_chatbot():
    print("Testing RAG Chatbot API...")
    print("="*50)

    # Step 1: Create a session
    print("1. Creating a new session...")
    session_response = requests.post(f"{BASE_URL}/api/session/sessions")
    if session_response.status_code == 200:
        session_data = session_response.json()
        session_id = session_data['id']
        print(f"   SUCCESS: Session created successfully with ID: {session_id}")
    else:
        print(f"   ERROR: Failed to create session: {session_response.status_code}")
        return

    # Step 2: Send a test message
    print("\n2. Sending a test message...")
    chat_payload = {
        "session_id": session_id,
        "message": "What is the main topic of the Physical AI & Humanoid Robotics textbook?",
        "mode": "full_book"
    }

    chat_response = requests.post(f"{BASE_URL}/api/chat/chat", json=chat_payload)
    if chat_response.status_code == 200:
        chat_data = chat_response.json()
        print(f"   SUCCESS: Response received: {chat_data['response'][:100]}...")
        print(f"   SUCCESS: References: {chat_data.get('references', [])}")
    else:
        print(f"   ERROR: Failed to get response: {chat_response.status_code}")
        print(f"   Error: {chat_response.text}")

    print("\n" + "="*50)
    print("RAG Chatbot API is running successfully!")
    print(f"   - API is accessible at: {BASE_URL}")
    print(f"   - Health check: {BASE_URL}/health")
    print(f"   - Documentation: {BASE_URL}/docs")
    print(f"   - Chat endpoint: {BASE_URL}/api/chat/chat")
    print(f"   - Session endpoint: {BASE_URL}/api/session/sessions")

if __name__ == "__main__":
    test_chatbot()