#!/usr/bin/env python3
"""
Test script to verify the chatbot functionality after fixes
"""
import requests
import json
import time

BASE_URL = "http://127.0.0.1:8000"

def create_session():
    """Create a new chat session"""
    try:
        response = requests.post(f"{BASE_URL}/api/session/sessions", json={})
        if response.status_code == 200:
            session_data = response.json()
            print(f"OK Session created: {session_data['id']}")
            return session_data['id']
        else:
            print(f"X Failed to create session: {response.status_code} - {response.text}")
            return None
    except Exception as e:
        print(f"X Error creating session: {e}")
        return None

def test_chat_message(session_id, message, expected_contains=None):
    """Send a chat message and return the response"""
    try:
        payload = {
            "session_id": session_id,
            "message": message,
            "mode": "full_book"
        }
        response = requests.post(f"{BASE_URL}/api/chat/chat", json=payload)
        if response.status_code == 200:
            data = response.json()
            print(f"OK Query: '{message}'")
            print(f"  Response: {data['response'][:200]}...")
            if expected_contains:
                if any(phrase in data['response'] for phrase in expected_contains):
                    print(f"  OK Contains expected content: {expected_contains}")
                else:
                    print(f"  X Missing expected content: {expected_contains}")
            print()
            return data
        else:
            print(f"X Failed to get response for '{message}': {response.status_code} - {response.text}")
            return None
    except Exception as e:
        print(f"X Error sending message '{message}': {e}")
        return None

def main():
    print("Testing Chatbot Functionality After Fixes")
    print("=" * 60)

    # Create a session
    session_id = create_session()
    if not session_id:
        print("Cannot test without a session")
        return

    print("\n1. Testing Greeting Behavior:")
    print("-" * 30)

    # Test greeting queries
    greetings = ["hello", "hi", "hey"]
    for greeting in greetings:
        response = test_chat_message(session_id, greeting, ["Physical AI & Humanoid Robotics", "Module 1", "Module 2", "Module 3"])
        time.sleep(1)  # Small delay between requests

    print("\n2. Testing Module Queries:")
    print("-" * 30)

    # Test module queries
    module_queries = ["modules name?", "list modules", "what are the modules"]
    for query in module_queries:
        response = test_chat_message(session_id, query, ["Module 1", "Module 2", "Module 3", "Module 4", "Module 5", "Module 6"])
        time.sleep(1)

    print("\n3. Testing Book Content Queries:")
    print("-" * 30)

    # Test book content queries
    book_queries = ["tell me about book", "what is this book about", "book content"]
    for query in book_queries:
        response = test_chat_message(session_id, query, ["Physical AI", "Humanoid Robotics"])
        time.sleep(1)

    print("\n4. Testing Regular Queries (should use RAG if content is available):")
    print("-" * 30)

    # Test regular queries
    regular_queries = ["What are humanoid robots?", "Tell me about AI", "What is robotics?"]
    for query in regular_queries:
        response = test_chat_message(session_id, query)
        time.sleep(1)

    print("\n5. Testing Error Handling (if API issues occur):")
    print("-" * 30)

    # Test a few more queries to ensure system stability
    stability_queries = ["test", "hello again", "modules"]
    for query in stability_queries:
        response = test_chat_message(session_id, query)
        time.sleep(1)

    print("\nTest Complete!")
    print("All the chatbot should now respond properly to greetings, module queries,")
    print("and book content queries, with appropriate fallback messages when needed.")

if __name__ == "__main__":
    main()