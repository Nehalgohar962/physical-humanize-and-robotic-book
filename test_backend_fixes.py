#!/usr/bin/env python3
"""
Test script to verify the backend fixes for OpenAI API and session handling issues.
"""

import requests
import json
import sys
import os

# Add the backend/src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

def test_api_endpoints():
    """Test the API endpoints to verify fixes are working"""
    base_url = "http://localhost:8000"

    print("Testing API endpoints...")

    # Test basic health check
    try:
        response = requests.get(f"{base_url}/health")
        print(f"Health check: {response.status_code} - {response.json()}")
    except Exception as e:
        print(f"Health check failed: {e}")

    # Test detailed health check (new endpoint)
    try:
        response = requests.get(f"{base_url}/api/health")
        print(f"Detailed health check: {response.status_code}")
        health_data = response.json()
        print(f"LLM connection status: {health_data.get('llm_connection', 'N/A')}")
    except Exception as e:
        print(f"Detailed health check failed: {e}")

    # Test session creation
    try:
        response = requests.post(f"{base_url}/api/session/sessions", json={})
        print(f"Session creation: {response.status_code}")
        if response.status_code == 200:
            session_data = response.json()
            session_id = session_data.get('id')
            print(f"Created session: {session_id}")

            # Test getting the session
            if session_id:
                response = requests.get(f"{base_url}/api/session/sessions/{session_id}")
                print(f"Get session: {response.status_code}")

    except Exception as e:
        print(f"Session operations failed: {e}")

    # Test chat session creation (if using the chat router)
    try:
        response = requests.post(f"{base_url}/api/chat/sessions", json={})
        print(f"Chat session creation: {response.status_code}")
        if response.status_code == 200:
            session_data = response.json()
            session_id = session_data.get('session_id')  # Note: different response format
            print(f"Created chat session: {session_id}")

    except Exception as e:
        print(f"Chat session creation failed: {e}")

def test_llm_connection():
    """Test the LLM connection directly"""
    print("\nTesting LLM connection directly...")

    # Import and test the RAG service
    try:
        from backend.src.services.rag_service import RAGService
        rag_service = RAGService()
        result = rag_service.test_llm_connection()
        print(f"LLM connection test result: {result}")
        return result.get('success', False)
    except Exception as e:
        print(f"LLM connection test failed: {e}")
        return False

def main():
    print("Testing backend fixes for LLM API and session handling...")

    # First test the LLM connection
    llm_ok = test_llm_connection()

    if not llm_ok:
        print("\n⚠️  LLM connection test failed. Please check your API key and quota.")
        print("Make sure your .env file has a valid GEMINI_API_KEY with sufficient quota.")
    else:
        print("\n✅ LLM connection test passed!")

    # Test API endpoints
    test_api_endpoints()

    print("\nTesting complete. If LLM connection is working, try making a chat request:")
    print("curl -X POST http://localhost:8000/api/chat/sessions -d '{}' -H 'Content-Type: application/json'")
    print("curl -X POST http://localhost:8000/api/chat/chat -d '{\"session_id\":\"<session_id>\",\"message\":\"Hello\"}' -H 'Content-Type: application/json'")

if __name__ == "__main__":
    main()