#!/usr/bin/env python3
"""
Debug script to test the exact API flow that happens in the chat endpoint
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

# Import the same way as the API router
from services.rag_service import RAGService

def test_api_flow():
    print("Testing the exact flow as used in the API...")

    # This mimics what happens in the chat endpoint
    print("\n1. Creating RAGService instance (like in chat endpoint)...")
    rag_service = RAGService()

    print("\n2. Calling generate_response with 'hello'...")
    result = rag_service.generate_response("hello")

    print(f"Response: {result['response'][:200]}...")
    print(f"References: {result['references']}")

    print("\n3. Testing other greetings...")
    for greeting in ["hi", "hey"]:
        result = rag_service.generate_response(greeting)
        print(f"   '{greeting}' -> Response starts with: {result['response'][:50]}..., Has refs: {bool(result['references'])}")

if __name__ == "__main__":
    test_api_flow()