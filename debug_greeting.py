#!/usr/bin/env python3
"""
Debug script to test the greeting functionality directly
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

from services.rag_service import RAGService

def test_greeting():
    print("Testing greeting functionality directly...")

    rag_service = RAGService()

    # Test different greeting variations
    greetings = ["hello", "hi", "hey", "Hello", "HELLO", "  hello  "]

    for greeting in greetings:
        print(f"\nTesting: '{greeting}'")
        result = rag_service.generate_response(greeting)
        print(f"Response: {result['response'][:100]}...")
        print(f"References: {result['references']}")

if __name__ == "__main__":
    test_greeting()