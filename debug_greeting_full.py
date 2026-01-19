#!/usr/bin/env python3
"""
Debug script to test the full greeting functionality in RAG service
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

from services.rag_service import RAGService
from services.module_index import handle_module_query

def test_full_flow():
    print("Testing the full flow step by step...")

    # Create a RAG service instance
    rag_service = RAGService()

    # Test query
    query = "hello"
    print(f"Testing query: '{query}'")

    # Test the handle_module_query function
    print("1. Testing handle_module_query...")
    module_response = handle_module_query(query)
    print(f"   handle_module_query returned: {module_response}")

    # Test the greeting detection logic manually
    print("\n2. Testing greeting detection logic...")
    query_lower = query.lower().strip()
    print(f"   query_lower: '{query_lower}'")

    greeting_words = ["hello", "hi", "hey", "greetings"]
    is_greeting = any(greeting in query_lower for greeting in greeting_words)
    print(f"   Is greeting: {is_greeting}")

    # Test the module-related query check
    has_module_words = "module" in query_lower or "modules" in query_lower or "name" in query_lower
    print(f"   Has module/name words: {has_module_words}")

    # Test the book content query check
    has_book_words = "book" in query_lower or "content" in query_lower
    print(f"   Has book/content words: {has_book_words}")

    # Now test the full generate_response method
    print("\n3. Testing full generate_response method...")
    result = rag_service.generate_response(query)
    print(f"   Response: {result['response'][:200]}...")
    print(f"   References: {result['references']}")

    print("\n4. Testing other greeting variations...")
    for greeting in ["hi", "hey", "Hello", "HELLO"]:
        print(f"   Testing '{greeting}':")
        result = rag_service.generate_response(greeting)
        print(f"     Response starts with: {result['response'][:50]}...")
        print(f"     Has references: {bool(result['references'])}")

if __name__ == "__main__":
    test_full_flow()