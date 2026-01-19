#!/usr/bin/env python3
"""
Debug script to test the exact flow in the RAG service
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

from services.rag_service import RAGService

def test_flow():
    print("Testing the exact flow step by step...")

    # Create a RAG service instance
    rag_service = RAGService()

    # Test the handle_module_query function first
    from services.module_index import handle_module_query
    query = "hello"
    module_response = handle_module_query(query)
    print(f"handle_module_query('hello') returned: {module_response}")

    # Test query processing
    query_lower = query.lower().strip()
    greeting_words = ["hello", "hi", "hey", "greetings"]
    is_greeting = any(greeting in query_lower for greeting in greeting_words)
    print(f"Is greeting check: {is_greeting}")

    # Test if "hello" matches any module patterns
    from services.module_index import is_module_query
    is_mod_query, mod_num = is_module_query(query)
    print(f"is_module_query('hello') returned: is_query={is_mod_query}, module_num={mod_num}")

    # Test if "hello" contains "module", "modules", or "name"
    has_module_words = "module" in query_lower or "modules" in query_lower or "name" in query_lower
    print(f"Contains module/name words: {has_module_words}")

    # Test if "hello" contains "book" or "content"
    has_book_words = "book" in query_lower or "content" in query_lower
    print(f"Contains book/content words: {has_book_words}")

    # Now test the full generate_response
    print("\nTesting generate_response with 'hello':")
    result = rag_service.generate_response("hello")
    print(f"Response: {result['response'][:200]}...")
    print(f"References: {result['references']}")

if __name__ == "__main__":
    test_flow()