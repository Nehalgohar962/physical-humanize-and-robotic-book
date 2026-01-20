#!/usr/bin/env python3
"""
Test script to verify the Vercel deployment works properly
"""
import os
import sys
import json

# Add the api directory to the path
sys.path.insert(0, './api')

def test_imports():
    """Test that all necessary imports work"""
    try:
        from api.index import app, settings, RAGService
        print("v All imports successful")
        return True
    except ImportError as e:
        print(f"x Import error: {e}")
        return False

def test_settings():
    """Test that settings are loaded properly"""
    try:
        from api.index import settings

        print("v Settings loaded:")
        print(f"  - LLM Provider: {settings.llm_provider}")
        print(f"  - Qdrant URL: {'SET' if settings.qdrant_url else 'NOT SET'}")
        print(f"  - Chunk Size: {settings.chunk_size}")
        return True
    except Exception as e:
        print(f"x Settings error: {e}")
        return False

def test_rag_service():
    """Test that RAG service can be instantiated"""
    try:
        from api.index import RAGService
        rag_service = RAGService()
        print("v RAG Service instantiated successfully")
        return True
    except Exception as e:
        print(f"x RAG Service error: {e}")
        return False

def main():
    print("Testing Vercel deployment readiness...")
    print("="*50)

    all_tests_passed = True

    print("\n1. Testing imports...")
    if not test_imports():
        all_tests_passed = False

    print("\n2. Testing settings...")
    if not test_settings():
        all_tests_passed = False

    print("\n3. Testing RAG service...")
    if not test_rag_service():
        all_tests_passed = False

    print("\n" + "="*50)
    if all_tests_passed:
        print("v All tests passed! Ready for Vercel deployment.")
        return 0
    else:
        print("x Some tests failed. Please fix the issues before deployment.")
        return 1

if __name__ == "__main__":
    sys.exit(main())