#!/usr/bin/env python3
"""
Test script to check API configuration and connectivity
"""
import sys
import os

# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

from backend.src.config import settings
from backend.src.services.rag_service import RAGService

def test_api_connection():
    """Test the API connection"""
    print("Testing API configuration and connectivity...")
    print("=" * 50)

    print(f"LLM Provider: {settings.llm_provider}")
    print(f"Cohere API Key available: {'Yes' if settings.cohere_api_key else 'No'}")
    print(f"OpenAI API Key available: {'Yes' if settings.openai_api_key else 'No'}")
    print(f"Gemini API Key available: {'Yes' if settings.gemini_api_key else 'No'}")
    print(f"Qdrant URL: {settings.qdrant_url}")
    print(f"Qdrant API Key available: {'Yes' if settings.qdrant_api_key else 'No'}")

    print("\nTesting RAG service initialization...")
    try:
        rag_service = RAGService()
        print("+ RAG service initialized successfully")

        print("\nTesting LLM connection...")
        result = rag_service.test_llm_connection()
        print(f"LLM connection test result: {result}")

        if result['success']:
            print("+ LLM connection successful")
        else:
            print(f"- LLM connection failed: {result.get('error', 'Unknown error')}")

    except Exception as e:
        print(f"- Error initializing RAG service: {e}")
        import traceback
        print(f"Full traceback: {traceback.format_exc()}")

    print("=" * 50)

if __name__ == "__main__":
    test_api_connection()