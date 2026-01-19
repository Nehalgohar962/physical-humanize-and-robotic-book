#!/usr/bin/env python3
"""
Test script to check embedding service functionality
"""
import sys
import os

# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

from backend.src.services.embedding_service import EmbeddingService

def test_embedding_service():
    """Test the embedding service"""
    print("Testing embedding service...")
    print("=" * 50)

    try:
        embedding_service = EmbeddingService()
        print("+ Embedding service initialized successfully")
        print(f"+ Using Cohere model: {embedding_service.model}")

        # Test embedding
        test_text = "This is a test sentence for embedding."
        print(f"\nTesting embedding for: '{test_text}'")

        embedding = embedding_service.embed(test_text)

        if embedding and len(embedding) > 0:
            print(f"+ Embedding successful: {len(embedding)} dimensions")
            print(f"+ First 5 dimensions: {embedding[:5]}")
        else:
            print("- Embedding failed or returned empty result")

    except Exception as e:
        print(f"- Error testing embedding service: {e}")
        import traceback
        print(f"Full traceback: {traceback.format_exc()}")

    print("=" * 50)

if __name__ == "__main__":
    test_embedding_service()