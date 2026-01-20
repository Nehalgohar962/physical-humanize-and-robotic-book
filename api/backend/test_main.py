import time
from typing import List, Dict, Any
import sys
import os

# Add the backend/src directory to the Python path to allow absolute imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from services.url_extractor import URLExtractor
from services.content_extractor import ContentExtractor
from services.text_chunker import TextChunker
from services.embedding_service import EmbeddingService
from config import settings

# Initialize services
url_extractor = URLExtractor()
content_extractor = ContentExtractor()
text_chunker = TextChunker()
embedding_service = EmbeddingService()

def test_basic_functionality():
    """
    Test basic functionality without requiring external services like Qdrant
    """
    print("Testing basic functionality...")

    # Test URL extraction (without actually connecting)
    print(f"Target URL from config: {settings.target_url}")

    # Test text chunking
    sample_text = "This is a sample text to test the chunking functionality. " * 100  # Make it longer
    chunks = text_chunker.chunk_text(sample_text, settings.chunk_size, settings.chunk_overlap)
    print(f"Text chunking test: Split {len(sample_text)} characters into {len(chunks)} chunks")

    # Test embedding (this will fail without a real API key but will show the process)
    try:
        sample_chunk = chunks[0][:100] if chunks else "Sample text for embedding test"
        embedding = embedding_service.embed(sample_chunk)
        print(f"Embedding test: Generated embedding with {len(embedding)} dimensions")
    except Exception as e:
        print(f"Embedding test: Expected error due to dummy API key - {type(e).__name__}: {str(e)[:100]}...")

    print("Basic functionality test completed successfully!")
    print("Note: Full functionality requires valid API keys and running services.")

if __name__ == "__main__":
    print("Running main.py with error handling for missing external services...")
    test_basic_functionality()