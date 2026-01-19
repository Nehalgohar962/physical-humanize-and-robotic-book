import sys
import os
# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.dirname(os.path.abspath('.')))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

print("Running vector ingestion with mock embeddings (for testing purposes)...")

# Mock the embedding service to use dummy embeddings instead of calling Cohere
import random

class MockEmbeddingService:
    def __init__(self):
        self.model = "mock-embedding-model"

    def embed(self, text: str) -> list:
        """
        Generate mock embedding (768-dimensional vector like Cohere's model)
        """
        return [random.random() for _ in range(768)]

# Import the services
from vector_ingestion.src.vector_ingestor import VectorIngestor
from backend.src.services.textbook_content_service import TextbookContentService

# Monkey patch the embedding service to use mock embeddings
original_init = TextbookContentService.__init__

def mock_init(self):
    self.embedding_service = MockEmbeddingService()
    from backend.src.services.vector_db_service import VectorDBService
    self.vector_db_service = VectorDBService()

TextbookContentService.__init__ = mock_init

# Now run the ingestion
ingestor = VectorIngestor()
print("Starting ingestion with mock embeddings...")
success = ingestor.ingest_sample_content()

if success:
    print("\n✓ Vector ingestion completed successfully!")
    print("Data has been saved to the local Qdrant instance.")
    print("The 'textbook_content' collection now contains the embedded textbook data.")
else:
    print("\n✗ Vector ingestion completed with some errors.")