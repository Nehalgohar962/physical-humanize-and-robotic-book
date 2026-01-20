import sys
import os
import random

# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.dirname(os.path.abspath('.')))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

print("Setting up Qdrant and ingesting textbook content...")

# Mock the embedding service to use dummy embeddings instead of calling Cohere
class MockEmbeddingService:
    def __init__(self):
        self.model = "mock-embedding-model"

    def embed(self, text: str) -> list:
        """
        Generate mock embedding (768-dimensional vector like Cohere's model)
        """
        return [random.random() for _ in range(768)]

# Monkey patch the embedding service to use mock embeddings
from backend.src.services import textbook_content_service
original_init = textbook_content_service.TextbookContentService.__init__

def new_init(self):
    self.embedding_service = MockEmbeddingService()
    from backend.src.services.vector_db_service import VectorDBService
    self.vector_db_service = VectorDBService()
    # Create the required collection
    self.vector_db_service.create_collection('textbook_content')

textbook_content_service.TextbookContentService.__init__ = new_init

# Import and run the ingestion
from vector_ingestion.src.vector_ingestor import VectorIngestor

ingestor = VectorIngestor()
print("Starting ingestion with mock embeddings...")
success = ingestor.ingest_sample_content()

if success:
    print("\nSUCCESS: Vector ingestion completed successfully!")
    print("Data has been saved to the local Qdrant instance.")
    print("The 'textbook_content' collection now contains the embedded textbook data.")
else:
    print("\nWARNING: Vector ingestion completed with some errors.")

# Verify the data was stored
from backend.src.services.vector_db_service import VectorDBService
db_service = VectorDBService()
try:
    collection_info = db_service.client.get_collection('textbook_content')
    print(f"\nCollection info: {collection_info.config.params.vectors_count} vectors stored")
    print("Data successfully saved to Qdrant!")
except Exception as e:
    print(f"Error verifying collection: {e}")