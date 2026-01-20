import sys
import os

# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

from backend.src.services.vector_db_service import VectorDBService

print("Testing Qdrant connection...")

try:
    db_service = VectorDBService()
    print("SUCCESS: Connected to Qdrant cloud successfully!")

    # Test search functionality
    dummy_embedding = [0.1] * 768
    results = db_service.search_similar(
        query_embedding=dummy_embedding,
        collection_name="textbook_content",
        limit=1
    )
    print(f"SUCCESS: Search working, found {len(results)} results")

except Exception as e:
    print(f"ERROR: Qdrant connection failed - {e}")
    import traceback
    traceback.print_exc()