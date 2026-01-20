import sys
import os

# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

from backend.src.services.vector_db_service import VectorDBService

print("Testing updated search functionality...")

# Initialize the VectorDBService
db_service = VectorDBService()

# Test the search functionality with a dummy query
dummy_embedding = [0.1] * 768  # 768-dimensions for Cohere embeddings

try:
    results = db_service.search_similar(
        query_embedding=dummy_embedding,
        collection_name="textbook_content",
        limit=3
    )
    print(f"[SUCCESS] Search completed successfully! Found {len(results)} results")

    if results:
        print("Sample results:")
        for i, result in enumerate(results[:2]):  # Show first 2 results
            print(f"  {i+1}. Text: {result['content_text'][:50]}...")
            print(f"     Score: {result['similarity_score']}")
    else:
        print("  No results found (this is normal if the dummy embedding doesn't match)")

except Exception as e:
    print(f"[ERROR] Error in search functionality: {e}")
    import traceback
    traceback.print_exc()

print("\nTesting complete!")