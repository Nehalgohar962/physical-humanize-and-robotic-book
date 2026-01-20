import sys
import os

# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

from backend.src.services.vector_db_service import VectorDBService

print("Checking Qdrant client methods...")

# Initialize the VectorDBService
db_service = VectorDBService()

# Check available methods in the client
available_methods = [method for method in dir(db_service.client) if not method.startswith('_')]
print("Available Qdrant client methods:")
search_related = [method for method in available_methods if 'search' in method.lower()]
print(f"Search-related methods: {search_related}")

# Check if the search method exists
if hasattr(db_service.client, 'search'):
    print("[SUCCESS] 'search' method exists in Qdrant client")
else:
    print("[ERROR] 'search' method does NOT exist in Qdrant client")

if hasattr(db_service.client, 'search_points'):
    print("[SUCCESS] 'search_points' method exists in Qdrant client")
else:
    print("[ERROR] 'search_points' method does NOT exist in Qdrant client")

# Check collection exists
collections = db_service.client.get_collections()
collection_names = [coll.name for coll in collections.collections]
print(f"Available collections: {collection_names}")

if 'textbook_content' in collection_names:
    print("✅ 'textbook_content' collection exists")

    # Check collection info
    collection_info = db_service.client.get_collection('textbook_content')
    print(f"Collection vectors count: {collection_info.points_count}")
else:
    print("❌ 'textbook_content' collection does not exist")