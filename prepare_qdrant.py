import sys
import os
# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.dirname(os.path.abspath('.')))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

from backend.src.services.vector_db_service import VectorDBService

print("Setting up Qdrant for vector storage...")

# Initialize the VectorDBService (this will connect to local mode)
db_service = VectorDBService()

# Create the required collection
print("Creating 'textbook_content' collection...")
success = db_service.create_collection('textbook_content')
if success:
    print("SUCCESS: Collection 'textbook_content' created successfully!")
else:
    print("ERROR: Failed to create collection")

# Verify the collection exists
try:
    collections = db_service.client.get_collections()
    collection_names = [coll.name for coll in collections.collections]
    print(f"Available collections: {collection_names}")

    if 'textbook_content' in collection_names:
        print("SUCCESS: 'textbook_content' collection is ready for data storage!")
    else:
        print("ERROR: 'textbook_content' collection was not found")
except Exception as e:
    print(f"Error checking collections: {e}")

print("Qdrant is now ready for data ingestion!")