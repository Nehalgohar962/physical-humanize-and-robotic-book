import sys
import os

# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

from backend.src.services.vector_db_service import VectorDBService

print("Creating 'textbook_content' collection in Qdrant...")

# Initialize the VectorDBService which will connect to your Qdrant instance
db_service = VectorDBService()

# Create the required collection
success = db_service.create_collection('textbook_content')
if success:
    print("[SUCCESS] Collection 'textbook_content' created successfully!")
    print("[SUCCESS] Qdrant is now ready for data ingestion!")

    # Verify the collection exists
    try:
        collections = db_service.client.get_collections()
        collection_names = [coll.name for coll in collections.collections]
        print(f"Available collections: {collection_names}")

        if 'textbook_content' in collection_names:
            print("✅ Verification: 'textbook_content' collection is confirmed to exist!")
        else:
            print("❌ Verification: 'textbook_content' collection was not found after creation")
    except Exception as e:
        print(f"Error verifying collections: {e}")
else:
    print("❌ Failed to create collection")

print("Collection setup complete!")