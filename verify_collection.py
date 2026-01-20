import sys
import os

# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

from backend.src.services.vector_db_service import VectorDBService

print("Verifying collection contents in cloud Qdrant...")

# Initialize the VectorDBService
db_service = VectorDBService()

# Get collection info
try:
    collection_info = db_service.client.get_collection("textbook_content")
    print(f"Collection 'textbook_content' exists")
    print(f"Collection vectors count: {collection_info.points_count}")
    print(f"Collection config: {collection_info.config}")
    print("Collection verification successful!")
except Exception as e:
    print(f"Error getting collection info: {e}")
    import traceback
    traceback.print_exc()