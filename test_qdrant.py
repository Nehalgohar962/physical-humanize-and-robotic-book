import os
import sys
# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath('.'))))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

from backend.src.services.vector_db_service import VectorDBService
from backend.src.config import settings

print('Testing Qdrant connection...')
try:
    db_service = VectorDBService()
    print('Qdrant client initialized successfully')
    # Try to create a collection to test the connection
    success = db_service.create_collection('test_collection')
    print(f'Collection creation test: {success}')
    print('Qdrant is connected and working!')
except Exception as e:
    print(f'Error connecting to Qdrant: {e}')
    print('Make sure Qdrant is running on', settings.qdrant_url)