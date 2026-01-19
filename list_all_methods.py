import sys
import os

# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

from backend.src.services.vector_db_service import VectorDBService

print("Listing all Qdrant client methods...")

# Initialize the VectorDBService
db_service = VectorDBService()

# Get all public methods
public_methods = [method for method in dir(db_service.client) if not method.startswith('_')]
print(f"Public methods ({len(public_methods)}):")
for i, method in enumerate(sorted(public_methods)):
    print(f"  {i+1:2d}. {method}")

print("\nLooking for search-related methods...")
search_like_methods = [method for method in public_methods if any(keyword in method.lower() for keyword in ['search', 'find', 'query', 'retrieve'])]
print(f"Search-like methods: {search_like_methods}")

# Check for methods that might be used for searching
potential_search_methods = []
for method in public_methods:
    if any(keyword in method.lower() for keyword in ['search', 'query', 'find', 'retrieve']):
        potential_search_methods.append(method)

if potential_search_methods:
    print(f"\nPotential search methods: {potential_search_methods}")
else:
    print("\nNo obvious search methods found.")

    # Check for collection-specific methods
    print("\nTrying to inspect collection object methods...")
    try:
        collection_obj = db_service.client.get_collection('textbook_content')
        print(f"Collection object type: {type(collection_obj)}")

        # Let's try to get the collection object differently
        print("\nTrying to use collection methods...")
        # Some versions of qdrant_client have methods that are called differently
        # Let's try to see what methods are available for search operations
        print("Looking for methods that might work for search...")

        # Test if there's a different syntax for search
        import qdrant_client
        print(f"Qdrant client version: {qdrant_client.__version__}")

    except Exception as e:
        print(f"Error: {e}")