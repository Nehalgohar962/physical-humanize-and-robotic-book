import sys
import os

# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

from backend.src.services.vector_db_service import VectorDBService

print("Checking detailed Qdrant client methods...")

# Initialize the VectorDBService
db_service = VectorDBService()

# Get all methods including private ones to see all available methods
all_methods = [method for method in dir(db_service.client)]
search_methods = [method for method in all_methods if 'search' in method.lower()]

print(f"All search-related methods: {search_methods}")

# Let's also try to find the correct search method by checking the search functionality
print("\nTrying to test search functionality...")

# Check if there are any points in the collection
try:
    collection_info = db_service.client.get_collection('textbook_content')
    print(f"Collection points count: {collection_info.points_count}")

    if collection_info.points_count > 0:
        # Try to perform a sample search with a dummy embedding
        sample_embedding = [0.1] * 768  # Dummy embedding

        # Try the standard search method
        try:
            results = db_service.client.search(
                collection_name="textbook_content",
                query_vector=sample_embedding,
                limit=1,
                with_payload=True
            )
            print("SUCCESS: Standard 'search' method worked!")
            print(f"Results: {len(results)} found")
        except AttributeError as e:
            print(f"ERROR: {e}")
            print("The 'search' method does not exist in this version of Qdrant client")

            # Try alternative methods
            if hasattr(db_service.client, 'search_points'):
                print("Trying 'search_points' method...")
                try:
                    results = db_service.client.search_points(
                        collection_name="textbook_content",
                        vector=sample_embedding,
                        limit=1,
                        with_payload=True
                    )
                    print("SUCCESS: 'search_points' method worked!")
                except Exception as e2:
                    print(f"ERROR with 'search_points': {e2}")
        except Exception as e:
            print(f"General error with search: {e}")
    else:
        print("No points in the collection to search")

except Exception as e:
    print(f"Error getting collection info: {e}")