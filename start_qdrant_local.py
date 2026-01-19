import os
from qdrant_client import QdrantClient

# Create a local Qdrant instance using in-memory storage
# For persistent storage, you can use: QdrantClient(path="./qdrant_data")
print("Starting local Qdrant instance...")
client = QdrantClient(":memory:")

# Test creating a collection
try:
    client.create_collection(
        collection_name="textbook_content",
        vectors_config={
            "size": 768,  # Cohere's embed-multilingual-v2.0 produces 768-dimensional vectors
            "distance": "Cosine"
        }
    )
    print("Collection 'textbook_content' created successfully!")
    print("Local Qdrant is ready to use!")

    # Show collections
    collections = client.get_collections()
    print(f"Available collections: {[coll.name for coll in collections.collections]}")

except Exception as e:
    print(f"Error: {e}")

print("Local Qdrant is running in memory mode.")
print("You can now run the vector ingestion script.")