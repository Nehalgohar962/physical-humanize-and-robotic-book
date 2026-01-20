from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
import uuid
import sys
import os
import certifi
import ssl

# Add the backend directory to the Python path to allow absolute imports
current_dir = os.path.dirname(os.path.abspath(__file__))
backend_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, backend_dir)

from config import settings


class VectorDBService:
    def __init__(self):
        # Initialize Qdrant client with proper SSL configuration for Windows
        import urllib.parse

        # Parse the URL to check if it's HTTPS
        parsed_url = urllib.parse.urlparse(settings.qdrant_url)
        is_https = parsed_url.scheme == 'https'

        if settings.qdrant_api_key:
            if is_https:
                # For HTTPS connections, use certifi certificates
                self.client = QdrantClient(
                    url=settings.qdrant_url,
                    api_key=settings.qdrant_api_key,
                    prefer_grpc=False,  # Use REST API for better SSL compatibility
                    https=True,
                    verify=certifi.where()  # Use certifi's certificate bundle
                )
            else:
                # For HTTP connections (local development)
                self.client = QdrantClient(
                    url=settings.qdrant_url,
                    api_key=settings.qdrant_api_key,
                    prefer_grpc=True
                )
        else:
            # For cloud without auth
            if is_https:
                self.client = QdrantClient(
                    url=settings.qdrant_url,
                    prefer_grpc=False,  # Use REST API for better SSL compatibility
                    https=True,
                    verify=certifi.where()  # Use certifi's certificate bundle
                )
            else:
                self.client = QdrantClient(url=settings.qdrant_url)

        # Test connection immediately and fail if connection fails
        try:
            self.client.get_collections()
            print(f"Successfully connected to Qdrant at {settings.qdrant_url}")
        except Exception as e:
            print(f"Could not connect to Qdrant server at {settings.qdrant_url}: {e}")
            raise Exception(f"Qdrant connection failed: {e}")

    def create_collection(self, collection_name: str, vector_size: int = 768) -> bool:
        """
        Create a Qdrant collection with the specified name and vector size
        Note: Cohere's embed-multilingual-v2.0 produces 768-dimensional vectors
        """
        try:
            # Check if collection already exists
            self.client.get_collection(collection_name)
            print(f"Collection {collection_name} already exists")
            return True
        except:
            # Collection doesn't exist, create it
            self.client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            )
            print(f"Created collection {collection_name}")
            return True

    def save_chunk_to_qdrant(
        self,
        text: str,
        embedding: List[float],
        source_url: str,
        collection_name: str,
        content_type: str = "paragraph",
        chunk_order: int = 0
    ) -> bool:
        """
        Save a content chunk with its embedding to Qdrant
        """
        try:
            point = models.PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload={
                    "text": text,
                    "source_url": source_url,
                    "content_type": content_type,
                    "chunk_order": chunk_order,
                    "created_at": str(uuid.uuid4())  # This would be a proper timestamp in real implementation
                }
            )

            self.client.upsert(
                collection_name=collection_name,
                points=[point]
            )
            return True
        except Exception as e:
            print(f"Error saving chunk to Qdrant: {e}")
            return False

    def search_similar(
        self,
        query_embedding: List[float],
        collection_name: str,
        limit: int = 5,
        filters: Optional[Dict] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar content in the specified collection with optional filters
        """
        try:
            # Build Qdrant filter if filters are provided
            qdrant_filter = None
            if filters:
                conditions = []
                for key, value in filters.items():
                    if key == "source_url_contains":
                        # Filter by source URL containing a specific pattern (like module name)
                        # Use MatchText for broader matching instead of exact MatchValue
                        conditions.append(
                            models.FieldCondition(
                                key="source_url",
                                match=models.MatchText(text=value)
                            )
                        )
                    elif key == "source_url_exact":
                        # Filter by exact source URL match
                        conditions.append(
                            models.FieldCondition(
                                key="source_url",
                                match=models.MatchValue(value=value)
                            )
                        )
                    elif key.startswith("source_url_"):
                        # Generic source_url filtering
                        field_name = key.replace("source_url_", "")
                        conditions.append(
                            models.FieldCondition(
                                key="source_url",
                                match=models.MatchText(text=value)
                            )
                        )
                    elif key == "content_type":
                        conditions.append(
                            models.FieldCondition(
                                key="content_type",
                                match=models.MatchValue(value=value)
                            )
                        )

                if conditions:
                    qdrant_filter = models.Filter(
                        must=conditions
                    )

            print(f"DEBUG: Searching in collection: {collection_name}, limit: {limit}")
            print(f"DEBUG: Query embedding length: {len(query_embedding)}, first 5 values: {query_embedding[:5]}")
            print(f"DEBUG: Query filter: {qdrant_filter}")

            results = self.client.query_points(
                collection_name=collection_name,
                query=query_embedding,
                limit=limit,
                with_payload=True,
                query_filter=qdrant_filter  # Pass the filter to the search
            )

            print(f"DEBUG: Found {len(results.points)} results from query_points")

            formatted_results = []
            for result in results.points:
                formatted_results.append({
                    "content_text": result.payload["text"],
                    "page_reference": result.payload["source_url"],
                    "similarity_score": result.score
                })

            return formatted_results
        except Exception as e:
            print(f"Error searching in Qdrant: {e}")
            import traceback
            print(f"Full traceback: {traceback.format_exc()}")
            return []