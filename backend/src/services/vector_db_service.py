# # from qdrant_client import QdrantClient
# # from qdrant_client.http import models
# # from typing import List, Dict, Any, Optional
# # import uuid
# # import sys
# # import os

# # # Allow absolute imports
# # sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# # from config import settings


# # class VectorDBService:
# #     def __init__(self):
# #         """
# #         Proper Qdrant Cloud initialization (FIXED)
# #         This version WORKS with Qdrant Cloud and avoids 404 errors
# #         """

# #         try:
# #             self.client = QdrantClient(
# #                 host="93464b93-75e3-496a-82f8-cba317f9e502.europe-west3-0.gcp.cloud.qdrant.io",
# #                 port=6333,
# #                 https=True,
# #                 api_key=settings.qdrant_api_key,
# #                 prefer_grpc=False,
# #                 check_compatibility=False
# #             )

# #             # Test connection
# #             self.client.get_collections()
# #             print("✅ Successfully connected to Qdrant Cloud")

# #         except Exception as e:
# #             print("❌ Qdrant connection failed")
# #             raise Exception(f"Qdrant connection failed: {e}")

# #     def create_collection(self, collection_name: str, vector_size: int = 768) -> bool:
# #         """
# #         Create collection if not exists
# #         """
# #         try:
# #             self.client.get_collection(collection_name)
# #             print(f"ℹ️ Collection already exists: {collection_name}")
# #             return True
# #         except:
# #             self.client.create_collection(
# #                 collection_name=collection_name,
# #                 vectors_config=models.VectorParams(
# #                     size=vector_size,
# #                     distance=models.Distance.COSINE
# #                 )
# #             )
# #             print(f"✅ Created collection: {collection_name}")
# #             return True

# #     def save_chunk_to_qdrant(
# #         self,
# #         text: str,
# #         embedding: List[float],
# #         source_url: str,
# #         collection_name: str,
# #         content_type: str = "paragraph",
# #         chunk_order: int = 0
# #     ) -> bool:
# #         """
# #         Save chunk into Qdrant
# #         """
# #         try:
# #             point = models.PointStruct(
# #                 id=str(uuid.uuid4()),
# #                 vector=embedding,
# #                 payload={
# #                     "text": text,
# #                     "source_url": source_url,
# #                     "content_type": content_type,
# #                     "chunk_order": chunk_order
# #                 }
# #             )

# #             self.client.upsert(
# #                 collection_name=collection_name,
# #                 points=[point]
# #             )
# #             return True

# #         except Exception as e:
# #             print(f"❌ Error saving chunk to Qdrant: {e}")
# #             return False

# #     def search_similar(
# #         self,
# #         query_embedding: List[float],
# #         collection_name: str,
# #         limit: int = 5,
# #         filters: Optional[Dict] = None
# #     ) -> List[Dict[str, Any]]:
# #         """
# #         Search similar vectors
# #         """
# #         try:
# #             qdrant_filter = None

# #             if filters:
# #                 conditions = []
# #                 for key, value in filters.items():
# #                     conditions.append(
# #                         models.FieldCondition(
# #                             key=key,
# #                             match=models.MatchValue(value=value)
# #                         )
# #                     )

# #                 if conditions:
# #                     qdrant_filter = models.Filter(must=conditions)

# #             results = self.client.search(
# #                 collection_name=collection_name,
# #                 query_vector=query_embedding,
# #                 limit=limit,
# #                 with_payload=True,
# #                 query_filter=qdrant_filter
# #             )

# #             formatted_results = []
# #             for r in results:
# #                 formatted_results.append({
# #                     "content_text": r.payload.get("text"),
# #                     "page_reference": r.payload.get("source_url"),
# #                     "similarity_score": r.score
# #                 })

# #             return formatted_results

# #         except Exception as e:
# #             print(f"❌ Error searching in Qdrant: {e}")
# #             return []




# from qdrant_client import QdrantClient
# from qdrant_client.http import models
# from typing import List, Dict, Any, Optional
# import uuid
# import os
# import sys

# sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# from config import settings


# class VectorDBService:
#     def __init__(self):
#         """
#         LOCAL FILE-BASED QDRANT (NO DOCKER, NO CLOUD)
#         """
#         try:
#             self.client = QdrantClient(
#                 path="./qdrant_data"   # <- THIS IS THE MAGIC LINE
#             )

#             self.client.get_collections()
#             print("✅ Connected to LOCAL FILE Qdrant")

#         except Exception as e:
#             raise Exception(f"Qdrant connection failed: {e}")

#     def create_collection(self, collection_name: str, vector_size: int = 768):
#         try:
#             self.client.get_collection(collection_name)
#         except:
#             self.client.create_collection(
#                 collection_name=collection_name,
#                 vectors_config=models.VectorParams(
#                     size=vector_size,
#                     distance=models.Distance.COSINE
#                 )
#             )

#     def save_chunk_to_qdrant(self, text, embedding, source_url, collection_name):
#         self.client.upsert(
#             collection_name=collection_name,
#             points=[
#                 models.PointStruct(
#                     id=str(uuid.uuid4()),
#                     vector=embedding,
#                     payload={
#                         "text": text,
#                         "source_url": source_url
#                     }
#                 )
#             ]
#         )

#     def search_similar(self, query_embedding, collection_name, limit=5):
#         results = self.client.search(
#             collection_name=collection_name,
#             query_vector=query_embedding,
#             limit=limit,
#             with_payload=True
#         )

#         return [
#             {
#                 "content_text": r.payload["text"],
#                 "page_reference": r.payload["source_url"],
#                 "similarity_score": r.score
#             }
#             for r in results
#         ]

from qdrant_client import QdrantClient
from qdrant_client.http import models
import uuid
from typing import List, Dict, Any


class VectorDBService:
    def __init__(self):
        """
        QDRANT IN-MEMORY MODE (NO CLOUD, NO DOCKER)
        """
        try:
            self.client = QdrantClient(location=":memory:")
            print("✅ Connected to IN-MEMORY Qdrant")
        except Exception as e:
            raise Exception(f"Qdrant connection failed: {e}")

    def create_collection(self, collection_name: str, vector_size: int = 768):
        """
        Create collection if it does not exist
        """
        try:
            self.client.get_collection(collection_name)
        except:
            self.client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            )
            print(f"✅ Created collection: {collection_name}")

    def save_chunk_to_qdrant(
        self,
        text: str,
        embedding: List[float],
        source_url: str,
        collection_name: str
    ):
        """
        Save a single text chunk into Qdrant
        """
        point = models.PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload={
                "text": text,
                "source_url": source_url
            }
        )

        self.client.upsert(
            collection_name=collection_name,
            points=[point]
        )

        print(f"✅ Saved chunk to collection: {collection_name}")

    def search_similar(
        self,
        query_embedding: List[float],
        collection_name: str,
        limit: int = 5
    ) -> List[Dict[str, Any]]:
        """
        Search similar vectors (NEW QDRANT API)
        """
        try:
            results = self.client.query_points(
                collection_name=collection_name,
                query=query_embedding,
                limit=limit,
                with_payload=True
            )

            formatted_results = []

            # results.points contains matches
            for r in results.points:
                formatted_results.append({
                    "content_text": r.payload.get("text", ""),
                    "page_reference": r.payload.get("source_url", ""),
                    "similarity_score": r.score
                })

            print(f"🔎 Found {len(formatted_results)} similar chunks in '{collection_name}'")
            return formatted_results

        except Exception as e:
            print(f"❌ Error searching in Qdrant: {e}")
            return []

