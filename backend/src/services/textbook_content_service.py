# import sys
# import os
# from typing import List, Dict, Any, Optional

# # Add the backend/src directory to the Python path to allow absolute imports
# sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# from models.textbook_content import TextbookContentCreate
# from services.embedding_service import EmbeddingService
# from services.vector_db_service import VectorDBService
# import uuid


# class TextbookContentService:
#     def __init__(self):
#         self.embedding_service = EmbeddingService()
#         self.vector_db_service = VectorDBService()

#     def process_and_store_content(
#         self,
#         chapter_id: str,
#         section_title: str,
#         content_text: str,
#         page_reference: str
#     ) -> bool:
#         """
#         Process textbook content by creating embeddings and storing in vector database
#         """
#         try:
#             # Create embedding for the content
#             embedding = self.embedding_service.embed(content_text)

#             # Generate a unique ID for this content
#             content_id = str(uuid.uuid4())

#             # Store in vector database
#             success = self.vector_db_service.save_chunk_to_qdrant(
#                 text=content_text,
#                 embedding=embedding,
#                 source_url=f"chapter_{chapter_id}_section_{section_title}",
#                 collection_name="textbook_content",
#                 content_type="textbook_content"
#             )

#             return success
#         except Exception as e:
#             print(f"Error processing and storing content: {e}")
#             return False

#     def batch_process_and_store(
#         self,
#         content_list: List[Dict[str, Any]]
#     ) -> bool:
#         """
#         Process and store multiple textbook content items in batch
#         """
#         try:
#             processed_contents = []

#             for content in content_list:
#                 # Create embedding for the content
#                 embedding = self.embedding_service.embed(
#                     content["content_text"]
#                 )

#                 processed_contents.append({
#                     "content_id": str(uuid.uuid4()),
#                     "embedding": embedding,
#                     "content_text": content["content_text"],
#                     "chapter_id": content["chapter_id"],
#                     "section_title": content["section_title"],
#                     "page_reference": content["page_reference"]
#                 })

#             # Store all in vector database one by one
#             success = True
#             for content in processed_contents:
#                 result = self.vector_db_service.save_chunk_to_qdrant(
#                     text=content["content_text"],
#                     embedding=content["embedding"],
#                     source_url=f"chapter_{content['chapter_id']}_section_{content['section_title']}",
#                     collection_name="textbook_content",
#                     content_type="textbook_content"
#                 )
#                 if not result:
#                     success = False
#             return success
#         except Exception as e:
#             print(f"Error batch processing and storing content: {e}")
#             return False

#     def search_content(
#         self,
#         query: str,
#         limit: int = 5,
#         module_filter: Optional[str] = None
#     ) -> List[Dict[str, Any]]:
#         """
#         Search for textbook content similar to the query
#         """
#         try:
#             # Ensure the collection exists before searching
#             self.vector_db_service.create_collection("textbook_content")

#             # Create embedding for the query
#             query_embedding = self.embedding_service.embed(query)

#             # Prepare filters if module is specified
#             filters = None
#             if module_filter:
#                 # Use the same pattern as in search_content_by_module
#                 filters = {"source_url_contains": f"chapter_{module_filter}"}

#             # Search in vector database
#             results = self.vector_db_service.search_similar(
#                 query_embedding=query_embedding,
#                 collection_name="textbook_content",
#                 limit=limit,
#                 filters=filters
#             )

#             return results
#         except Exception as e:
#             print(f"Error searching content: {e}")
#             return []

#     def search_content_by_module(self, query: str, module_id: str, limit: int = 5) -> List[Dict[str, Any]]:
#         """
#         Search for content specifically within a given module
#         """
#         try:
#             # Ensure the collection exists before searching
#             self.vector_db_service.create_collection("textbook_content")

#             # Create embedding for the query
#             query_embedding = self.embedding_service.embed(query)

#             # Search in vector database with module filter
#             # Use more specific filtering to match the exact module pattern used during ingestion
#             results = self.vector_db_service.search_similar(
#                 query_embedding=query_embedding,
#                 collection_name="textbook_content",
#                 limit=limit,
#                 filters={"source_url_contains": f"chapter_{module_id}"}
#             )

#             return results
#         except Exception as e:
#             print(f"Error searching content by module: {e}")
#             return []

#     def search_content_fallback(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
#         """
#         Fallback search without module filtering to ensure content retrieval
#         """
#         try:
#             # Ensure the collection exists before searching
#             self.vector_db_service.create_collection("textbook_content")

#             # Create embedding for the query
#             query_embedding = self.embedding_service.embed(query)

#             # Search in vector database without filters
#             results = self.vector_db_service.search_similar(
#                 query_embedding=query_embedding,
#                 collection_name="textbook_content",
#                 limit=limit
#             )

#             return results
#         except Exception as e:
#             print(f"Error in fallback search: {e}")
#             return []

#     def chunk_and_process_text(
#         self,
#         chapter_id: str,
#         section_title: str,
#         full_text: str,
#         page_reference: str,
#         max_tokens: int = 1000
#     ) -> bool:
#         """
#         Split text into chunks and process each chunk
#         """
#         try:
#             # Use text_chunker service instead
#             from services.text_chunker import TextChunker
#             chunker = TextChunker()
#             chunks = chunker.chunk_text(full_text, max_tokens, max_tokens//10)  # 10% overlap

#             success_count = 0
#             for i, chunk in enumerate(chunks):
#                 chunk_page_ref = f"{page_reference}-chunk-{i+1}"
#                 chunk_section_title = f"{section_title} (Chunk {i+1})"

#                 if self.process_and_store_content(
#                     chapter_id=chapter_id,
#                     section_title=chunk_section_title,
#                     content_text=chunk,
#                     page_reference=chunk_page_ref
#                 ):
#                     success_count += 1

#             return success_count == len(chunks)
#         except Exception as e:
#             print(f"Error chunking and processing text: {e}")
#             return False






# import os
# from dotenv import load_dotenv
# from sentence_transformers import SentenceTransformer
# import google.generativeai as genai

# # 🔹 Load .env file from project root
# BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
# ENV_PATH = os.path.join(BASE_DIR, ".env")
# load_dotenv(ENV_PATH)

# class TextbookContentService:
#     def __init__(self):
#         print("✅ TextbookContentService initialized (GEMINI MODE)")

#         # Embedding model
#         self.embedding_model = SentenceTransformer("all-mpnet-base-v2")

#         # Get API key from .env
#         api_key = os.getenv("GEMINI_API_KEY")
#         print("DEBUG GEMINI KEY =", api_key)  # check if key loaded

#         if not api_key:
#             raise ValueError("❌ GEMINI_API_KEY not found in .env file")

#         # Configure Gemini
#         genai.configure(api_key=api_key)
#         self.model = genai.GenerativeModel("gemini-1.5-flash")

#     # Convert text to embedding
#     def embed_query(self, text: str):
#         return self.embedding_model.encode(text).tolist()

#     # Textbook chunks
#     def get_all_textbook_chunks(self):
#         return [
#             {
#                 "text": "ROS 2 is a robotics middleware framework designed for distributed systems and real-time humanoid robot control.",
#                 "source": "chapter_1_page_10"
#             },
#             {
#                 "text": "Humanoid robots use ROS 2 nodes to modularize perception, planning, and control components.",
#                 "source": "chapter_2_page_25"
#             },
#             {
#                 "text": "DDS communication in ROS 2 enables reliable data exchange between humanoid robot sensors and actuators.",
#                 "source": "chapter_3_page_40"
#             },
#             {
#                 "text": "ROS 2 improves scalability and fault tolerance compared to ROS 1, making it suitable for complex humanoid robots.",
#                 "source": "chapter_4_page_58"
#             }
#         ]

#     # Generate answer using Gemini
#     def generate_answer(self, query: str, context: str):
#         prompt = f"""
# You are an expert robotics tutor.

# Answer the question using ONLY the textbook context below.

# Context:
# {context}

# Question:
# {query}

# Answer clearly and concisely.
# """
#         response = self.model.generate_content(prompt)
#         return response.text.strip()




import os
from dotenv import load_dotenv
from sentence_transformers import SentenceTransformer
import numpy as np

# ==============================
# GEMINI IMPORT
# ==============================
try:
    import google.generativeai as genai
except ImportError:
    raise ImportError("❌ pip install google-generativeai")

# ==============================
# LOAD ENV
# ==============================
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
ENV_PATH = os.path.join(BASE_DIR, ".env")

load_dotenv(dotenv_path=ENV_PATH)


class TextbookContentService:

    # =========================================================
    # INIT
    # =========================================================
    def __init__(self):
        print("✅ TextbookContentService initialized (REAL RAG MODE)")

        self.embedding_model = SentenceTransformer("all-mpnet-base-v2")

        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise ValueError("❌ GEMINI_API_KEY missing in .env")

        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel("models/gemini-2.5-flash")

        # preload chunks + embeddings
        self.chunks = self.get_all_textbook_chunks()
        self.chunk_embeddings = [
            self.embed_query(chunk["text"]) for chunk in self.chunks
        ]

    # =========================================================
    # EMBEDDING
    # =========================================================
    def embed_query(self, text: str):
        return self.embedding_model.encode(text)

    # =========================================================
    # COSINE SIMILARITY
    # =========================================================
    def cosine_similarity(self, a, b):
        return np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))

    # =========================================================
    # RETRIEVE RELEVANT CONTEXT
    # =========================================================
    def retrieve_context(self, query, top_k=3):

        query_embedding = self.embed_query(query)

        scores = []
        for emb in self.chunk_embeddings:
            score = self.cosine_similarity(query_embedding, emb)
            scores.append(score)

        # top matches
        top_indices = np.argsort(scores)[-top_k:][::-1]

        context = ""
        for idx in top_indices:
            context += self.chunks[idx]["text"] + "\n\n"

        return context.strip()

    # =========================================================
    # TEXTBOOK KNOWLEDGE
    # =========================================================
    def get_all_textbook_chunks(self):
        return [

            {
                "text": """A robot relies on a software framework that acts as its digital nervous system.
This system manages communication between sensors, actuators, and computational units.
Robot Operating System (ROS) is the standard framework used for this purpose.
ROS 2 is the second generation of ROS and improves performance, scalability, and security.""",
                "source": "module1_intro"
            },

            {
                "text": """In ROS 2, a node is the smallest unit of computation.
Each node performs one specific task.
Nodes make robotics systems modular, flexible, and easier to maintain.""",
                "source": "module1_nodes_definition"
            },

            {
                "text": """ROS 2 provides three main communication mechanisms:
topics, services, and actions.
These allow nodes to exchange data and coordinate robot behavior.""",
                "source": "module1_comm"
            },

            {
                "text": """Topics use a publish-subscribe communication model.
Publishers send data and subscribers receive it asynchronously.""",
                "source": "module1_topics"
            },

            {
                "text": """Services use request-response communication.
A client sends a request and waits for a response from a server.""",
                "source": "module1_services"
            },

            {
                "text": """Actions manage long-running tasks.
They provide feedback during execution and a final result when finished.""",
                "source": "module1_actions"
            }
        ]

    # =========================================================
    # MAIN RAG ANSWER FUNCTION
    # =========================================================
    def answer_question(self, query: str):

        context = self.retrieve_context(query)

        if not context:
            return "I could not find the answer in the textbook."

        prompt = f"""
You are a robotics professor.

RULES:
Answer ONLY from textbook.
If not found say:
"I could not find the answer in the textbook."

TEXTBOOK:
{context}

QUESTION:
{query}

ANSWER:
"""

        try:
            response = self.model.generate_content(prompt)

            if not response or not response.text:
                return "I could not find the answer in the textbook."

            return response.text.strip()

        except Exception as e:
            print("Gemini error:", e)
            return "AI error occurred."
