import sys
import os
from typing import List, Dict, Any, Optional

# Add the backend directory to the Python path to allow absolute imports
current_dir = os.path.dirname(os.path.abspath(__file__))
backend_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, backend_dir)

from models.textbook_content import TextbookContentCreate
from services.embedding_service import EmbeddingService
from services.vector_db_service import VectorDBService
import uuid


class TextbookContentService:
    def __init__(self):
        self.embedding_service = EmbeddingService()
        self.vector_db_service = VectorDBService()

    def process_and_store_content(
        self,
        chapter_id: str,
        section_title: str,
        content_text: str,
        page_reference: str
    ) -> bool:
        """
        Process textbook content by creating embeddings and storing in vector database
        """
        try:
            # Create embedding for the content
            embedding = self.embedding_service.embed(content_text)

            # Generate a unique ID for this content
            content_id = str(uuid.uuid4())

            # Store in vector database
            success = self.vector_db_service.save_chunk_to_qdrant(
                text=content_text,
                embedding=embedding,
                source_url=f"chapter_{chapter_id}_section_{section_title}",
                collection_name="textbook_content",
                content_type="textbook_content"
            )

            return success
        except Exception as e:
            print(f"Error processing and storing content: {e}")
            return False

    def batch_process_and_store(
        self,
        content_list: List[Dict[str, Any]]
    ) -> bool:
        """
        Process and store multiple textbook content items in batch
        """
        try:
            processed_contents = []

            for content in content_list:
                # Create embedding for the content
                embedding = self.embedding_service.embed(
                    content["content_text"]
                )

                processed_contents.append({
                    "content_id": str(uuid.uuid4()),
                    "embedding": embedding,
                    "content_text": content["content_text"],
                    "chapter_id": content["chapter_id"],
                    "section_title": content["section_title"],
                    "page_reference": content["page_reference"]
                })

            # Store all in vector database one by one
            success = True
            for content in processed_contents:
                result = self.vector_db_service.save_chunk_to_qdrant(
                    text=content["content_text"],
                    embedding=content["embedding"],
                    source_url=f"chapter_{content['chapter_id']}_section_{content['section_title']}",
                    collection_name="textbook_content",
                    content_type="textbook_content"
                )
                if not result:
                    success = False
            return success
        except Exception as e:
            print(f"Error batch processing and storing content: {e}")
            return False

    def search_content(
        self,
        query: str,
        limit: int = 5,
        module_filter: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for textbook content similar to the query
        """
        try:
            # Ensure the collection exists before searching
            self.vector_db_service.create_collection("textbook_content")

            # Create embedding for the query
            query_embedding = self.embedding_service.embed(query)

            # Prepare filters if module is specified
            filters = None
            if module_filter:
                # Use the same pattern as in search_content_by_module
                filters = {"source_url_contains": f"chapter_{module_filter}"}

            # Search in vector database
            results = self.vector_db_service.search_similar(
                query_embedding=query_embedding,
                collection_name="textbook_content",
                limit=limit,
                filters=filters
            )

            return results
        except Exception as e:
            print(f"Error searching content: {e}")
            return []

    def search_content_by_module(self, query: str, module_id: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for content specifically within a given module
        """
        try:
            # Ensure the collection exists before searching
            self.vector_db_service.create_collection("textbook_content")

            # Create embedding for the query
            query_embedding = self.embedding_service.embed(query)

            # Search in vector database with module filter
            # Use more specific filtering to match the exact module pattern used during ingestion
            results = self.vector_db_service.search_similar(
                query_embedding=query_embedding,
                collection_name="textbook_content",
                limit=limit,
                filters={"source_url_contains": f"chapter_{module_id}"}
            )

            return results
        except Exception as e:
            print(f"Error searching content by module: {e}")
            return []

    def search_content_fallback(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Fallback search without module filtering to ensure content retrieval
        """
        try:
            # Ensure the collection exists before searching
            self.vector_db_service.create_collection("textbook_content")

            # Create embedding for the query
            query_embedding = self.embedding_service.embed(query)

            # Search in vector database without filters
            results = self.vector_db_service.search_similar(
                query_embedding=query_embedding,
                collection_name="textbook_content",
                limit=limit
            )

            return results
        except Exception as e:
            print(f"Error in fallback search: {e}")
            return []

    def chunk_and_process_text(
        self,
        chapter_id: str,
        section_title: str,
        full_text: str,
        page_reference: str,
        max_tokens: int = 1000
    ) -> bool:
        """
        Split text into chunks and process each chunk
        """
        try:
            # Use text_chunker service instead
            from services.text_chunker import TextChunker
            chunker = TextChunker()
            chunks = chunker.chunk_text(full_text, max_tokens, max_tokens//10)  # 10% overlap

            success_count = 0
            for i, chunk in enumerate(chunks):
                chunk_page_ref = f"{page_reference}-chunk-{i+1}"
                chunk_section_title = f"{section_title} (Chunk {i+1})"

                if self.process_and_store_content(
                    chapter_id=chapter_id,
                    section_title=chunk_section_title,
                    content_text=chunk,
                    page_reference=chunk_page_ref
                ):
                    success_count += 1

            return success_count == len(chunks)
        except Exception as e:
            print(f"Error chunking and processing text: {e}")
            return False