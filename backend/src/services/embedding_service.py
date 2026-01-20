import cohere
import sys
import os
import hashlib
from typing import List

# Add the backend/src directory to the Python path to allow absolute imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import settings


class EmbeddingService:
    def __init__(self):
        try:
            self.client = cohere.Client(settings.cohere_api_key)
            self.model = settings.cohere_model
            self.use_cohere = True
        except Exception as e:
            print(f"Error initializing Cohere client: {e}")
            print("Falling back to mock embeddings")
            self.use_cohere = False

    def _create_mock_embedding(self, text: str) -> List[float]:
        """Create a deterministic mock embedding for text"""
        # Create a hash of the text
        text_hash = hashlib.md5(text.encode('utf-8')).hexdigest()

        # Convert hash to a list of floats
        embedding = []
        for i in range(0, len(text_hash), 2):
            if i + 1 < len(text_hash):
                hex_pair = text_hash[i:i+2]
                # Convert hex to int and normalize to [-1, 1] range
                value = (int(hex_pair, 16) / 255.0) * 2 - 1
                embedding.append(value)

        # Pad or truncate to 768 dimensions (Cohere's embedding size)
        while len(embedding) < 768:
            embedding.append(0.0)
        embedding = embedding[:768]

        return embedding

    def embed(self, text: str) -> List[float]:
        """
        Generate embedding for a single text using Cohere or mock if API fails
        """
        if self.use_cohere:
            try:
                response = self.client.embed(
                    texts=[text],
                    model=self.model,
                    input_type="search_document"
                )
                return response.embeddings[0]  # Return the first (and only) embedding
            except Exception as e:
                print(f"Cohere API error: {e}")
                print("Falling back to mock embeddings")

        # Use mock embedding as fallback
        return self._create_mock_embedding(text)

    def embed_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a batch of texts using Cohere or mock if API fails
        """
        if self.use_cohere:
            try:
                response = self.client.embed(
                    texts=texts,
                    model=self.model,
                    input_type="search_document"
                )
                return response.embeddings
            except Exception as e:
                print(f"Cohere API error in batch: {e}")
                print("Falling back to mock embeddings")

        # Use mock embeddings as fallback
        return [self._create_mock_embedding(text) for text in texts]