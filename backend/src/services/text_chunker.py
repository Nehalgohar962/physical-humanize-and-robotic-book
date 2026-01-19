import re
import sys
import os
from typing import List

# Add the backend/src directory to the Python path to allow absolute imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import settings


class TextChunker:
    def __init__(self):
        self.chunk_size = settings.chunk_size
        self.chunk_overlap = settings.chunk_overlap

    def chunk_text(self, text: str, chunk_size: int = None, overlap: int = None) -> List[str]:
        """
        Split text into chunks of specified size with overlap
        """
        if chunk_size is None:
            chunk_size = self.chunk_size
        if overlap is None:
            overlap = self.chunk_overlap

        if not text or len(text) <= chunk_size:
            return [text] if text else []

        chunks = []
        start = 0

        while start < len(text):
            end = start + chunk_size

            # If we're near the end, include the remainder
            if end >= len(text):
                chunks.append(text[start:])
                break

            # Try to break at sentence boundary if possible
            chunk = text[start:end]
            last_sentence_end = max(
                chunk.rfind('.'),
                chunk.rfind('!'),
                chunk.rfind('?'),
                chunk.rfind('\n'),
                chunk.rfind(';'),
                chunk.rfind(',')
            )

            # If we found a good break point and it's not too close to the start
            if last_sentence_end > len(chunk) // 2:
                actual_end = start + last_sentence_end + 1
                chunks.append(text[start:actual_end])
                start = actual_end - overlap
            else:
                # No good break point found, just cut at chunk_size
                chunks.append(text[start:end])
                start = end - overlap

            # Ensure we make progress
            if start <= end - overlap:
                start = end - overlap

        # Remove empty chunks and strip whitespace
        chunks = [chunk.strip() for chunk in chunks if chunk.strip()]
        return chunks