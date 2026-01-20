from typing import List, Dict, Any
import re


class TextSplitter:
    """
    Split large text documents into smaller chunks for vectorization
    """

    def __init__(self, chunk_size: int = 1000, chunk_overlap: int = 100):
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap

    def split_by_sentences(self, text: str) -> List[str]:
        """
        Split text by sentences, trying to keep chunks within size limits
        """
        # Split by sentence endings
        sentences = re.split(r'[.!?]+\s+', text)

        chunks = []
        current_chunk = ""

        for sentence in sentences:
            # Check if adding this sentence would exceed chunk size
            if len(current_chunk) + len(sentence) > self.chunk_size and current_chunk:
                chunks.append(current_chunk.strip())
                # Start new chunk with overlap from previous chunk
                if self.chunk_overlap > 0:
                    # Get the end of the current chunk for overlap
                    overlap_start = max(0, len(current_chunk) - self.chunk_overlap)
                    current_chunk = current_chunk[overlap_start:] + " " + sentence
                else:
                    current_chunk = sentence
            else:
                current_chunk += " " + sentence if current_chunk else sentence

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    def split_by_paragraphs(self, text: str) -> List[str]:
        """
        Split text by paragraphs
        """
        paragraphs = text.split('\n\n')
        chunks = []

        current_chunk = ""

        for paragraph in paragraphs:
            paragraph = paragraph.strip()
            if not paragraph:
                continue

            if len(current_chunk) + len(paragraph) > self.chunk_size and current_chunk:
                chunks.append(current_chunk.strip())
                current_chunk = paragraph
            else:
                current_chunk += "\n\n" + paragraph if current_chunk else paragraph

        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    def split_by_size(self, text: str) -> List[str]:
        """
        Split text into chunks of approximately equal size
        """
        words = text.split()
        chunks = []

        for i in range(0, len(words), self.chunk_size - self.chunk_overlap):
            chunk_words = words[i:i + self.chunk_size]
            chunk = " ".join(chunk_words)
            chunks.append(chunk)

        return chunks

    def split_textbook_content(
        self,
        content_items: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """
        Split multiple textbook content items, preserving metadata
        """
        split_content = []

        for item in content_items:
            # Split the content text
            chunks = self.split_by_sentences(item["content_text"])

            for i, chunk in enumerate(chunks):
                split_item = {
                    "chapter_id": item["chapter_id"],
                    "section_title": f"{item['section_title']} (Part {i+1})",
                    "content_text": chunk,
                    "page_reference": f"{item['page_reference']}-part-{i+1}"
                }
                split_content.append(split_item)

        return split_content