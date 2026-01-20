import pytest
from backend.src.services.text_chunker import TextChunker


class TestTextChunker:
    def test_chunk_text_with_small_text(self):
        chunker = TextChunker()
        text = "This is a short text."
        chunks = chunker.chunk_text(text, chunk_size=100, overlap=10)
        assert len(chunks) == 1
        assert chunks[0] == text

    def test_chunk_text_with_large_text(self):
        chunker = TextChunker()
        text = "This is a longer text. " * 50  # Create a longer text
        chunks = chunker.chunk_text(text, chunk_size=50, overlap=5)

        assert len(chunks) > 1
        assert all(len(chunk) <= 50 for chunk in chunks)

        # Check that chunks overlap correctly (approximately)
        if len(chunks) > 1:
            # The overlap should cause some text to appear in both chunks
            assert len(chunks) > 1

    def test_chunk_text_with_empty_text(self):
        chunker = TextChunker()
        chunks = chunker.chunk_text("", chunk_size=100, overlap=10)
        assert chunks == []

    def test_chunk_text_with_exact_size(self):
        chunker = TextChunker()
        text = "A" * 100  # Exactly 100 characters
        chunks = chunker.chunk_text(text, chunk_size=100, overlap=10)
        assert len(chunks) == 1
        assert chunks[0] == text