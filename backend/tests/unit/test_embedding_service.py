import pytest
from unittest.mock import Mock, patch
from backend.src.services.embedding_service import EmbeddingService


class TestEmbeddingService:
    @patch('backend.src.services.embedding_service.settings')
    def test_create_embedding(self, mock_settings):
        # Mock the settings
        mock_settings.openai_api_key = "test-key"

        # Create service instance
        service = EmbeddingService()

        # Mock the OpenAI client call
        with patch.object(service.client.embeddings, 'create') as mock_create:
            mock_create.return_value = Mock()
            mock_create.return_value.data = [Mock()]
            mock_create.return_value.data[0].embedding = [0.1, 0.2, 0.3]

            result = service.create_embedding("test text")

            # Verify the result
            assert result == [0.1, 0.2, 0.3]
            mock_create.assert_called_once()

    def test_count_tokens(self):
        service = EmbeddingService()
        result = service.count_tokens("test text")
        assert isinstance(result, int)
        assert result > 0

    def test_chunk_text(self):
        service = EmbeddingService()
        long_text = "This is a test. " * 100  # Create a long text
        chunks = service.chunk_text(long_text, max_tokens=50)

        assert len(chunks) > 0
        assert all(isinstance(chunk, str) for chunk in chunks)