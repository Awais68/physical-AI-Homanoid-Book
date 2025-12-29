"""Unit tests for RAG client module."""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch

from openai import OpenAI

from src.text_processor.rag_client import RAGClient, format_citation_list
from src.text_processor.models import Citation, RAGContext


@pytest.fixture
def mock_openai_client():
    """Fixture for mocked OpenAI client."""
    return MagicMock(spec=OpenAI)


@pytest.fixture
def mock_qdrant_client():
    """Fixture for mocked Qdrant client."""
    qdrant_mock = MagicMock()
    qdrant_mock.get_collections.return_value = MagicMock(
        collections=[
            MagicMock(name="physical_ai_docs"),
        ]
    )
    return qdrant_mock


@pytest.fixture
def rag_client_with_mocks(mock_openai_client, mock_qdrant_client):
    """Fixture for RAG client with mocked dependencies."""
    with patch("src.text_processor.rag_client.QdrantClient", return_value=mock_qdrant_client):
        client = RAGClient(openai_client=mock_openai_client)
        return client


class TestRAGClientInit:
    """Test RAG client initialization."""

    def test_init_when_rag_disabled(self, mock_openai_client, monkeypatch):
        """Test that client initializes correctly when RAG is disabled."""
        monkeypatch.setenv("RAG_ENABLED", "False")
        client = RAGClient(openai_client=mock_openai_client)

        assert client.enabled is False
        assert client.qdrant is None

    def test_init_with_missing_collection(
        self, mock_openai_client, mock_qdrant_client, monkeypatch
    ):
        """Test that client disables itself when collection doesn't exist."""
        monkeypatch.setenv("RAG_ENABLED", "True")
        monkeypatch.setenv("QDRANT_COLLECTION", "nonexistent_collection")

        # Mock qdrant to return empty collections
        mock_qdrant_client.get_collections.return_value = MagicMock(collections=[])

        client = RAGClient(openai_client=mock_openai_client)

        assert client.enabled is False


class TestRAGContextRetrieval:
    """Test RAG context retrieval."""

    def test_retrieve_context_when_disabled(self, rag_client_with_mocks):
        """Test that empty context is returned when RAG is disabled."""
        rag_client_with_mocks.enabled = False

        result = rag_client_with_mocks.retrieve_context("test query")

        assert result.query == "test query"
        assert result.documents == []
        assert result.retrieval_time_ms == 0

    @pytest.mark.asyncio
    async def test_retrieve_context_success(
        self, rag_client_with_mocks, mock_openai_client
    ):
        """Test successful context retrieval."""
        rag_client_with_mocks.enabled = True

        # Mock embedding response
        mock_embedding = MagicMock()
        mock_embedding.data = [MagicMock(embedding=[0.1, 0.2, 0.3] * 1536)]
        mock_openai_client.embeddings = AsyncMock()
        mock_openai_client.embeddings.create = AsyncMock(return_value=mock_embedding)

        # Mock Qdrant search response
        mock_point = MagicMock(
            score=0.85,
            payload={
                "title": "Test Document",
                "content": "This is test content",
                "source_url": "https://example.com/doc",
            },
        )
        rag_client_with_mocks.qdrant.search.return_value = [mock_point]

        result = await rag_client_with_mocks.retrieve_context("test query")

        assert result.query == "test query"
        assert len(result.documents) == 1
        assert result.documents[0].title == "Test Document"
        assert result.documents[0].relevance_score == 0.85

    @pytest.mark.asyncio
    async def test_retrieve_context_with_score_threshold(
        self, rag_client_with_mocks, mock_openai_client
    ):
        """Test that documents below threshold are filtered."""
        rag_client_with_mocks.enabled = True
        rag_client_with_mocks.settings.rag_similarity_threshold = 0.8

        # Mock embedding response
        mock_embedding = MagicMock()
        mock_embedding.data = [MagicMock(embedding=[0.1] * 1536)]
        mock_openai_client.embeddings = AsyncMock(return_value=mock_embedding)

        # Mock Qdrant search response - one below threshold
        mock_low_score = MagicMock(
            score=0.75,
            payload={"title": "Low Score", "content": "Content"},
        )
        mock_high_score = MagicMock(
            score=0.85,
            payload={"title": "High Score", "content": "Content"},
        )
        rag_client_with_mocks.qdrant.search.return_value = [
            mock_low_score,
            mock_high_score,
        ]

        result = await rag_client_with_mocks.retrieve_context("test query")

        # Should filter out document below threshold
        assert len(result.documents) == 1
        assert result.documents[0].title == "High Score"


class TestCitationFormatting:
    """Test citation formatting functions."""

    def test_format_citation_list_empty(self):
        """Test formatting empty citation list."""
        citations = []
        result = format_citation_list(citations)

        assert result == []

    def test_format_citation_list(self):
        """Test formatting citations for JSON response."""
        citations = [
            Citation(
                source_index=0,
                title="Document 1",
                source_url="https://example.com/1",
                file_path="/docs/1.md",
                snippet="Test snippet",
                relevance_score=0.9,
            ),
        ]

        result = format_citation_list(citations)

        assert len(result) == 1
        assert result[0]["index"] == 1
        assert result[0]["title"] == "Document 1"
        assert result[0]["source_url"] == "https://example.com/1"
        assert result[0]["snippet"] == "Test snippet"
        assert result[0]["relevance_score"] == 0.9

    def test_rag_context_model(self):
        """Test RAGContext model validation."""
        context = RAGContext(
            query="test query",
            documents=[],
            retrieval_time_ms=100,
            formatted_context="Context here",
        )

        assert context.query == "test query"
        assert context.documents == []
        assert context.retrieval_time_ms == 100
        assert context.formatted_context == "Context here"

    def test_citation_model_validation(self):
        """Test Citation model with invalid score."""
        with pytest.raises(Exception):
            Citation(
                source_index=0,
                title="Test",
                source_url=None,
                file_path=None,
                snippet="Content",
                relevance_score=1.5,  # Invalid > 1.0
            )
