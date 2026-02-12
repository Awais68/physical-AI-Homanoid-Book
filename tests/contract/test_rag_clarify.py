"""Contract tests for RAG-enhanced clarification."""

import json

from fastapi.testclient import TestClient

from src.api.main import app
from src.text_processor.models import Citation, RAGContext


class TestRAGClarifyContract:
    """Contract tests for RAG-enhanced clarification endpoint."""

    def test_process_request_with_rag_enabled(self):
        """Test that process endpoint accepts use_rag parameter."""
        client = TestClient(app)
        response = client.post(
            "/process",
            json={
                "user_text": "test text",
                "target_language": "en",
                "use_rag": True,
            },
        )

        assert response.status_code == 200
        data = response.json()

        # Verify required fields exist
        assert "original_text" in data
        assert "translated_text" in data
        assert data["original_text"] == "test text"

    def test_rag_response_includes_citations(self):
        """Test that RAG response includes citations when available."""
        client = TestClient(app)

        # Mock response (in real scenario, RAG client would retrieve docs)
        response = client.post(
            "/process",
            json={
                "user_text": "test text",
                "target_language": "en",
                "use_rag": True,
            },
        )

        # Response should include citations field (even if empty)
        data = response.json()
        assert "citations" in data

    def test_rag_context_in_response(self):
        """Test that RAG context is included in response."""
        client = TestClient(app)
        response = client.post(
            "/process",
            json={
                "user_text": "test text",
                "target_language": "en",
                "use_rag": True,
            },
        )

        data = response.json()

        # RAG context should be present (even if None)
        assert "rag_context" in data

    def test_use_rag_default_false(self):
        """Test that use_rag defaults to False."""
        client = TestClient(app)
        response = client.post(
            "/process",
            json={
                "user_text": "test text",
                "target_language": "en",
            },
        )

        assert response.status_code == 200
        data = response.json()

        # RAG context should still be present but None
        assert "rag_context" in data
        assert data["rag_context"] is None
        assert "citations" in data
        assert len(data["citations"]) == 0

    def test_rag_citation_schema(self):
        """Test that citations follow correct schema."""
        # Create mock citation
        citation = Citation(
            source_index=0,
            title="Test Document",
            source_url="https://example.com/doc",
            file_path="/docs/test.md",
            snippet="Test content snippet",
            relevance_score=0.85,
        )

        # Verify required fields
        assert citation.source_index == 0
        assert citation.title == "Test Document"
        assert citation.source_url == "https://example.com/doc"
        assert citation.file_path == "/docs/test.md"
        assert citation.snippet == "Test content snippet"
        assert citation.relevance_score == 0.85
        assert 0.0 <= citation.relevance_score <= 1.0

    def test_rag_context_schema(self):
        """Test that RAG context follows correct schema."""
        rag_context = RAGContext(
            query="test query",
            documents=[],
            retrieval_time_ms=150,
            formatted_context="Test formatted context",
        )

        # Verify required fields
        assert rag_context.query == "test query"
        assert rag_context.documents == []
        assert rag_context.retrieval_time_ms == 150
        assert rag_context.formatted_context == "Test formatted context"

    def test_process_without_rag(self):
        """Test that standard process works (use_rag=false or omitted)."""
        client = TestClient(app)
        response = client.post(
            "/process",
            json={
                "user_text": "test text",
                "target_language": "en",
                "use_rag": False,
            },
        )

        assert response.status_code == 200
        data = response.json()

        # Standard response fields
        assert "original_text" in data
        assert "translated_text" in data
        assert "rag_context" in data
        assert "citations" in data
