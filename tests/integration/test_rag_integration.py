"""Integration tests for RAG + clarification workflow."""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from fastapi.testclient import TestClient

from src.api.main import app
from src.text_processor.models import RAGContext, Citation


@pytest.fixture
def mock_rag_client():
    """Fixture for mocked RAG client."""
    rag_mock = MagicMock()

    # Mock RAG context retrieval
    mock_rag_client.retrieve_context = AsyncMock(
        return_value=RAGContext(
            query="test query",
            documents=[
                Citation(
                    source_index=0,
                    title="Physical AI Fundamentals",
                    source_url="https://example.com/fundamentals",
                    file_path="/docs/fundamentals.md",
                    snippet="Physical AI combines robotics with...",
                    relevance_score=0.92,
                )
            ],
            retrieval_time_ms=125,
            formatted_context="Based on following Physical AI documentation:\n\n[Source 1] Physical AI Fundamentals\nPhysical AI combines robotics with...",
        )
    )

    return rag_mock


class TestRAGIntegrationWorkflow:
    """End-to-end tests for RAG-enhanced clarification."""

    @pytest.mark.asyncio
    async def test_full_workflow_with_rag_enabled(self, mock_rag_client):
        """Test complete workflow: input → RAG retrieval → clarification → translation."""
        with patch("src.text_processor.rag_client.RAGClient", return_value=mock_rag_client):
            client = TestClient(app)

            response = client.post(
                "/process",
                json={
                    "user_text": "The robot kinematics are complex and need calibration",
                    "target_language": "en",
                    "use_rag": True,
                },
            )

            assert response.status_code == 200
            data = response.json()

            # Verify response structure
            assert "original_text" in data
            assert "translated_text" in data
            assert "rag_context" in data
            assert "citations" in data

            # Verify RAG data
            assert data["rag_context"]["query"] == "test query"
            assert len(data["rag_context"]["documents"]) == 1
            assert data["rag_context"]["retrieval_time_ms"] == 125

            # Verify citations
            assert len(data["citations"]) == 1
            assert data["citations"][0]["title"] == "Physical AI Fundamentals"
            assert data["citations"][0]["relevance_score"] == 0.92

    @pytest.mark.asyncio
    async def test_workflow_with_rag_no_documents(self, mock_rag_client):
        """Test workflow when RAG finds no relevant documents."""
        # Mock empty RAG context
        mock_rag_client.retrieve_context = AsyncMock(
            return_value=RAGContext(
                query="unrelated query",
                documents=[],
                retrieval_time_ms=50,
                formatted_context="",
            )
        )

        with patch("src.text_processor.rag_client.RAGClient", return_value=mock_rag_client):
            client = TestClient(app)
            response = client.post(
                "/process",
                json={
                    "user_text": "random unrelated text",
                    "target_language": "en",
                    "use_rag": True,
                },
            )

            assert response.status_code == 200
            data = response.json()

            # Should still work but without RAG data
            assert data["rag_context"]["documents"] == []
            assert len(data["citations"]) == 0
            assert data["rag_context"]["retrieval_time_ms"] == 50

    @pytest.mark.asyncio
    async def test_workflow_without_rag(self, mock_rag_client):
        """Test standard workflow without RAG enabled."""
        client = TestClient(app)
        response = client.post(
            "/process",
            json={
                "user_text": "simplify this text",
                "target_language": "en",
                "use_rag": False,
            },
        )

        assert response.status_code == 200
        data = response.json()

        # RAG context should be None
        assert data["rag_context"] is None
        assert len(data["citations"]) == 0

    @pytest.mark.asyncio
    async def test_rag_with_translation(self, mock_rag_client):
        """Test RAG-enabled workflow with non-English translation."""
        with patch("src.text_processor.rag_client.RAGClient", return_value=mock_rag_client):
            client = TestClient(app)
            response = client.post(
                "/process",
                json={
                    "user_text": "The actuator needs calibration",
                    "target_language": "es",
                    "use_rag": True,
                },
            )

            assert response.status_code == 200
            data = response.json()

            # Clarification uses RAG, translation follows
            assert "rag_context" in data
            assert "citations" in data
            # Translation should preserve RAG citations

    @pytest.mark.asyncio
    async def test_rag_error_handling(self, mock_rag_client):
        """Test graceful degradation when RAG fails."""
        # Mock RAG client that raises exception
        mock_rag_client.retrieve_context = AsyncMock(
            side_effect=Exception("Qdrant connection failed")
        )

        with patch("src.text_processor.rag_client.RAGClient", return_value=mock_rag_client):
            client = TestClient(app)
            response = client.post(
                "/process",
                json={
                    "user_text": "some text",
                    "target_language": "en",
                    "use_rag": True,
                },
            )

            # Should still succeed with degraded RAG
            assert response.status_code == 200
            data = response.json()

            # RAG context should indicate failure gracefully
            # (clarification still works, just without RAG data)
            assert "rag_context" in data

    @pytest.mark.asyncio
    async def test_validation_before_rag(self, mock_rag_client):
        """Test that input validation happens before RAG retrieval."""
        client = TestClient(app)
        response = client.post(
            "/process",
            json={
                "user_text": "",  # Empty input
                "target_language": "en",
                "use_rag": True,
            },
        )

        # Validation should fail before RAG is called
        assert response.status_code == 400

    @pytest.mark.asyncio
    async def test_rag_max_documents_retrieved(self, mock_rag_client):
        """Test RAG respects top_k configuration."""
        # Create mock with multiple documents
        many_docs_context = RAGContext(
            query="broad query",
            documents=[
                Citation(
                    source_index=i,
                    title=f"Document {i}",
                    source_url=f"https://example.com/doc{i}",
                    file_path=f"/docs/doc{i}.md",
                    snippet=f"Content {i}",
                    relevance_score=0.9 - (i * 0.05),
                )
                for i in range(10)  # More than top_k
            ],
            retrieval_time_ms=200,
            formatted_context="Multiple documents...",
        )

        mock_rag_client.retrieve_context = AsyncMock(return_value=many_docs_context)

        with patch("src.text_processor.rag_client.RAGClient", return_value=mock_rag_client):
            client = TestClient(app)
            response = client.post(
                "/process",
                json={
                    "user_text": "query that matches many docs",
                    "target_language": "en",
                    "use_rag": True,
                },
            )

            # Should respect top_k (default 5)
            data = response.json()
            assert len(data["rag_context"]["documents"]) <= 5  # top_k default

    @pytest.mark.asyncio
    async def test_rag_similarity_threshold_filtering(self, mock_rag_client):
        """Test RAG filters documents below similarity threshold."""
        # Mix of high and low scoring documents
        mixed_context = RAGContext(
            query="specific query",
            documents=[
                Citation(
                    source_index=0,
                    title="High relevance doc",
                    source_url="https://example.com/high",
                    file_path="/docs/high.md",
                    snippet="High quality content",
                    relevance_score=0.9,
                ),
                Citation(
                    source_index=1,
                    title="Medium relevance doc",
                    source_url="https://example.com/medium",
                    file_path="/docs/medium.md",
                    snippet="Medium quality content",
                    relevance_score=0.6,
                ),
                Citation(
                    source_index=2,
                    title="Below threshold doc",
                    source_url="https://example.com/low",
                    file_path="/docs/low.md",
                    snippet="Low quality content",
                    relevance_score=0.3,  # Below default threshold 0.4
                ),
            ],
            retrieval_time_ms=150,
            formatted_context="Filtered documents...",
        )

        mock_rag_client.retrieve_context = AsyncMock(return_value=mixed_context)

        with patch("src.text_processor.rag_client.RAGClient", return_value=mock_rag_client):
            client = TestClient(app)
            response = client.post(
                "/process",
                json={
                    "user_text": "specific technical query",
                    "target_language": "en",
                    "use_rag": True,
                },
            )

            # Low-relevance document should be filtered out
            data = response.json()
            retrieved_titles = [doc["title"] for doc in data["rag_context"]["documents"]]
            assert "High relevance doc" in retrieved_titles
            assert "Medium relevance doc" in retrieved_titles
            assert "Below threshold doc" not in retrieved_titles
