"""Integration tests for text clarification flow."""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from httpx import AsyncClient


class TestClarificationFlow:
    """Integration tests for the complete clarification workflow."""

    @pytest.mark.asyncio
    async def test_full_clarification_flow(
        self,
        async_client: AsyncClient,
        verbose_text: str,
        clear_text: str,
    ) -> None:
        """Test complete flow: request → clarification → response."""
        request_data = {
            "user_text": verbose_text,
            "target_language": "en",
        }

        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content=clear_text))]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_openai:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(return_value=mock_response)
            mock_openai.return_value = mock_client

            response = await async_client.post("/process", json=request_data)

        # Verify successful response
        assert response.status_code == 200

        # Verify response structure
        data = response.json()
        assert data["original_text"] == verbose_text
        assert data["translated_text"] == clear_text

    @pytest.mark.asyncio
    async def test_clarification_removes_redundancy(
        self,
        async_client: AsyncClient,
    ) -> None:
        """Test that clarification removes redundant phrases."""
        redundant_text = (
            "I would like to inform you that I am writing to let you know "
            "that the meeting has been scheduled for tomorrow."
        )
        concise_text = "The meeting is scheduled for tomorrow."

        request_data = {
            "user_text": redundant_text,
            "target_language": "en",
        }

        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content=concise_text))]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_openai:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(return_value=mock_response)
            mock_openai.return_value = mock_client

            response = await async_client.post("/process", json=request_data)

        data = response.json()
        # Clarified text should be shorter
        assert len(data["translated_text"]) < len(redundant_text)

    @pytest.mark.asyncio
    async def test_clarification_with_grammar_errors(
        self,
        async_client: AsyncClient,
    ) -> None:
        """Test that clarification fixes grammar errors."""
        text_with_errors = "He dont know nothing about the situation."
        corrected_text = "He doesn't know anything about the situation."

        request_data = {
            "user_text": text_with_errors,
            "target_language": "en",
        }

        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content=corrected_text))]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_openai:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(return_value=mock_response)
            mock_openai.return_value = mock_client

            response = await async_client.post("/process", json=request_data)

        assert response.status_code == 200
        data = response.json()
        assert data["translated_text"] == corrected_text

    @pytest.mark.asyncio
    async def test_response_includes_request_id_header(
        self,
        async_client: AsyncClient,
        sample_request_data: dict[str, str],
        clear_text: str,
    ) -> None:
        """Test that response includes X-Request-ID header."""
        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content=clear_text))]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_openai:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(return_value=mock_response)
            mock_openai.return_value = mock_client

            response = await async_client.post("/process", json=sample_request_data)

        assert "x-request-id" in response.headers


class TestClarificationErrorHandling:
    """Integration tests for error handling in clarification flow."""

    @pytest.mark.asyncio
    async def test_validation_error_for_empty_text(
        self,
        async_client: AsyncClient,
    ) -> None:
        """Test validation error response for empty text."""
        request_data = {
            "user_text": "",
            "target_language": "en",
        }

        response = await async_client.post("/process", json=request_data)

        # FastAPI returns 422 for Pydantic validation errors
        assert response.status_code == 422
        data = response.json()
        assert "detail" in data

    @pytest.mark.asyncio
    async def test_llm_failure_returns_processing_error(
        self,
        async_client: AsyncClient,
        sample_request_data: dict[str, str],
    ) -> None:
        """Test that LLM failures return PROCESSING_ERROR."""
        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_openai:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(
                side_effect=Exception("LLM API error")
            )
            mock_openai.return_value = mock_client

            response = await async_client.post("/process", json=sample_request_data)

        assert response.status_code == 500
        data = response.json()
        # FastAPI HTTPException returns error in 'detail' field
        assert "detail" in data
        assert data["detail"]["error_type"] == "PROCESSING_ERROR"
