"""Contract tests for /process endpoint (clarify-only mode)."""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from httpx import AsyncClient


class TestProcessEndpointClarifyContract:
    """Contract tests for POST /process endpoint in clarify-only mode."""

    @pytest.mark.asyncio
    async def test_process_returns_json_with_required_fields(
        self,
        async_client: AsyncClient,
        sample_request_data: dict[str, str],
        clear_text: str,
    ) -> None:
        """Test that response contains original_text and translated_text fields."""
        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content=clear_text))]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_openai:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(return_value=mock_response)
            mock_openai.return_value = mock_client

            response = await async_client.post("/process", json=sample_request_data)

        assert response.status_code == 200
        data = response.json()
        assert "original_text" in data
        assert "translated_text" in data

    @pytest.mark.asyncio
    async def test_process_original_text_matches_input(
        self,
        async_client: AsyncClient,
        sample_request_data: dict[str, str],
        verbose_text: str,
        clear_text: str,
    ) -> None:
        """Test that original_text matches the input user_text."""
        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content=clear_text))]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_openai:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(return_value=mock_response)
            mock_openai.return_value = mock_client

            response = await async_client.post("/process", json=sample_request_data)

        data = response.json()
        assert data["original_text"] == verbose_text

    @pytest.mark.asyncio
    async def test_process_translated_text_not_empty(
        self,
        async_client: AsyncClient,
        sample_request_data: dict[str, str],
        clear_text: str,
    ) -> None:
        """Test that translated_text is not empty for successful requests."""
        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content=clear_text))]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_openai:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(return_value=mock_response)
            mock_openai.return_value = mock_client

            response = await async_client.post("/process", json=sample_request_data)

        data = response.json()
        assert data["translated_text"]
        assert len(data["translated_text"]) > 0

    @pytest.mark.asyncio
    async def test_process_default_language_is_english(
        self,
        async_client: AsyncClient,
        verbose_text: str,
        clear_text: str,
    ) -> None:
        """Test that target_language defaults to 'en' when not provided."""
        request_data = {"user_text": verbose_text}  # No target_language

        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content=clear_text))]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_openai:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(return_value=mock_response)
            mock_openai.return_value = mock_client

            response = await async_client.post("/process", json=request_data)

        assert response.status_code == 200


class TestProcessEndpointErrorContract:
    """Contract tests for error responses from /process endpoint."""

    @pytest.mark.asyncio
    async def test_empty_text_returns_validation_error(
        self,
        async_client: AsyncClient,
        empty_text_request: dict[str, str],
    ) -> None:
        """Test that empty text returns validation error (422 from FastAPI/Pydantic)."""
        response = await async_client.post("/process", json=empty_text_request)

        # FastAPI returns 422 for Pydantic validation errors
        assert response.status_code == 422
        data = response.json()
        assert "detail" in data

    @pytest.mark.asyncio
    async def test_error_response_has_required_fields(
        self,
        async_client: AsyncClient,
        empty_text_request: dict[str, str],
    ) -> None:
        """Test that error response contains detail field (FastAPI format)."""
        response = await async_client.post("/process", json=empty_text_request)

        data = response.json()
        # FastAPI validation errors have 'detail' field
        assert "detail" in data

    @pytest.mark.asyncio
    async def test_too_long_text_returns_validation_error(
        self,
        async_client: AsyncClient,
        too_long_text_request: dict[str, str],
    ) -> None:
        """Test that text exceeding max length returns validation error."""
        response = await async_client.post("/process", json=too_long_text_request)

        # FastAPI returns 422 for Pydantic validation errors
        assert response.status_code == 422
        data = response.json()
        assert "detail" in data
