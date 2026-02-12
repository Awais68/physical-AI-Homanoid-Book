"""Contract tests for combined /process endpoint (clarify + translate)."""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from httpx import AsyncClient


class TestProcessCombinedContract:
    """Contract tests for POST /process endpoint with combined workflow."""

    @pytest.mark.asyncio
    async def test_combined_response_has_both_fields(
        self,
        async_client: AsyncClient,
        verbose_text: str,
        spanish_translation: str,
    ) -> None:
        """Test that combined response has original_text and translated_text."""
        request_data = {
            "user_text": verbose_text,
            "target_language": "es",
        }

        mock_clarify_response = MagicMock()
        mock_clarify_response.choices = [
            MagicMock(message=MagicMock(content="This is important."))
        ]

        mock_translate_response = MagicMock()
        mock_translate_response.choices = [
            MagicMock(message=MagicMock(content=spanish_translation))
        ]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
            with patch("src.text_processor.translator.AsyncOpenAI") as mock_translator:
                mock_clarifier_client = MagicMock()
                mock_clarifier_client.chat.completions.create = AsyncMock(
                    return_value=mock_clarify_response
                )
                mock_clarifier.return_value = mock_clarifier_client

                mock_translator_client = MagicMock()
                mock_translator_client.chat.completions.create = AsyncMock(
                    return_value=mock_translate_response
                )
                mock_translator.return_value = mock_translator_client

                response = await async_client.post("/process", json=request_data)

        assert response.status_code == 200
        data = response.json()
        assert "original_text" in data
        assert "translated_text" in data

    @pytest.mark.asyncio
    async def test_original_text_unchanged(
        self,
        async_client: AsyncClient,
        verbose_text: str,
    ) -> None:
        """Test that original_text is exactly the input user_text."""
        request_data = {
            "user_text": verbose_text,
            "target_language": "fr",
        }

        mock_response = MagicMock()
        mock_response.choices = [
            MagicMock(message=MagicMock(content="Processed text"))
        ]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
            with patch("src.text_processor.translator.AsyncOpenAI") as mock_translator:
                mock_client = MagicMock()
                mock_client.chat.completions.create = AsyncMock(
                    return_value=mock_response
                )
                mock_clarifier.return_value = mock_client
                mock_translator.return_value = mock_client

                response = await async_client.post("/process", json=request_data)

        data = response.json()
        assert data["original_text"] == verbose_text

    @pytest.mark.asyncio
    async def test_response_is_valid_json(
        self,
        async_client: AsyncClient,
        sample_translation_request: dict[str, str],
    ) -> None:
        """Test that response is valid parseable JSON."""
        mock_response = MagicMock()
        mock_response.choices = [
            MagicMock(message=MagicMock(content="Texto procesado"))
        ]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
            with patch("src.text_processor.translator.AsyncOpenAI") as mock_translator:
                mock_client = MagicMock()
                mock_client.chat.completions.create = AsyncMock(
                    return_value=mock_response
                )
                mock_clarifier.return_value = mock_client
                mock_translator.return_value = mock_client

                response = await async_client.post(
                    "/process", json=sample_translation_request
                )

        # Should not raise JSON decode error
        data = response.json()
        assert isinstance(data, dict)


class TestProcessResponseHeaders:
    """Contract tests for response headers."""

    @pytest.mark.asyncio
    async def test_response_has_request_id_header(
        self,
        async_client: AsyncClient,
        sample_request_data: dict[str, str],
    ) -> None:
        """Test that response includes X-Request-ID header."""
        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content="Clarified"))]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(return_value=mock_response)
            mock_clarifier.return_value = mock_client

            response = await async_client.post("/process", json=sample_request_data)

        assert "x-request-id" in response.headers

    @pytest.mark.asyncio
    async def test_response_has_process_time_header(
        self,
        async_client: AsyncClient,
        sample_request_data: dict[str, str],
    ) -> None:
        """Test that response includes X-Process-Time-Ms header."""
        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content="Clarified"))]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(return_value=mock_response)
            mock_clarifier.return_value = mock_client

            response = await async_client.post("/process", json=sample_request_data)

        assert "x-process-time-ms" in response.headers


class TestCombinedErrorContract:
    """Contract tests for error responses in combined workflow."""

    @pytest.mark.asyncio
    async def test_processing_error_format(
        self,
        async_client: AsyncClient,
        sample_request_data: dict[str, str],
    ) -> None:
        """Test that processing errors have correct format."""
        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(
                side_effect=Exception("API failure")
            )
            mock_clarifier.return_value = mock_client

            response = await async_client.post("/process", json=sample_request_data)

        assert response.status_code == 500
        data = response.json()
        assert "detail" in data
        assert data["detail"]["error_type"] == "PROCESSING_ERROR"
        assert "message" in data["detail"]
