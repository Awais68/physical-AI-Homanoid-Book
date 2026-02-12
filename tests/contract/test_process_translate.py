"""Contract tests for /process endpoint (translation mode)."""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from httpx import AsyncClient


class TestProcessEndpointTranslateContract:
    """Contract tests for POST /process endpoint in translation mode."""

    @pytest.mark.asyncio
    async def test_process_returns_translated_text(
        self,
        async_client: AsyncClient,
        sample_translation_request: dict[str, str],
        spanish_translation: str,
    ) -> None:
        """Test that response contains translated text in target language."""
        mock_response = MagicMock()
        mock_response.choices = [
            MagicMock(message=MagicMock(content=spanish_translation))
        ]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
            with patch("src.text_processor.translator.AsyncOpenAI") as mock_translator:
                # Mock clarifier
                mock_clarifier_client = MagicMock()
                mock_clarifier_client.chat.completions.create = AsyncMock(
                    return_value=mock_response
                )
                mock_clarifier.return_value = mock_clarifier_client

                # Mock translator
                mock_translator_client = MagicMock()
                mock_translator_client.chat.completions.create = AsyncMock(
                    return_value=mock_response
                )
                mock_translator.return_value = mock_translator_client

                response = await async_client.post(
                    "/process", json=sample_translation_request
                )

        assert response.status_code == 200
        data = response.json()
        assert "translated_text" in data

    @pytest.mark.asyncio
    async def test_translation_to_all_supported_languages(
        self,
        async_client: AsyncClient,
        verbose_text: str,
    ) -> None:
        """Test that all supported languages can be specified."""
        supported_languages = [
            "en", "es", "fr", "de", "zh", "ja", "ar", "hi", "pt", "ru", "ko", "it"
        ]

        mock_response = MagicMock()
        mock_response.choices = [
            MagicMock(message=MagicMock(content="Translated text"))
        ]

        for lang in supported_languages:
            request_data = {
                "user_text": verbose_text,
                "target_language": lang,
            }

            with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
                with patch("src.text_processor.translator.AsyncOpenAI") as mock_translator:
                    mock_clarifier_client = MagicMock()
                    mock_clarifier_client.chat.completions.create = AsyncMock(
                        return_value=mock_response
                    )
                    mock_clarifier.return_value = mock_clarifier_client

                    mock_translator_client = MagicMock()
                    mock_translator_client.chat.completions.create = AsyncMock(
                        return_value=mock_response
                    )
                    mock_translator.return_value = mock_translator_client

                    response = await async_client.post("/process", json=request_data)

            assert response.status_code == 200, f"Failed for language: {lang}"


class TestUnsupportedLanguageContract:
    """Contract tests for unsupported language errors."""

    @pytest.mark.asyncio
    async def test_unsupported_language_returns_error(
        self,
        async_client: AsyncClient,
        unsupported_language_request: dict[str, str],
    ) -> None:
        """Test that unsupported language returns UNSUPPORTED_LANGUAGE error."""
        response = await async_client.post(
            "/process", json=unsupported_language_request
        )

        assert response.status_code == 400
        data = response.json()
        assert data["detail"]["error_type"] == "UNSUPPORTED_LANGUAGE"

    @pytest.mark.asyncio
    async def test_unsupported_language_includes_supported_list(
        self,
        async_client: AsyncClient,
        unsupported_language_request: dict[str, str],
    ) -> None:
        """Test that error includes list of supported languages."""
        response = await async_client.post(
            "/process", json=unsupported_language_request
        )

        data = response.json()
        assert "details" in data["detail"]
        assert "supported_languages" in data["detail"]["details"]
        assert len(data["detail"]["details"]["supported_languages"]) == 12
