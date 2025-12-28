"""Integration tests for text translation flow."""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from httpx import AsyncClient


class TestTranslationFlow:
    """Integration tests for the complete translation workflow."""

    @pytest.mark.asyncio
    async def test_full_translation_flow(
        self,
        async_client: AsyncClient,
        verbose_text: str,
        spanish_translation: str,
    ) -> None:
        """Test complete flow: request → clarification → translation → response."""
        request_data = {
            "user_text": verbose_text,
            "target_language": "es",
        }

        mock_clarify_response = MagicMock()
        mock_clarify_response.choices = [
            MagicMock(message=MagicMock(content="This is very important."))
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
        assert data["original_text"] == verbose_text

    @pytest.mark.asyncio
    async def test_translation_to_multiple_languages(
        self,
        async_client: AsyncClient,
    ) -> None:
        """Test translation to different target languages."""
        test_text = "Hello, how are you?"
        translations = {
            "es": "Hola, como estas?",
            "fr": "Bonjour, comment allez-vous?",
            "de": "Hallo, wie geht es Ihnen?",
        }

        for lang, expected_translation in translations.items():
            request_data = {
                "user_text": test_text,
                "target_language": lang,
            }

            mock_response = MagicMock()
            mock_response.choices = [
                MagicMock(message=MagicMock(content=expected_translation))
            ]

            with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
                with patch(
                    "src.text_processor.translator.AsyncOpenAI"
                ) as mock_translator:
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

            assert response.status_code == 200

    @pytest.mark.asyncio
    async def test_translation_preserves_formatting(
        self,
        async_client: AsyncClient,
    ) -> None:
        """Test that basic formatting is preserved during translation."""
        text_with_format = """First paragraph.

Second paragraph with multiple lines.

Third paragraph."""

        translated_text = """Primer parrafo.

Segundo parrafo con multiples lineas.

Tercer parrafo."""

        request_data = {
            "user_text": text_with_format,
            "target_language": "es",
        }

        mock_response = MagicMock()
        mock_response.choices = [
            MagicMock(message=MagicMock(content=translated_text))
        ]

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

        assert response.status_code == 200


class TestLanguagesEndpoint:
    """Integration tests for /languages endpoint."""

    @pytest.mark.asyncio
    async def test_get_languages_returns_all_supported(
        self,
        async_client: AsyncClient,
    ) -> None:
        """Test that /languages returns all 12 supported languages."""
        response = await async_client.get("/languages")

        assert response.status_code == 200
        data = response.json()
        assert "languages" in data
        assert len(data["languages"]) == 12

    @pytest.mark.asyncio
    async def test_languages_have_required_fields(
        self,
        async_client: AsyncClient,
    ) -> None:
        """Test that each language has code, name, and native_name."""
        response = await async_client.get("/languages")

        data = response.json()
        for language in data["languages"]:
            assert "code" in language
            assert "name" in language
            assert "native_name" in language

    @pytest.mark.asyncio
    async def test_languages_includes_english(
        self,
        async_client: AsyncClient,
    ) -> None:
        """Test that English is in the supported languages."""
        response = await async_client.get("/languages")

        data = response.json()
        codes = [lang["code"] for lang in data["languages"]]
        assert "en" in codes
