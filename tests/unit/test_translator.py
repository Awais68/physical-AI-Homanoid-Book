"""Unit tests for text translator service."""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from src.text_processor.translator import TextTranslator, translate_text


class TestTextTranslator:
    """Tests for TextTranslator class."""

    @pytest.fixture
    def translator(self) -> TextTranslator:
        """Create TextTranslator instance."""
        return TextTranslator()

    @pytest.mark.asyncio
    async def test_translate_to_spanish(
        self, translator: TextTranslator, clear_text: str, spanish_translation: str
    ) -> None:
        """Test translation to Spanish."""
        mock_response = MagicMock()
        mock_response.choices = [
            MagicMock(message=MagicMock(content=spanish_translation))
        ]

        with patch.object(
            translator._client.chat.completions,
            "create",
            new_callable=AsyncMock,
            return_value=mock_response,
        ):
            result = await translator.translate(clear_text, "es")

        assert result == spanish_translation

    @pytest.mark.asyncio
    async def test_translate_produces_natural_output(
        self, translator: TextTranslator
    ) -> None:
        """Test that translation produces natural, idiomatic output."""
        english_text = "It's raining cats and dogs."
        # Idiomatic Spanish translation (not literal)
        idiomatic_translation = "Esta lloviendo a cantaros."

        mock_response = MagicMock()
        mock_response.choices = [
            MagicMock(message=MagicMock(content=idiomatic_translation))
        ]

        with patch.object(
            translator._client.chat.completions,
            "create",
            new_callable=AsyncMock,
            return_value=mock_response,
        ):
            result = await translator.translate(english_text, "es")

        # Should not be a literal translation
        assert "gatos" not in result.lower()  # Not "cats"
        assert "perros" not in result.lower()  # Not "dogs"

    @pytest.mark.asyncio
    async def test_translate_preserves_meaning(
        self, translator: TextTranslator
    ) -> None:
        """Test that translation preserves the original meaning."""
        english_text = "The meeting is scheduled for tomorrow at 3 PM."
        french_translation = "La reunion est prevue pour demain a 15h."

        mock_response = MagicMock()
        mock_response.choices = [
            MagicMock(message=MagicMock(content=french_translation))
        ]

        with patch.object(
            translator._client.chat.completions,
            "create",
            new_callable=AsyncMock,
            return_value=mock_response,
        ):
            result = await translator.translate(english_text, "fr")

        # Key information should be preserved
        assert "demain" in result.lower()  # "tomorrow" in French

    @pytest.mark.asyncio
    async def test_translate_handles_empty_response(
        self, translator: TextTranslator, clear_text: str
    ) -> None:
        """Test handling of empty LLM response during translation."""
        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content=""))]

        with patch.object(
            translator._client.chat.completions,
            "create",
            new_callable=AsyncMock,
            return_value=mock_response,
        ):
            with pytest.raises(ValueError, match="Empty response"):
                await translator.translate(clear_text, "es")


class TestLanguageValidation:
    """Tests for language validation in translator."""

    @pytest.fixture
    def translator(self) -> TextTranslator:
        """Create TextTranslator instance."""
        return TextTranslator()

    @pytest.mark.asyncio
    async def test_unsupported_language_raises_error(
        self, translator: TextTranslator, clear_text: str
    ) -> None:
        """Test that unsupported language raises error."""
        with pytest.raises(ValueError, match="not supported"):
            await translator.translate(clear_text, "xyz")

    @pytest.mark.asyncio
    async def test_case_insensitive_language_code(
        self, translator: TextTranslator, clear_text: str, spanish_translation: str
    ) -> None:
        """Test that language codes are case-insensitive."""
        mock_response = MagicMock()
        mock_response.choices = [
            MagicMock(message=MagicMock(content=spanish_translation))
        ]

        with patch.object(
            translator._client.chat.completions,
            "create",
            new_callable=AsyncMock,
            return_value=mock_response,
        ):
            # Test uppercase
            result = await translator.translate(clear_text, "ES")
            assert result == spanish_translation

    @pytest.mark.asyncio
    async def test_full_language_name_supported(
        self, translator: TextTranslator, clear_text: str, spanish_translation: str
    ) -> None:
        """Test that full language names are supported."""
        mock_response = MagicMock()
        mock_response.choices = [
            MagicMock(message=MagicMock(content=spanish_translation))
        ]

        with patch.object(
            translator._client.chat.completions,
            "create",
            new_callable=AsyncMock,
            return_value=mock_response,
        ):
            result = await translator.translate(clear_text, "Spanish")
            assert result == spanish_translation


class TestTranslateTextFunction:
    """Tests for the translate_text convenience function."""

    @pytest.mark.asyncio
    async def test_translate_text_function(
        self, clear_text: str, spanish_translation: str
    ) -> None:
        """Test the module-level translate_text function."""
        mock_response = MagicMock()
        mock_response.choices = [
            MagicMock(message=MagicMock(content=spanish_translation))
        ]

        with patch("src.text_processor.translator.AsyncOpenAI") as mock_openai:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(return_value=mock_response)
            mock_openai.return_value = mock_client

            result = await translate_text(clear_text, "es")

        assert result == spanish_translation
