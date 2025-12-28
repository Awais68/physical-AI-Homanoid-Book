"""Unit tests for combined text processor (clarify + translate)."""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from src.text_processor import process_text, TextProcessor


class TestTextProcessor:
    """Tests for the combined TextProcessor class."""

    @pytest.fixture
    def processor(self) -> TextProcessor:
        """Create TextProcessor instance."""
        return TextProcessor()

    @pytest.mark.asyncio
    async def test_process_clarifies_then_translates(
        self,
        verbose_text: str,
        clear_text: str,
        spanish_translation: str,
    ) -> None:
        """Test that text is first clarified, then translated."""
        mock_clarify_response = MagicMock()
        mock_clarify_response.choices = [
            MagicMock(message=MagicMock(content=clear_text))
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

                # Create processor inside the patch context
                processor = TextProcessor()
                result = await processor.process(verbose_text, "es")

        assert result.original_text == verbose_text
        assert result.translated_text == spanish_translation

    @pytest.mark.asyncio
    async def test_process_returns_json_compliant_response(
        self,
        processor: TextProcessor,
        verbose_text: str,
        clear_text: str,
    ) -> None:
        """Test that response is JSON schema compliant."""
        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content=clear_text))]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(return_value=mock_response)
            mock_clarifier.return_value = mock_client

            result = await processor.process(verbose_text, "en")

        # Verify JSON structure
        json_result = result.model_dump()
        assert "original_text" in json_result
        assert "translated_text" in json_result
        assert isinstance(json_result["original_text"], str)
        assert isinstance(json_result["translated_text"], str)

    @pytest.mark.asyncio
    async def test_process_english_only_clarifies(
        self,
        verbose_text: str,
        clear_text: str,
    ) -> None:
        """Test that English target only clarifies, doesn't translate."""
        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content=clear_text))]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
            with patch(
                "src.text_processor.translator.AsyncOpenAI"
            ) as mock_translator:
                mock_client = MagicMock()
                mock_client.chat.completions.create = AsyncMock(
                    return_value=mock_response
                )
                mock_clarifier.return_value = mock_client

                # Create processor inside the patch context
                processor = TextProcessor()
                # Translator should not be called for English
                result = await processor.process(verbose_text, "en")

        assert result.translated_text == clear_text


class TestProcessTextFunction:
    """Tests for the module-level process_text function."""

    @pytest.mark.asyncio
    async def test_process_text_convenience_function(
        self, verbose_text: str, clear_text: str, spanish_translation: str
    ) -> None:
        """Test the convenience function for processing text."""
        mock_clarify_response = MagicMock()
        mock_clarify_response.choices = [
            MagicMock(message=MagicMock(content=clear_text))
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

                result = await process_text(verbose_text, "es")

        assert result.original_text == verbose_text
        assert result.translated_text == spanish_translation


class TestErrorHandling:
    """Tests for error handling in processor."""

    @pytest.mark.asyncio
    async def test_clarifier_error_propagates(
        self, verbose_text: str
    ) -> None:
        """Test that clarifier errors are properly propagated."""
        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(
                side_effect=Exception("LLM API error")
            )
            mock_clarifier.return_value = mock_client

            # Create processor inside the patch context
            processor = TextProcessor()
            with pytest.raises(Exception, match="LLM API error"):
                await processor.process(verbose_text, "en")

    @pytest.mark.asyncio
    async def test_translator_error_propagates(
        self, verbose_text: str, clear_text: str
    ) -> None:
        """Test that translator errors are properly propagated."""
        mock_clarify_response = MagicMock()
        mock_clarify_response.choices = [
            MagicMock(message=MagicMock(content=clear_text))
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
                    side_effect=Exception("Translation API error")
                )
                mock_translator.return_value = mock_translator_client

                # Create processor inside the patch context
                processor = TextProcessor()
                with pytest.raises(Exception, match="Translation API error"):
                    await processor.process(verbose_text, "es")
