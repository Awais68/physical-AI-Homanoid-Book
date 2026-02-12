"""Unit tests for text clarifier service."""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from src.text_processor.clarifier import TextClarifier, clarify_text


class TestTextClarifier:
    """Tests for TextClarifier class."""

    @pytest.fixture
    def clarifier(self) -> TextClarifier:
        """Create TextClarifier instance with mocked OpenAI client."""
        return TextClarifier()

    @pytest.mark.asyncio
    async def test_clarify_verbose_text_returns_concise(
        self, clarifier: TextClarifier, verbose_text: str, clear_text: str
    ) -> None:
        """Test that verbose input is clarified to concise output."""
        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content=clear_text))]

        with patch.object(
            clarifier._client.chat.completions,
            "create",
            new_callable=AsyncMock,
            return_value=mock_response,
        ):
            result = await clarifier.clarify(verbose_text)

        assert result == clear_text
        assert len(result) < len(verbose_text)

    @pytest.mark.asyncio
    async def test_clarify_fixes_grammar_errors(self, clarifier: TextClarifier) -> None:
        """Test that grammar errors are fixed in output."""
        text_with_errors = "He go to the store yesterday and buyed some food."
        corrected_text = "He went to the store yesterday and bought some food."

        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content=corrected_text))]

        with patch.object(
            clarifier._client.chat.completions,
            "create",
            new_callable=AsyncMock,
            return_value=mock_response,
        ):
            result = await clarifier.clarify(text_with_errors)

        assert result == corrected_text
        assert "go" not in result.lower() or "went" in result.lower()

    @pytest.mark.asyncio
    async def test_clarify_preserves_meaning(
        self, clarifier: TextClarifier, verbose_text: str
    ) -> None:
        """Test that original meaning is preserved after clarification."""
        # The clarified text should still convey importance
        preserved_meaning = "This is very important."

        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content=preserved_meaning))]

        with patch.object(
            clarifier._client.chat.completions,
            "create",
            new_callable=AsyncMock,
            return_value=mock_response,
        ):
            result = await clarifier.clarify(verbose_text)

        # Core meaning should be preserved
        assert "important" in result.lower()

    @pytest.mark.asyncio
    async def test_clarify_handles_empty_response(
        self, clarifier: TextClarifier, verbose_text: str
    ) -> None:
        """Test handling of empty LLM response."""
        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content=""))]

        with patch.object(
            clarifier._client.chat.completions,
            "create",
            new_callable=AsyncMock,
            return_value=mock_response,
        ):
            # Should raise or return original text
            with pytest.raises(ValueError, match="Empty response"):
                await clarifier.clarify(verbose_text)


class TestClarifyTextFunction:
    """Tests for the clarify_text convenience function."""

    @pytest.mark.asyncio
    async def test_clarify_text_function(self, verbose_text: str, clear_text: str) -> None:
        """Test the module-level clarify_text function."""
        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content=clear_text))]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_openai:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(return_value=mock_response)
            mock_openai.return_value = mock_client

            result = await clarify_text(verbose_text)

        assert result == clear_text


class TestInputValidation:
    """Tests for input validation in clarifier."""

    @pytest.fixture
    def clarifier(self) -> TextClarifier:
        """Create TextClarifier instance."""
        return TextClarifier()

    @pytest.mark.asyncio
    async def test_empty_text_raises_error(self, clarifier: TextClarifier) -> None:
        """Test that empty text raises validation error."""
        with pytest.raises(ValueError, match="cannot be empty"):
            await clarifier.clarify("")

    @pytest.mark.asyncio
    async def test_whitespace_only_raises_error(self, clarifier: TextClarifier) -> None:
        """Test that whitespace-only text raises validation error."""
        with pytest.raises(ValueError, match="cannot be whitespace"):
            await clarifier.clarify("   \n\t   ")

    @pytest.mark.asyncio
    async def test_too_long_text_raises_error(self, clarifier: TextClarifier) -> None:
        """Test that text exceeding max length raises validation error."""
        long_text = "x" * 5001
        with pytest.raises(ValueError, match="exceeds maximum"):
            await clarifier.clarify(long_text)
