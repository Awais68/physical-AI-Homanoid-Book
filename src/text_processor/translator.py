"""Text translation service using OpenAI LLM."""

import logging

from openai import AsyncOpenAI

from src.config import get_settings
from src.text_processor.language_config import (
    SUPPORTED_LANGUAGE_CODES,
    get_language,
    normalize_language_code,
)

logger = logging.getLogger(__name__)

# Translation prompt template
TRANSLATION_PROMPT = """Translate the following text to {language_name}.

Guidelines:
- Use natural, idiomatic phrasing appropriate for the target language
- Preserve the original meaning exactly
- Do not add or remove information
- Maintain the tone and style of the original
- Keep formatting (paragraphs, lists) intact

Text to translate:
{text}

Provide only the translated text, without any explanations or preamble."""


class TextTranslator:
    """Service for translating text using OpenAI LLM."""

    def __init__(self) -> None:
        """Initialize the text translator with OpenAI client."""
        settings = get_settings()
        self._client = AsyncOpenAI(api_key=settings.openai_api_key)
        self._model = settings.openai_model
        self._timeout = settings.openai_timeout

    def _validate_language(self, target_language: str) -> str:
        """Validate and normalize the target language.

        Args:
            target_language: Language code or name

        Returns:
            Normalized language code

        Raises:
            ValueError: If language is not supported
        """
        normalized = normalize_language_code(target_language)
        if normalized is None:
            raise ValueError(
                f"Language '{target_language}' is not supported",
            )
        return normalized

    def _get_language_name(self, code: str) -> str:
        """Get the full language name for a code.

        Args:
            code: ISO 639-1 language code

        Returns:
            Full language name
        """
        language = get_language(code)
        return language.name if language else code

    async def translate(self, text: str, target_language: str) -> str:
        """Translate the given text to the target language.

        Args:
            text: The text to translate
            target_language: Target language code or name

        Returns:
            Translated text

        Raises:
            ValueError: If language validation fails or LLM returns empty response
            Exception: If LLM API call fails
        """
        # Validate and normalize language
        language_code = self._validate_language(target_language)
        language_name = self._get_language_name(language_code)

        logger.info(
            f"Translating text of length {len(text)} to {language_name} ({language_code})"
        )

        try:
            # Build the prompt
            prompt = TRANSLATION_PROMPT.format(
                language_name=language_name,
                text=text,
            )

            # Call OpenAI API
            response = await self._client.chat.completions.create(
                model=self._model,
                messages=[
                    {
                        "role": "system",
                        "content": f"You are a professional translator specializing in {language_name}. Provide natural, idiomatic translations.",
                    },
                    {"role": "user", "content": prompt},
                ],
                temperature=0.3,  # Lower temperature for consistent translations
                max_tokens=len(text) * 2,  # Allow expansion for some languages
                timeout=self._timeout,
            )

            # Extract result
            result = response.choices[0].message.content

            if not result or not result.strip():
                raise ValueError("Empty response from LLM during translation")

            translated_text = result.strip()

            logger.info(
                f"Translation complete: {len(text)} -> {len(translated_text)} chars"
            )

            return translated_text

        except ValueError:
            # Re-raise validation errors
            raise
        except Exception as e:
            logger.error(f"LLM translation failed: {type(e).__name__}: {str(e)}")
            raise

    def get_supported_languages(self) -> list[str]:
        """Get list of supported language codes.

        Returns:
            List of ISO 639-1 language codes
        """
        return SUPPORTED_LANGUAGE_CODES.copy()


async def translate_text(text: str, target_language: str) -> str:
    """Convenience function to translate text.

    Args:
        text: The text to translate
        target_language: Target language code or name

    Returns:
        Translated text
    """
    translator = TextTranslator()
    return await translator.translate(text, target_language)
