"""Text Processor Library - Core text processing logic for clarification and translation.

This module provides the main orchestration for combining text clarification
and translation into a unified workflow.
"""

import logging
import time
import uuid
from typing import Any

from src.config import get_settings
from src.text_processor.clarifier import TextClarifier
from src.text_processor.language_config import normalize_language_code
from src.text_processor.models import ProcessingMetrics, TextProcessResponse
from src.text_processor.translator import TextTranslator

logger = logging.getLogger(__name__)


class TextProcessor:
    """Orchestrates the combined text clarification and translation workflow."""

    def __init__(self) -> None:
        """Initialize the text processor with clarifier and translator."""
        self._clarifier = TextClarifier()
        self._translator = TextTranslator()
        self._settings = get_settings()

    async def process(
        self,
        text: str,
        target_language: str = "en",
        request_id: str | None = None,
    ) -> TextProcessResponse:
        """Process text: clarify and optionally translate.

        Args:
            text: The text to process
            target_language: Target language code or name (defaults to "en")
            request_id: Optional request ID for tracking

        Returns:
            TextProcessResponse with original and processed text

        Raises:
            ValueError: If input validation fails
            Exception: If LLM processing fails
        """
        if request_id is None:
            request_id = str(uuid.uuid4())

        start_time = time.time()

        # Normalize language code
        normalized_language = normalize_language_code(target_language)
        if normalized_language is None:
            raise ValueError(f"Language '{target_language}' is not supported")

        logger.info(
            f"Processing text: length={len(text)}, target={normalized_language}",
            extra={"request_id": request_id},
        )

        try:
            # Step 1: Clarify the text
            clarified_text = await self._clarifier.clarify(text)

            # Step 2: Translate if target is not English
            if normalized_language != "en":
                translated_text = await self._translator.translate(
                    clarified_text, normalized_language
                )
            else:
                translated_text = clarified_text

            # Calculate processing time
            processing_time_ms = int((time.time() - start_time) * 1000)

            # Log metrics
            metrics = ProcessingMetrics(
                request_id=request_id,
                source_language_detected=None,  # Could be enhanced with language detection
                target_language=normalized_language,
                input_length=len(text),
                output_length=len(translated_text),
                processing_time_ms=processing_time_ms,
                llm_model_used=self._settings.openai_model,
            )

            logger.info(
                f"Processing complete: {metrics.input_length} -> {metrics.output_length} chars "
                f"in {metrics.processing_time_ms}ms",
                extra={"request_id": request_id, "metrics": metrics.model_dump()},
            )

            return TextProcessResponse(
                original_text=text,
                translated_text=translated_text,
            )

        except ValueError:
            # Re-raise validation errors
            raise
        except Exception as e:
            logger.error(
                f"Processing failed: {type(e).__name__}: {str(e)}",
                extra={"request_id": request_id},
            )
            raise


async def process_text(
    text: str,
    target_language: str = "en",
    request_id: str | None = None,
) -> TextProcessResponse:
    """Convenience function to process text.

    Args:
        text: The text to process
        target_language: Target language code or name
        request_id: Optional request ID for tracking

    Returns:
        TextProcessResponse with original and processed text
    """
    processor = TextProcessor()
    return await processor.process(text, target_language, request_id)


# Export public API
__all__ = [
    "TextProcessor",
    "process_text",
    "TextClarifier",
    "TextTranslator",
]
