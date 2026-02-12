"""Text clarification service using OpenAI LLM."""

import logging
from typing import Any

from openai import AsyncOpenAI

from src.config import get_settings
from src.text_processor.models import RAGContext

logger = logging.getLogger(__name__)

# Clarification prompt template (without RAG)
CLARIFICATION_PROMPT = """Analyze the following text and provide a clearer, more concise version.

Guidelines:
- Fix grammatical errors
- Remove redundancy and unnecessary phrases
- Preserve original meaning exactly
- Do not add new information
- Maintain tone (formal/informal)
- Keep response in the same language as input

Text to clarify:
{text}

Provide only the clarified text, without any explanations or preamble."""

# Clarification prompt template (with RAG)
CLARIFICATION_PROMPT_WITH_RAG = """Analyze the following text and provide a clearer, more concise version.

Use the following Physical AI documentation to inform your clarification:

{rag_context}

Guidelines:
- Fix grammatical errors
- Remove redundancy and unnecessary phrases
- Preserve original meaning exactly
- Use knowledge from documentation only if it helps clarify technical terms
- Do not add new information not present in the text
- Maintain tone (formal/informal)
- Keep response in the same language as input

Text to clarify:
{text}

Provide only the clarified text, without any explanations or preamble."""


class TextClarifier:
    """Service for clarifying text using OpenAI LLM."""

    def __init__(self) -> None:
        """Initialize the text clarifier with OpenAI client."""
        settings = get_settings()
        self._client = AsyncOpenAI(api_key=settings.openai_api_key)
        self._model = settings.openai_model
        self._timeout = settings.openai_timeout
        self._max_input_length = settings.max_input_length

    def _validate_input(self, text: str) -> None:
        """Validate input text before processing.

        Args:
            text: Text to validate

        Raises:
            ValueError: If text is invalid
        """
        if not text:
            raise ValueError("Text input cannot be empty")

        if not text.strip():
            raise ValueError("Text input cannot be whitespace only")

        if len(text) > self._max_input_length:
            raise ValueError(
                f"Text exceeds maximum length of {self._max_input_length} characters"
            )

    async def clarify(self, text: str) -> str:
        """Clarify the given text to make it clearer and more concise.

        Args:
            text: The text to clarify

        Returns:
            Clarified version of the text

        Raises:
            ValueError: If input validation fails or LLM returns empty response
            Exception: If LLM API call fails
        """
        # Validate input
        self._validate_input(text)

        logger.info(f"Clarifying text of length {len(text)}")

        try:
            # Build prompt
            prompt = CLARIFICATION_PROMPT.format(text=text)

            # Call OpenAI API
            response = await self._client.chat.completions.create(
                model=self._model,
                messages=[
                    {
                        "role": "system",
                        "content": "You are a professional text editor specializing in clarity and conciseness.",
                    },
                    {"role": "user", "content": prompt},
                ],
                temperature=0.3,  # Lower temperature for more consistent output
                max_tokens=len(text) + 100,  # Allow some buffer
                timeout=self._timeout,
            )

            # Extract result
            result = response.choices[0].message.content

            if not result or not result.strip():
                raise ValueError("Empty response from LLM")

            clarified_text = result.strip()

            logger.info(
                f"Clarification complete: {len(text)} -> {len(clarified_text)} chars"
            )

            return clarified_text

        except ValueError:
            # Re-raise validation errors
            raise
        except Exception as e:
            logger.error(f"LLM clarification failed: {type(e).__name__}: {str(e)}")
            raise

    async def clarify_with_rag(
        self, text: str, rag_context: RAGContext
    ) -> str:
        """Clarify text with RAG context from Qdrant.

        Args:
            text: The text to clarify
            rag_context: Retrieved context from Qdrant vector database

        Returns:
            Clarified version of the text

        Raises:
            ValueError: If input validation fails or LLM returns empty response
            Exception: If LLM API call fails
        """
        # Validate input
        self._validate_input(text)

        logger.info(
            f"Clarifying text with RAG: {len(text)} chars, "
            f"{len(rag_context.documents)} docs"
        )

        try:
            # Build prompt with RAG context
            prompt = CLARIFICATION_PROMPT_WITH_RAG.format(
                text=text, rag_context=rag_context.formatted_context
            )

            # Call OpenAI API
            response = await self._client.chat.completions.create(
                model=self._model,
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are a professional text editor specializing in clarity and conciseness. "
                            "Use provided Physical AI documentation to help clarify technical terms."
                        ),
                    },
                    {"role": "user", "content": prompt},
                ],
                temperature=0.3,
                max_tokens=len(text) + 100 + len(rag_context.formatted_context),
                timeout=self._timeout,
            )

            # Extract result
            result = response.choices[0].message.content

            if not result or not result.strip():
                raise ValueError("Empty response from LLM")

            clarified_text = result.strip()

            logger.info(
                f"RAG clarification complete: {len(text)} -> {len(clarified_text)} chars"
            )

            return clarified_text

        except ValueError:
            # Re-raise validation errors
            raise
        except Exception as e:
            logger.error(f"LLM RAG clarification failed: {type(e).__name__}: {str(e)}")
            raise


async def clarify_text(text: str) -> str:
    """Convenience function to clarify text.

    Args:
        text: The text to clarify

    Returns:
        Clarified version of the text
    """
    clarifier = TextClarifier()
    return await clarifier.clarify(text)
