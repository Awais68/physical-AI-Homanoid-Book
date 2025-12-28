"""API routes for text processing service."""

import logging

from fastapi import APIRouter, HTTPException, Request

from src.text_processor.clarifier import TextClarifier
from src.text_processor.language_config import (
    SUPPORTED_LANGUAGE_CODES,
    get_all_languages,
    normalize_language_code,
)
from src.text_processor.models import (
    ErrorResponse,
    ErrorType,
    HealthResponse,
    LanguageListResponse,
    TextProcessRequest,
    TextProcessResponse,
)
from src.text_processor.translator import TextTranslator

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post(
    "/process",
    response_model=TextProcessResponse,
    responses={
        400: {"model": ErrorResponse, "description": "Validation error"},
        500: {"model": ErrorResponse, "description": "Processing error"},
        503: {"model": ErrorResponse, "description": "Service unavailable"},
    },
    summary="Process text for clarification and translation",
    description=(
        "Accepts text input and optional target language. Clarifies the input text "
        "by removing redundancy, fixing grammar, and improving clarity. Then translates "
        "to the specified target language (defaults to English)."
    ),
    tags=["Text Processing"],
)
async def process_text(
    request: Request,
    body: TextProcessRequest,
) -> TextProcessResponse:
    """Process text for clarification and/or translation.

    Args:
        request: FastAPI request object
        body: Request body with user_text and target_language

    Returns:
        TextProcessResponse with original and processed text

    Raises:
        HTTPException: On validation or processing errors
    """
    request_id = getattr(request.state, "request_id", "unknown")
    logger.info(
        f"Processing text request: length={len(body.user_text)}, "
        f"target_language={body.target_language}",
        extra={"request_id": request_id},
    )

    # Validate target language early
    normalized_language = normalize_language_code(body.target_language)
    if normalized_language is None:
        logger.warning(
            f"Unsupported language: {body.target_language}",
            extra={"request_id": request_id},
        )
        raise HTTPException(
            status_code=400,
            detail={
                "error_type": ErrorType.UNSUPPORTED_LANGUAGE.value,
                "message": f"Language '{body.target_language}' is not supported",
                "details": {"supported_languages": SUPPORTED_LANGUAGE_CODES},
            },
        )

    try:
        # Step 1: Clarify the text
        clarifier = TextClarifier()
        clarified_text = await clarifier.clarify(body.user_text)

        # Step 2: Translate if target language is not English
        if normalized_language != "en":
            translator = TextTranslator()
            translated_text = await translator.translate(
                clarified_text, normalized_language
            )
        else:
            # For English, clarification is the final output
            translated_text = clarified_text

        return TextProcessResponse(
            original_text=body.user_text,
            translated_text=translated_text,
        )

    except ValueError as e:
        # Validation errors
        logger.warning(
            f"Validation error: {str(e)}",
            extra={"request_id": request_id},
        )
        raise HTTPException(
            status_code=400,
            detail={
                "error_type": ErrorType.VALIDATION_ERROR.value,
                "message": str(e),
                "details": None,
            },
        )

    except Exception as e:
        # LLM or other processing errors
        logger.error(
            f"Processing error: {type(e).__name__}: {str(e)}",
            extra={"request_id": request_id},
        )
        raise HTTPException(
            status_code=500,
            detail={
                "error_type": ErrorType.PROCESSING_ERROR.value,
                "message": "Failed to process text. Please try again.",
                "details": {"request_id": request_id},
            },
        )


@router.get(
    "/languages",
    response_model=LanguageListResponse,
    summary="Get list of supported languages",
    description="Returns all languages supported for translation.",
    tags=["Configuration"],
)
async def get_supported_languages() -> LanguageListResponse:
    """Get list of supported languages.

    Returns:
        LanguageListResponse with all supported languages
    """
    languages = get_all_languages()
    return LanguageListResponse(languages=languages)


@router.get(
    "/health",
    response_model=HealthResponse,
    summary="Health check endpoint",
    description="Returns service health status.",
    tags=["Health"],
)
async def health_check() -> HealthResponse:
    """Check service health status.

    Returns:
        HealthResponse with status and version
    """
    from src.config import get_settings, is_openai_configured
    from src.text_processor.models import HealthStatus

    settings = get_settings()

    # Check if OpenAI is configured
    if is_openai_configured():
        status = HealthStatus.HEALTHY
    else:
        status = HealthStatus.DEGRADED

    return HealthResponse(
        status=status,
        version=settings.app_version,
    )
