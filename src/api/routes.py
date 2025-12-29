"""API routes for text processing service."""

import logging

from fastapi import APIRouter, HTTPException, Request

from openai import OpenAI

from src.config import get_settings, is_openai_configured
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
    HealthStatus,
    LanguageListResponse,
    TextProcessRequest,
    TextProcessResponse,
)
from src.text_processor.rag_client import RAGClient, format_citation_list
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
        "Accepts text input and optional target language. Clarifies "
        "input text by removing redundancy, fixing grammar, and improving clarity. "
        "Optionally uses RAG to enhance clarification with Physical AI documentation. "
        "Then translates to specified target language (defaults to English)."
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
        body: Request body with user_text, target_language, and use_rag

    Returns:
        TextProcessResponse with original, processed text, and optional RAG context

    Raises:
        HTTPException: On validation or processing errors
    """
    request_id = getattr(request.state, "request_id", "unknown")

    logger.info(
        f"Processing text request: length={len(body.user_text)}, "
        f"target_language={body.target_language}, use_rag={body.use_rag}",
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
        # Initialize RAG client for this request if enabled
        rag_client = None
        rag_context = None
        citations = []

        if body.use_rag:
            settings = get_settings()
            openai_client = OpenAI(api_key=settings.openai_api_key)
            rag_client = RAGClient(openai_client=openai_client)
            rag_context = await rag_client.retrieve_context(body.user_text)

            if rag_context and rag_context.documents:
                citations = rag_context.documents
                logger.info(
                    f"RAG retrieved {len(citations)} documents for request {request_id}"
                )
            else:
                logger.info(f"RAG retrieval returned no documents for request {request_id}")

        # Step 1: Clarify text (with or without RAG)
        clarifier = TextClarifier()
        if rag_context and rag_context.documents:
            # Use RAG-enhanced clarification
            clarified_text = await clarifier.clarify_with_rag(
                body.user_text, rag_context
            )
        else:
            # Standard clarification
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

        response = TextProcessResponse(
            original_text=body.user_text,
            translated_text=translated_text,
            rag_context=rag_context,
            citations=[],
        )

        # Add formatted citations if available
        if citations:
            response.citations = format_citation_list(citations)

        return response

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
    description="Returns service health status and configuration.",
    tags=["Health"],
)
async def health_check() -> HealthResponse:
    """Check service health status.

    Returns:
        HealthResponse with status and version
    """
    settings = get_settings()

    # Check OpenAI configuration
    if not is_openai_configured():
        return HealthResponse(
            status=HealthStatus.DEGRADED,
            version=settings.app_version,
        )

    # Check RAG configuration
    from src.config import is_rag_enabled
    rag_enabled = is_rag_enabled()

    if rag_enabled:
        return HealthResponse(
            status=HealthStatus.HEALTHY,
            version=settings.app_version,
        )
    else:
        # RAG optional, OpenAI configured = healthy
        return HealthResponse(
            status=HealthStatus.HEALTHY,
            version=settings.app_version,
        )
