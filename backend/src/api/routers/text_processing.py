"""
Text Processing API Router
Provides endpoints for multilingual text processing with support for Urdu and Roman Urdu
"""

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel
from typing import Optional, Dict, Any
import logging

from src.text_processor.language_config import get_all_languages
from src.text_processor.models import TextProcessRequest, TextProcessResponse
from src.text_processor.translator import TextTranslator
from src.text_processor.clarifier import TextClarifier
from src.text_processor.rag_client import RAGClient
from src.config.translations import get_translation, is_rtl_language, get_supported_languages as get_translation_supported_languages

router = APIRouter(prefix="/text", tags=["text-processing"])

logger = logging.getLogger(__name__)

class MultilingualTextRequest(BaseModel):
    """Request model for multilingual text processing."""
    text: str
    target_language: str = "en"
    source_language: Optional[str] = None
    use_rag: bool = False
    operation: str = "translate"  # translate, clarify, both


class MultilingualTextResponse(BaseModel):
    """Response model for multilingual text processing."""
    original_text: str
    processed_text: str
    source_language: str
    target_language: str
    is_rtl: bool
    operation: str
    confidence: Optional[float] = None
    metadata: Optional[Dict[str, Any]] = None


@router.post("/process", response_model=MultilingualTextResponse)
async def process_multilingual_text(request: MultilingualTextRequest) -> MultilingualTextResponse:
    """
    Process text with multilingual support for Urdu and Roman Urdu.

    Args:
        request: Text processing request with language specifications

    Returns:
        Processed text with language metadata
    """
    try:
        # Validate language codes against supported languages
        supported_langs = get_translation_supported_languages()
        if request.target_language not in supported_langs:
            raise HTTPException(
                status_code=400,
                detail=f"Language '{request.target_language}' is not supported. Supported languages: {list(supported_langs.keys())}"
            )

        # Determine operation
        if request.operation not in ["translate", "clarify", "both"]:
            raise HTTPException(
                status_code=400,
                detail="Operation must be 'translate', 'clarify', or 'both'"
            )

        # Initialize components
        translator = TextTranslator()
        clarifier = TextClarifier()

        processed_text = request.text
        source_lang = request.source_language or "auto"  # In a real implementation, we'd detect the source language

        if request.operation in ["clarify", "both"]:
            processed_text = await clarifier.clarify(processed_text)

        if request.operation in ["translate", "both"] and request.target_language != "en":
            processed_text = await translator.translate(processed_text, request.target_language)

        # Determine if target language is right-to-left
        is_rtl = is_rtl_language(request.target_language)

        response = MultilingualTextResponse(
            original_text=request.text,
            processed_text=processed_text,
            source_language=source_lang,
            target_language=request.target_language,
            is_rtl=is_rtl,
            operation=request.operation,
            confidence=0.95,  # Placeholder confidence score
            metadata={
                "character_count_original": len(request.text),
                "character_count_processed": len(processed_text),
                "word_count_original": len(request.text.split()),
                "word_count_processed": len(processed_text.split()),
            }
        )

        logger.info(f"Processed text in {request.target_language}: {len(request.text)} -> {len(processed_text)} chars")

        return response

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Text processing failed: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Text processing failed: {str(e)}"
        )


@router.post("/translate", response_model=MultilingualTextResponse)
async def translate_text_endpoint(request: MultilingualTextRequest) -> MultilingualTextResponse:
    """
    Translate text to target language with special support for Urdu and Roman Urdu.

    Args:
        request: Translation request with text and target language

    Returns:
        Translated text with language metadata
    """
    try:
        # Override operation to translate
        request.operation = "translate"

        # Validate language codes against supported languages
        supported_langs = get_translation_supported_languages()
        if request.target_language not in supported_langs:
            raise HTTPException(
                status_code=400,
                detail=f"Language '{request.target_language}' is not supported. Supported languages: {list(supported_langs.keys())}"
            )

        translator = TextTranslator()
        translated_text = await translator.translate(request.text, request.target_language)

        # Determine if target language is right-to-left
        is_rtl = is_rtl_language(request.target_language)

        response = MultilingualTextResponse(
            original_text=request.text,
            processed_text=translated_text,
            source_language=request.source_language or "auto",
            target_language=request.target_language,
            is_rtl=is_rtl,
            operation="translate",
            confidence=0.90,  # Placeholder confidence score
            metadata={
                "character_count_original": len(request.text),
                "character_count_translated": len(translated_text),
                "word_count_original": len(request.text.split()),
                "word_count_translated": len(translated_text.split()),
            }
        )

        logger.info(f"Translated text to {request.target_language}: {len(request.text)} -> {len(translated_text)} chars")

        return response

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Translation failed: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Translation failed: {str(e)}"
        )


@router.post("/clarify", response_model=MultilingualTextResponse)
async def clarify_text_endpoint(request: MultilingualTextRequest) -> MultilingualTextResponse:
    """
    Clarify text to make it clearer and more concise, preserving the original language.

    Args:
        request: Clarification request with text

    Returns:
        Clarified text with metadata
    """
    try:
        # Override operation to clarify
        request.operation = "clarify"

        # Validate language codes if provided
        if request.source_language:
            supported_langs = get_translation_supported_languages()
            if request.source_language not in supported_langs:
                raise HTTPException(
                    status_code=400,
                    detail=f"Language '{request.source_language}' is not supported. Supported languages: {list(supported_langs.keys())}"
                )

        clarifier = TextClarifier()
        clarified_text = await clarifier.clarify(request.text)

        # Determine source language (this would normally be detected)
        source_lang = request.source_language or "auto"

        # Determine if source language is right-to-left
        is_rtl = is_rtl_language(source_lang) if source_lang != "auto" else False

        response = MultilingualTextResponse(
            original_text=request.text,
            processed_text=clarified_text,
            source_language=source_lang,
            target_language=source_lang,
            is_rtl=is_rtl,
            operation="clarify",
            confidence=0.85,  # Placeholder confidence score
            metadata={
                "character_count_original": len(request.text),
                "character_count_clarified": len(clarified_text),
                "word_count_original": len(request.text.split()),
                "word_count_clarified": len(clarified_text.split()),
            }
        )

        logger.info(f"Clarified text: {len(request.text)} -> {len(clarified_text)} chars")

        return response

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Clarification failed: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Clarification failed: {str(e)}"
        )


@router.get("/languages")
async def get_supported_languages() -> Dict[str, Any]:
    """
    Get list of all supported languages with their details.

    Returns:
        Dictionary of supported languages
    """
    try:
        # Get languages from the text processor config
        languages = get_all_languages()

        # Also get languages from translation config for completeness
        translation_langs = get_translation_supported_languages()

        # Format response
        formatted_languages = []
        for lang in languages:
            formatted_languages.append({
                "code": lang.code,
                "name": lang.name,
                "native_name": lang.native_name,
                "is_rtl": is_rtl_language(lang.code),
                "has_translations": lang.code in translation_langs
            })

        return {
            "languages": formatted_languages,
            "total_count": len(formatted_languages),
            "rtl_languages": [lang["code"] for lang in formatted_languages if lang["is_rtl"]],
            "l2r_languages": [lang["code"] for lang in formatted_languages if not lang["is_rtl"]],
            "translation_support": {
                "available_count": len(translation_langs),
                "supported_codes": list(translation_langs.keys())
            }
        }
    except Exception as e:
        logger.error(f"Getting languages failed: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Failed to get supported languages: {str(e)}"
        )


@router.get("/health")
async def text_processing_health():
    """
    Health check for the text processing service.

    Returns:
        Health status of the text processing service
    """
    supported_langs = get_translation_supported_languages()
    return {
        "status": "healthy",
        "service": "text-processing",
        "features": {
            "multilingual_support": True,
            "urdu_support": "ur" in supported_langs,
            "roman_urdu_support": "ur-PK" in supported_langs,
            "right_to_left_layout": True,
            "translation": True,
            "clarification": True,
            "rag_integration": True,
            "supported_languages_count": len(supported_langs),
            "languages": list(supported_langs.keys())
        }
    }


@router.post("/detect-language")
async def detect_language_post(text: str) -> Dict[str, str]:
    """
    Detect the language of the provided text (simplified detection).

    Args:
        text: Text to detect language for

    Returns:
        Detected language code and name
    """
    # This is a simplified language detection based on character sets
    # In a real implementation, we would use a more sophisticated approach
    arabic_chars = sum(1 for c in text if '\u0600' <= c <= '\u06FF')
    latin_chars = sum(1 for c in text if '\u0041' <= c <= '\u007A' or '\u00C0' <= c <= '\u00FF')

    if arabic_chars > latin_chars:
        detected_lang = "ur"  # Urdu/Arabic script
    elif text.encode('utf-8', errors='ignore').lower().startswith(('a', 'e', 'i', 'o', 'u')):
        detected_lang = "en"  # Simplified English detection
    else:
        detected_lang = "en"  # Default to English

    supported_langs = get_translation_supported_languages()
    if detected_lang not in supported_langs:
        detected_lang = "en"  # Fall back to English if detected language not supported

    return {
        "detected_language": detected_lang,
        "language_name": get_translation(detected_lang, "language", detected_lang),
        "script_type": "RTL" if is_rtl_language(detected_lang) else "LTR",
        "is_supported": detected_lang in supported_langs
    }

# Add GET version for consistency
@router.get("/detect-language")
async def detect_language_get(request: Request) -> Dict[str, str]:
    """
    Detect the language from the request context.

    Args:
        request: FastAPI request object

    Returns:
        Detected language code and name
    """
    from .translation_middleware import get_request_language
    detected_lang = get_request_language(request)
    supported_langs = get_translation_supported_languages()

    return {
        "detected_language": detected_lang,
        "language_name": get_translation(detected_lang, "language", detected_lang) if detected_lang in supported_langs else "Unknown",
        "script_type": "RTL" if is_rtl_language(detected_lang) else "LTR",
        "is_supported": detected_lang in supported_langs
    }