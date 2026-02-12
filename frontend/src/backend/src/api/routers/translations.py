"""
Translation API Router
Provides endpoints for language management and translations
"""

from fastapi import APIRouter, Request, HTTPException
from typing import Dict, List, Any
from src.config.translations import get_translation, get_supported_languages, is_rtl_language, TRANSLATIONS
from src.api.translation_middleware import get_request_language

router = APIRouter(prefix="/translations", tags=["translations"])

@router.get("/languages")
async def get_languages() -> Dict[str, Any]:
    """Get all supported languages with comprehensive details"""
    supported = get_supported_languages()

    # Format with additional metadata
    languages_with_details = []
    for code, name in supported.items():
        languages_with_details.append({
            "code": code,
            "name": name,
            "is_rtl": is_rtl_language(code),
            "has_translations": code in TRANSLATIONS,
            "translation_count": len(TRANSLATIONS.get(code, {})) if code in TRANSLATIONS else 0
        })

    return {
        "languages": languages_with_details,
        "total_count": len(supported),
        "rtl_languages": [code for code in supported.keys() if is_rtl_language(code)],
        "l2r_languages": [code for code in supported.keys() if not is_rtl_language(code)],
        "summary": supported
    }

@router.get("/{lang}")
async def get_language_translations(lang: str) -> Dict[str, Any]:
    """Get all translations for a specific language"""
    supported = get_supported_languages()
    if lang not in supported:
        raise HTTPException(
            status_code=404,
            detail=f"Language '{lang}' is not supported. Supported languages: {list(supported.keys())}"
        )

    if lang not in TRANSLATIONS:
        return {
            "error": f"No translations available for '{lang}'",
            "language": lang,
            "available_keys": []
        }

    translations = TRANSLATIONS[lang]
    return {
        "language": lang,
        "language_name": supported[lang],
        "is_rtl": is_rtl_language(lang),
        "translation_count": len(translations),
        "translations": translations
    }

@router.get("/{lang}/{key}")
async def get_single_translation(lang: str, key: str) -> Dict[str, Any]:
    """Get a specific translation with enhanced details"""
    supported = get_supported_languages()
    if lang not in supported:
        raise HTTPException(
            status_code=404,
            detail=f"Language '{lang}' is not supported. Supported languages: {list(supported.keys())}"
        )

    translation = get_translation(lang, key)

    # Get the translation in English as fallback reference
    english_translation = get_translation('en', key, key)

    return {
        "language": lang,
        "language_name": supported[lang],
        "key": key,
        "translation": translation,
        "english_reference": english_translation,
        "is_rtl": is_rtl_language(lang),
        "exists_in_source": key in TRANSLATIONS.get(lang, {}),
        "fallback_applied": translation != get_translation(lang, key, None) if key in TRANSLATIONS.get(lang, {}) else False
    }

@router.post("/translate")
async def translate_text(request_data: Dict[str, str]):
    """Translate text from source to target language"""
    source_text = request_data.get("text", "")
    source_lang = request_data.get("source_language", "auto")
    target_lang = request_data.get("target_language", "en")

    supported = get_supported_languages()
    if target_lang not in supported:
        raise HTTPException(
            status_code=400,
            detail=f"Target language '{target_lang}' is not supported. Supported languages: {list(supported.keys())}"
        )

    # For now, we'll use a simple approach - in a real implementation, this would use AI translation
    # Here we'll look for direct term translations if the source text matches a known key
    translated_text = get_translation(target_lang, source_text.lower().replace(" ", "_"), source_text)

    return {
        "original_text": source_text,
        "source_language": source_lang,
        "target_language": target_lang,
        "translated_text": translated_text,
        "is_rtl_target": is_rtl_language(target_lang),
        "is_rtl_source": is_rtl_language(source_lang) if source_lang in supported else False
    }

@router.get("/detect")
async def detect_language(request: Request) -> Dict[str, Any]:
    """Detect the current request language with enhanced details"""
    detected_lang = get_request_language(request)
    supported = get_supported_languages()

    return {
        "detected_language": detected_lang,
        "language_name": supported.get(detected_lang, "Unknown"),
        "is_rtl": is_rtl_language(detected_lang),
        "supported": detected_lang in supported,
        "available_languages": list(supported.keys()),
        "confidence": 1.0  # For now, assuming 100% confidence in detection from middleware
    }

@router.get("/health")
async def translations_health():
    """Health check for the translation service"""
    supported = get_supported_languages()
    return {
        "status": "healthy",
        "service": "translations",
        "supported_languages_count": len(supported),
        "languages": list(supported.keys()),
        "features": {
            "urdu_support": "ur" in supported,
            "roman_urdu_support": "ur-PK" in supported,
            "rtl_support": True,
            "comprehensive_translations": True,
            "middleware_integration": True
        }
    }
