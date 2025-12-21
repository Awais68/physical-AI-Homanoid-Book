"""
Translation API Router
Provides endpoints for language management and translations
"""

from fastapi import APIRouter, Request
from typing import Dict, List
from src.config.translations import get_translation, get_supported_languages, is_rtl_language, TRANSLATIONS
from src.api.translation_middleware import get_request_language

router = APIRouter()

@router.get("/translations/languages")
async def get_languages() -> Dict[str, str]:
    """Get all supported languages"""
    return get_supported_languages()

@router.get("/translations/{lang}")
async def get_language_translations(lang: str) -> Dict[str, str]:
    """Get all translations for a specific language"""
    if lang not in TRANSLATIONS:
        return {"error": f"Language '{lang}' not supported"}
    
    return TRANSLATIONS[lang]

@router.get("/translations/{lang}/{key}")
async def get_single_translation(lang: str, key: str) -> Dict[str, str]:
    """Get a specific translation"""
    translation = get_translation(lang, key)
    return {
        "language": lang,
        "key": key,
        "translation": translation,
        "is_rtl": is_rtl_language(lang)
    }

@router.get("/translations/detect")
async def detect_language(request: Request) -> Dict[str, str]:
    """Detect the current request language"""
    lang = get_request_language(request)
    return {
        "detected_language": lang,
        "language_name": get_supported_languages().get(lang, "Unknown"),
        "is_rtl": is_rtl_language(lang)
    }
