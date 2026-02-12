"""Internationalization API endpoints."""
from fastapi import APIRouter
from typing import Dict, Any

from src.config.translations import get_supported_languages, TRANSLATIONS, get_translation

router = APIRouter(prefix="/i18n", tags=["i18n"])


@router.get("/languages")
async def get_available_languages():
    """Get list of available languages with comprehensive details."""
    supported = get_supported_languages()

    # Format the languages with detailed information
    languages_detail = []
    for code, name in supported.items():
        # Get a sample translation to demonstrate the language
        sample_welcome = get_translation(code, "welcome", "Welcome")

        # Determine if the language is right-to-left
        is_rtl = code in ['ur', 'ar']  # Urdu and Arabic are RTL

        languages_detail.append({
            "code": code,
            "name": name,
            "native_name": sample_welcome,  # This will show the native name
            "is_rtl": is_rtl,
            "has_full_support": code in TRANSLATIONS
        })

    return {
        "languages": languages_detail,
        "default": "en",
        "total_count": len(supported),
        "rtl_languages": [code for code in supported.keys() if code in ['ur', 'ar']],
        "l2r_languages": [code for code in supported.keys() if code not in ['ur', 'ar']]
    }


@router.get("/translations/{language_code}")
async def get_translations(language_code: str):
    """Get translations for a specific language."""
    # Validate that the language is supported
    supported = get_supported_languages()
    if language_code not in supported:
        return {
            "error": f"Language '{language_code}' is not supported",
            "supported_languages": list(supported.keys()),
            "language": language_code,
            "translations": {}
        }

    # Get the full translation dictionary for the language
    translations_dict = TRANSLATIONS.get(language_code, {})

    return {
        "language": language_code,
        "language_name": supported[language_code],
        "is_rtl": language_code in ['ur', 'ar'],
        "translation_count": len(translations_dict),
        "translations": translations_dict,
    }


@router.get("/translations/{language_code}/{key}")
async def get_single_translation(language_code: str, key: str):
    """Get a specific translation key for a language."""
    supported = get_supported_languages()
    if language_code not in supported:
        return {
            "error": f"Language '{language_code}' is not supported",
            "language": language_code,
            "key": key,
            "translation": None
        }

    translation = get_translation(language_code, key)

    return {
        "language": language_code,
        "key": key,
        "translation": translation,
        "is_rtl": language_code in ['ur', 'ar']
    }


@router.get("/health")
async def i18n_health():
    """Check i18n service health."""
    supported = get_supported_languages()
    return {
        "status": "healthy",
        "service": "internationalization",
        "supported_languages_count": len(supported),
        "languages": list(supported.keys()),
        "features": {
            "urdu_support": "ur" in supported,
            "roman_urdu_support": "ur-PK" in supported,
            "rtl_support": True,
            "comprehensive_translations": True
        }
    }
