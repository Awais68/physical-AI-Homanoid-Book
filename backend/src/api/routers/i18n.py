"""Internationalization API endpoints."""
from fastapi import APIRouter

router = APIRouter(prefix="/i18n", tags=["i18n"])


@router.get("/languages")
async def get_available_languages():
    """Get list of available languages."""
    return {
        "languages": [
            {"code": "en", "name": "English", "native_name": "English"},
            {"code": "es", "name": "Spanish", "native_name": "Espanol"},
            {"code": "fr", "name": "French", "native_name": "Francais"},
            {"code": "de", "name": "German", "native_name": "Deutsch"},
            {"code": "zh", "name": "Chinese", "native_name": "Chinese"},
        ],
        "default": "en",
    }


@router.get("/translations/{language_code}")
async def get_translations(language_code: str):
    """Get translations for a specific language."""
    return {
        "language": language_code,
        "translations": {
            "welcome": "Welcome to Physical AI Edge Kit",
            "chat": "Chat with AI Assistant",
            "help": "Help",
        },
    }


@router.get("/health")
async def i18n_health():
    """Check i18n service health."""
    return {"status": "healthy", "service": "internationalization"}
