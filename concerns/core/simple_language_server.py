#!/usr/bin/env python3
"""
Simplified Physical AI Edge Kit Server
Focuses on multilingual functionality without database dependencies
"""

import os
import sys
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import the translation functionality (this works without database)
from backend.src.config.translations import get_translation, get_supported_languages, is_rtl_language
from backend.src.api.translation_middleware import TranslationMiddleware

# Create a simplified app without database dependencies
app = FastAPI(
    title="Physical AI Edge Kit - Language Services",
    description="Multilingual API with Urdu and Roman Urdu support",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add translation middleware
from backend.src.api.translation_middleware import TranslationMiddleware
app.add_middleware(TranslationMiddleware)

@app.get("/")
async def root():
    return {
        "message": "Physical AI Edge Kit - Language Services Running",
        "features": {
            "multilingual_support": True,
            "urdu_support": True,
            "roman_urdu_support": True,
            "rtl_layout": True
        }
    }

@app.get("/api/i18n/languages")
async def get_available_languages():
    """Get all supported languages."""
    supported = get_supported_languages()

    languages_detail = []
    for code, name in supported.items():
        sample_welcome = get_translation(code, "welcome", "Welcome")
        is_rtl = code in ['ur', 'ar']

        languages_detail.append({
            "code": code,
            "name": name,
            "native_name": sample_welcome,
            "is_rtl": is_rtl,
            "has_full_support": code in ['en', 'ur', 'ur-PK', 'ar', 'es', 'fr', 'de', 'zh', 'hi', 'pt', 'ru', 'ja']
        })

    return {
        "languages": languages_detail,
        "total_count": len(supported),
        "default": "en",
        "features": {
            "urdu_support": "ur" in supported,
            "roman_urdu_support": "ur-PK" in supported,
            "rtl_support": True
        }
    }

@app.get("/api/translations/{language_code}")
async def get_language_translations(language_code: str):
    """Get translations for a specific language."""
    from backend.src.config.translations import TRANSLATIONS

    supported = get_supported_languages()
    if language_code not in supported:
        raise HTTPException(
            status_code=404,
            detail=f"Language '{language_code}' is not supported. Supported: {list(supported.keys())}"
        )

    if language_code not in TRANSLATIONS:
        return {
            "error": f"No translations available for '{language_code}'",
            "language": language_code,
            "available_keys": []
        }

    translations = TRANSLATIONS[language_code]
    return {
        "language": language_code,
        "language_name": supported[language_code],
        "is_rtl": is_rtl_language(language_code),
        "translation_count": len(translations),
        "translations": translations
    }

@app.get("/api/translations/{language_code}/{key}")
async def get_single_translation(language_code: str, key: str):
    """Get a specific translation."""
    supported = get_supported_languages()
    if language_code not in supported:
        raise HTTPException(
            status_code=404,
            detail=f"Language '{language_code}' is not supported"
        )

    translation = get_translation(language_code, key)
    return {
        "language": language_code,
        "key": key,
        "translation": translation,
        "is_rtl": is_rtl_language(language_code)
    }

@app.get("/api/health")
async def health_check():
    """Health check endpoint."""
    supported = get_supported_languages()
    return {
        "status": "healthy",
        "service": "language-services",
        "supported_languages_count": len(supported),
        "languages": list(supported.keys()),
        "features": {
            "urdu_support": "ur" in supported,
            "roman_urdu_support": "ur-PK" in supported,
            "rtl_support": True,
            "comprehensive_translations": True
        }
    }

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080, help="Port to run the server on")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host to bind to")
    args = parser.parse_args()

    print("🚀 Starting Physical AI Edge Kit - Language Services...")
    print(f"🌐 Access the API at: http://{args.host}:{args.port}")
    print(f"📖 API Documentation at: http://{args.host}:{args.port}/docs")
    print(f"🌍 Available languages: {list(get_supported_languages().keys())}")
    print()
    print("Press Ctrl+C to stop the server")

    uvicorn.run(app, host=args.host, port=args.port)