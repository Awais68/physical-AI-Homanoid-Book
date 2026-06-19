"""
Translation middleware for FastAPI
Handles language detection and response translation
"""

from fastapi import Request
from typing import Optional
import re

class TranslationMiddleware:
    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] == "http":
            request = Request(scope, receive)

            # Detect language from various sources
            lang = self.detect_language(request)

            # Add language to request state
            if "state" not in scope:
                scope["state"] = {}
            scope["state"]["lang"] = lang

        await self.app(scope, receive, send)

    def detect_language(self, request: Request) -> str:
        """
        Detect language from request
        Priority: 1. User preference (via API) 2. Query param 3. Header 4. Cookie 5. Default
        """
        # 1. Check for user preference in path (e.g., /api/i18n/translate?lang=ur)
        lang = request.query_params.get("lang")
        if lang:
            validated_lang = self.validate_language(lang)
            if validated_lang != "en":  # Only return if it's a valid non-default language
                return validated_lang

        # 2. Check for language in request path patterns that might indicate preferred language
        # This could come from frontend routing or user preferences stored in session

        # 3. Check Accept-Language header
        accept_language = request.headers.get("Accept-Language")
        if accept_language:
            # Parse Accept-Language header (e.g., "en-US,en;q=0.9,ur;q=0.8")
            languages = self.parse_accept_language(accept_language)
            if languages:
                for lang_code in languages:
                    validated_lang = self.validate_language(lang_code)
                    if validated_lang != "en":  # Prioritize non-English languages if user specified them
                        return validated_lang

        # 4. Check cookie (could be set by frontend based on user selection)
        lang = request.cookies.get("NEXT_LOCALE") or request.cookies.get("locale")
        if lang:
            validated_lang = self.validate_language(lang)
            if validated_lang != "en":
                return validated_lang

        # 5. Default to English
        return "en"

    def parse_accept_language(self, header: str) -> list:
        """Parse Accept-Language header and return sorted list of languages by priority"""
        languages = []
        # Split by comma and sort by quality value (q=x.x)
        items = []
        for item in header.split(","):
            parts = item.strip().split(";")
            lang = parts[0].strip()
            quality = 1.0  # Default quality

            # Extract quality value if present (e.g., "ur;q=0.8")
            for part in parts[1:]:
                if part.strip().startswith("q="):
                    try:
                        quality = float(part.strip()[2:])
                    except ValueError:
                        pass
            items.append((lang, quality))

        # Sort by quality (highest first)
        items.sort(key=lambda x: x[1], reverse=True)
        for lang, _ in items:
            # Extract language code (e.g., "en-US" -> "en")
            lang_code = lang.split("-")[0]
            if lang_code not in languages:  # Avoid duplicates
                languages.append(lang_code)

        return languages

    def validate_language(self, lang: str) -> str:
        """Validate and normalize language code"""
        # List of supported languages
        supported = ['en', 'ur', 'ur-PK', 'ar', 'es', 'fr', 'de', 'zh', 'hi', 'pt', 'ru', 'ja']

        if lang in supported:
            return lang

        # Handle common variations and aliases
        lang_aliases = {
            'urdu': 'ur',
            'roman-urdu': 'ur-PK',
            'roman_urdu': 'ur-PK',
            'ur-pk': 'ur-PK',
            'uk': 'ur',
            'pakistani': 'ur-PK',
            'english': 'en',
            'spanish': 'es',
            'french': 'fr',
            'german': 'de',
            'chinese': 'zh',
            'japanese': 'ja',
            'hindi': 'hi',
            'portuguese': 'pt',
            'russian': 'ru',
            'arabic': 'ar',
        }

        lang_lower = lang.lower().replace('_', '-').strip()

        if lang_lower in supported:
            return lang_lower

        if lang_lower in lang_aliases:
            return lang_aliases[lang_lower]

        # Handle locale variants (e.g., ur_PK -> ur-PK)
        if '_' in lang_lower:
            locale_variant = lang_lower.replace('_', '-')
            if locale_variant in supported:
                return locale_variant

        # Try to find partial match
        for supported_lang in supported:
            if lang_lower == supported_lang.lower():
                return supported_lang

        # Default to English
        return "en"

def get_request_language(request: Request) -> str:
    """Helper function to get language from request state"""
    if hasattr(request.state, 'lang'):
        return getattr(request.state, "lang", "en")
    # Fallback for cases where middleware didn't set the language
    return "en"
