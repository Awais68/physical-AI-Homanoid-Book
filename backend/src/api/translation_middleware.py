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
            scope["state"] = getattr(scope, "state", {})
            scope["state"]["lang"] = lang
        
        await self.app(scope, receive, send)
    
    def detect_language(self, request: Request) -> str:
        """
        Detect language from request
        Priority: 1. Query param 2. Header 3. Cookie 4. Default
        """
        # 1. Check query parameter
        lang = request.query_params.get("lang")
        if lang:
            return self.validate_language(lang)
        
        # 2. Check Accept-Language header
        accept_language = request.headers.get("Accept-Language")
        if accept_language:
            # Parse Accept-Language header (e.g., "en-US,en;q=0.9,ur;q=0.8")
            languages = self.parse_accept_language(accept_language)
            if languages:
                return self.validate_language(languages[0])
        
        # 3. Check cookie
        lang = request.cookies.get("NEXT_LOCALE")
        if lang:
            return self.validate_language(lang)
        
        # 4. Default to English
        return "en"
    
    def parse_accept_language(self, header: str) -> list:
        """Parse Accept-Language header and return sorted list of languages"""
        languages = []
        for item in header.split(","):
            parts = item.strip().split(";")
            lang = parts[0].strip()
            # Extract language code (e.g., "en-US" -> "en")
            lang_code = lang.split("-")[0]
            languages.append(lang_code)
        return languages
    
    def validate_language(self, lang: str) -> str:
        """Validate and normalize language code"""
        # List of supported languages
        supported = ['en', 'ur', 'ur-PK', 'ar', 'es', 'fr', 'de', 'zh', 'hi', 'pt', 'ru', 'ja']
        
        if lang in supported:
            return lang
        
        # Try to find partial match
        for supported_lang in supported:
            if lang.lower() == supported_lang.lower():
                return supported_lang
        
        return "en"

def get_request_language(request: Request) -> str:
    """Helper function to get language from request state"""
    return getattr(request.state, "lang", "en")
