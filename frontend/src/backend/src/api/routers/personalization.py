"""Personalization API endpoints."""
from typing import Optional, Dict, Any
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from src.config.translations import get_supported_languages, is_rtl_language

router = APIRouter(prefix="/personalization", tags=["personalization"])

# In-memory storage for demonstration (in production, use a database)
user_preferences = {}

class BookmarkCreate(BaseModel):
    """Bookmark creation model."""
    content_reference: str
    title: Optional[str] = None
    notes: Optional[str] = None


class PreferenceUpdate(BaseModel):
    """Preference update model."""
    language: Optional[str] = None
    theme: Optional[str] = None
    settings: Optional[dict] = None


@router.get("/bookmarks")
async def get_bookmarks(user_id: str = "demo-user"):
    """Get user bookmarks."""
    # Initialize user if not exists
    if user_id not in user_preferences:
        user_preferences[user_id] = {
            "language": "en",
            "theme": "light",
            "settings": {},
            "bookmarks": []
        }

    user_data = user_preferences[user_id]
    return {
        "user_id": user_id,
        "bookmarks": user_data.get("bookmarks", []),
    }


@router.post("/bookmarks")
async def create_bookmark(bookmark: BookmarkCreate, user_id: str = "demo-user"):
    """Create a new bookmark."""
    # Initialize user if not exists
    if user_id not in user_preferences:
        user_preferences[user_id] = {
            "language": "en",
            "theme": "light",
            "settings": {},
            "bookmarks": []
        }

    bookmark_id = f"bookmark_{len(user_preferences[user_id]['bookmarks']) + 1}"
    new_bookmark = {
        "id": bookmark_id,
        "content_reference": bookmark.content_reference,
        "title": bookmark.title,
        "notes": bookmark.notes
    }

    user_preferences[user_id]["bookmarks"].append(new_bookmark)

    return {
        "success": True,
        "bookmark_id": bookmark_id,
        "message": "Bookmark created",
        "bookmark": new_bookmark
    }


@router.get("/preferences")
async def get_preferences(user_id: str = "demo-user"):
    """Get user preferences."""
    # Initialize user if not exists
    if user_id not in user_preferences:
        user_preferences[user_id] = {
            "language": "en",
            "theme": "light",
            "settings": {},
            "bookmarks": []
        }

    user_data = user_preferences[user_id]

    # Add language information
    supported = get_supported_languages()
    current_language_info = {
        "code": user_data["language"],
        "name": supported.get(user_data["language"], "Unknown"),
        "is_rtl": is_rtl_language(user_data["language"]),
        "supported": user_data["language"] in supported
    }

    return {
        "user_id": user_id,
        "language": user_data["language"],
        "language_info": current_language_info,
        "theme": user_data["theme"],
        "settings": user_data["settings"],
        "supported_languages": list(supported.keys()),
    }


@router.put("/preferences")
async def update_preferences(prefs: PreferenceUpdate, user_id: str = "demo-user"):
    """Update user preferences with validation."""
    # Initialize user if not exists
    if user_id not in user_preferences:
        user_preferences[user_id] = {
            "language": "en",
            "theme": "light",
            "settings": {},
            "bookmarks": []
        }

    # Validate language if provided
    if prefs.language is not None:
        supported = get_supported_languages()
        if prefs.language not in supported:
            raise HTTPException(
                status_code=400,
                detail=f"Language '{prefs.language}' is not supported. Supported languages: {list(supported.keys())}"
            )
        user_preferences[user_id]["language"] = prefs.language

    # Update theme if provided
    if prefs.theme is not None:
        user_preferences[user_id]["theme"] = prefs.theme

    # Update settings if provided
    if prefs.settings is not None:
        user_preferences[user_id]["settings"] = {**user_preferences[user_id]["settings"], **prefs.settings}

    # Get updated language information
    updated_language = user_preferences[user_id]["language"]
    supported = get_supported_languages()
    language_info = {
        "code": updated_language,
        "name": supported.get(updated_language, "Unknown"),
        "is_rtl": is_rtl_language(updated_language),
        "supported": updated_language in supported
    }

    return {
        "success": True,
        "message": "Preferences updated",
        "updated_preferences": {
            "language": user_preferences[user_id]["language"],
            "language_info": language_info,
            "theme": user_preferences[user_id]["theme"],
            "settings": user_preferences[user_id]["settings"]
        }
    }


@router.get("/health")
async def personalization_health():
    """Check personalization service health."""
    supported = get_supported_languages()
    return {
        "status": "healthy",
        "service": "personalization",
        "features": {
            "user_preferences": True,
            "language_support": True,
            "urdu_support": "ur" in supported,
            "roman_urdu_support": "ur-PK" in supported,
            "rtl_layout": True
        }
    }
