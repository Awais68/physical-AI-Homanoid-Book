"""API routers module."""
from . import devices
from . import chat
from . import safety
from . import users
from . import personalization
from . import i18n

__all__ = [
    "devices",
    "chat",
    "safety",
    "users",
    "personalization",
    "i18n",
]
