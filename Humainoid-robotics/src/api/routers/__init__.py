"""API routers module."""
from . import chat

# Optional routers - may fail without database dependencies
try:
    from . import devices
    from . import safety
    from . import users
    from . import personalization
    from . import i18n
except ImportError:
    devices = None
    safety = None
    users = None
    personalization = None
    i18n = None

__all__ = [
    "chat",
]
