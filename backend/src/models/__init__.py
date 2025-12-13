"""Data models module."""
from src.config.database import Base
from .edge_device import EdgeDevice
from .chat_session import ChatSession
from .knowledge_document import KnowledgeDocument
from .user_preference import UserPreference
from .bookmark import Bookmark

__all__ = [
    "Base",
    "EdgeDevice",
    "ChatSession",
    "KnowledgeDocument",
    "UserPreference",
    "Bookmark",
]
