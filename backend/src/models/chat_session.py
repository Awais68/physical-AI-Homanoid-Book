"""ChatSession model for RAG chatbot conversations."""
from datetime import datetime
from typing import Optional
from sqlalchemy import Column, String, DateTime, JSON, Text
from sqlalchemy.dialects.postgresql import UUID
import uuid
from src.config.database import Base


class ChatSession(Base):
    """Represents a conversation session with the RAG chatbot."""

    __tablename__ = "chat_sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(String(255), nullable=True, index=True)
    conversation_history = Column(JSON, default=list)
    context = Column(JSON, default=dict)
    source_citations = Column(JSON, default=list)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    def to_dict(self) -> dict:
        """Convert model to dictionary."""
        return {
            "id": str(self.id),
            "user_id": self.user_id,
            "conversation_history": self.conversation_history,
            "context": self.context,
            "source_citations": self.source_citations,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None,
        }

    def add_message(self, role: str, content: str) -> None:
        """Add a message to conversation history.

        Args:
            role: Message role (user, assistant, system).
            content: Message content.
        """
        if self.conversation_history is None:
            self.conversation_history = []
        self.conversation_history.append({
            "role": role,
            "content": content,
            "timestamp": datetime.utcnow().isoformat(),
        })

    def add_citation(self, source: dict) -> None:
        """Add a source citation.

        Args:
            source: Source document information.
        """
        if self.source_citations is None:
            self.source_citations = []
        self.source_citations.append(source)
