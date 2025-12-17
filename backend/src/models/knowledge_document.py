"""KnowledgeDocument model for RAG indexed content."""
from datetime import datetime
from sqlalchemy import Column, String, DateTime, JSON, Text, Integer
import uuid
from src.config.database import Base


class KnowledgeDocument(Base):
    """Represents indexed content from documentation and educational materials."""

    __tablename__ = "knowledge_documents"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    content = Column(Text, nullable=False)
    title = Column(String(500), nullable=True)
    source_url = Column(String(1000), nullable=True)
    file_path = Column(String(1000), nullable=True)
    section = Column(String(500), nullable=True)
    tags = Column(JSON, default=list)
    doc_metadata = Column(JSON, default=dict)
    chunk_index = Column(Integer, default=0)
    embedding_id = Column(String(255), nullable=True)  # ID in Qdrant
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    def to_dict(self) -> dict:
        """Convert model to dictionary."""
        return {
            "id": str(self.id),
            "content": self.content,
            "title": self.title,
            "source_url": self.source_url,
            "file_path": self.file_path,
            "section": self.section,
            "tags": self.tags,
            "metadata": self.doc_metadata,
            "chunk_index": self.chunk_index,
            "embedding_id": self.embedding_id,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None,
        }

    def to_payload(self) -> dict:
        """Convert to Qdrant payload format."""
        return {
            "id": str(self.id),
            "content": self.content,
            "title": self.title or "",
            "source_url": self.source_url or "",
            "file_path": self.file_path or "",
            "section": self.section or "",
            "tags": self.tags or [],
            "chunk_index": self.chunk_index,
        }
