"""Application configuration settings."""
import os
from typing import Optional
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Application
    APP_NAME: str = "Physical AI Edge Kit API"
    APP_VERSION: str = "1.0.0"
    DEBUG: bool = False

    # Database - Neon Postgres
    DATABASE_URL: str = os.getenv("DATABASE_URL", "postgresql://localhost:5432/edgekit")

    # Qdrant Vector Database
    QDRANT_URL: str = os.getenv("QDRANT_URL", "http://localhost:6333").strip()
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY", "").strip() or None
    QDRANT_COLLECTION: str = os.getenv("QDRANT_COLLECTION", "physical_ai_docs").strip()

    # Cohere (for embeddings - matches existing data)
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "").strip()
    COHERE_EMBEDDING_MODEL: str = os.getenv("COHERE_EMBEDDING_MODEL", "embed-english-v3.0").strip()

    # Gemini (for chat completion)
    GEMINI_API_KEY: str = os.getenv("GEMINI_API_KEY", "").strip()
    GEMINI_CHAT_MODEL: str = os.getenv("GEMINI_CHAT_MODEL", "gemini-2.5-flash").strip()

    # OpenAI (fallback)
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "").strip()
    OPENAI_EMBEDDING_MODEL: str = os.getenv("OPENAI_EMBEDDING_MODEL", "text-embedding-3-small")
    OPENAI_CHAT_MODEL: str = os.getenv("OPENAI_CHAT_MODEL", "gpt-4o-mini")

    # RAG Configuration
    RAG_CHUNK_SIZE: int = 500
    RAG_CHUNK_OVERLAP: int = 50
    RAG_TOP_K: int = 5
    RAG_SIMILARITY_THRESHOLD: float = 0.4
    RAG_MAX_RESPONSE_TOKENS: int = 1000

    # Authentication
    SECRET_KEY: str = os.getenv("SECRET_KEY", "dev-secret-key-change-in-production")
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30

    # CORS
    CORS_ORIGINS: list = ["*"]

    class Config:
        env_file = ".env"
        case_sensitive = True
        extra = "ignore"


settings = Settings()
