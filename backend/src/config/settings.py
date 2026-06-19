"""Application configuration settings."""
import os
from typing import Optional
from pathlib import Path
from pydantic_settings import BaseSettings
from pydantic import ConfigDict

# Load .env from the project root (frontend/src/)
_backend_dir = Path(__file__).resolve().parent  # src/config/
_src_dir = _backend_dir.parent.parent           # src/
_dotenv_path = _src_dir / ".env"
if _dotenv_path.exists():
    from dotenv import load_dotenv
    load_dotenv(_dotenv_path)


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    model_config = ConfigDict(
        env_file=str(_dotenv_path),
        env_file_encoding="utf-8",
        case_sensitive=True,
        extra="ignore",
    )

    # Application
    APP_NAME: str = "Physical AI Edge Kit API"
    APP_VERSION: str = "1.0.0"
    DEBUG: bool = False

    # Database - Neon Postgres
    DATABASE_URL: str = os.getenv("DATABASE_URL", "postgresql://localhost:5432/edgekit")

    # Qdrant Vector Database
    QDRANT_URL: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")
    QDRANT_COLLECTION: str = os.getenv("QDRANT_COLLECTION", "physical_ai_docs")

    # OpenRouter (DEFAULT provider - chat + embeddings)
    OPENROUTER_API_KEY: str = os.getenv("OPENROUTER_API_KEY", "").strip()
    OPENROUTER_CHAT_MODEL: str = os.getenv("OPENROUTER_CHAT_MODEL", "openai/gpt-oss-20b:free").strip()
    OPENROUTER_EMBEDDING_MODEL: str = os.getenv("OPENROUTER_EMBEDDING_MODEL", "nvidia/llama-nemotron-embed-vl-1b-v2:free").strip()
    EMBEDDING_DIM: int = int(os.getenv("EMBEDDING_DIM", "2048"))

    # Cohere (fallback for embeddings - matches legacy data)
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "").strip()
    COHERE_EMBEDDING_MODEL: str = os.getenv("COHERE_EMBEDDING_MODEL", "embed-english-v3.0")

    # Gemini (for chat completion)
    GEMINI_API_KEY: str = os.getenv("GEMINI_API_KEY", "")
    GEMINI_CHAT_MODEL: str = os.getenv("GEMINI_CHAT_MODEL", "gemini-2.5-flash")

    # OpenAI (fallback)
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "")
    OPENAI_EMBEDDING_MODEL: str = os.getenv("OPENAI_EMBEDDING_MODEL", "text-embedding-3-small")
    OPENAI_CHAT_MODEL: str = os.getenv("OPENAI_CHAT_MODEL", "gpt-4o-mini")

    # RAG Configuration
    RAG_CHUNK_SIZE: int = 500
    RAG_CHUNK_OVERLAP: int = 50
    RAG_TOP_K: int = 5
    RAG_SIMILARITY_THRESHOLD: float = 0.20
    RAG_MAX_RESPONSE_TOKENS: int = 1000

    # Authentication
    SECRET_KEY: str = os.getenv("SECRET_KEY", "dev-secret-key-change-in-production")
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30

    # CORS
    CORS_ORIGINS: list = ["*"]


settings = Settings()
