"""Application configuration management."""

import os
from functools import lru_cache

from dotenv import load_dotenv
from pydantic import Field
from pydantic_settings import BaseSettings

# Load environment variables from .env file
load_dotenv()


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # OpenAI API Configuration
    openai_api_key: str = Field(
        default="",
        description="OpenAI API key for LLM access",
    )

    # Application Settings
    max_input_length: int = Field(
        default=5000,
        ge=1,
        le=10000,
        description="Maximum allowed input text length",
    )
    max_concurrent_requests: int = Field(
        default=10,
        ge=1,
        le=100,
        description="Maximum concurrent LLM requests",
    )
    log_level: str = Field(
        default="INFO",
        description="Logging level (DEBUG, INFO, WARNING, ERROR)",
    )

    # Server Configuration
    host: str = Field(
        default="0.0.0.0",
        description="Server host address",
    )
    port: int = Field(
        default=8000,
        ge=1,
        le=65535,
        description="Server port",
    )

    # LLM Configuration
    openai_model: str = Field(
        default="gpt-3.5-turbo",
        description="OpenAI model to use for processing",
    )
    openai_timeout: int = Field(
        default=30,
        ge=5,
        le=120,
        description="OpenAI API timeout in seconds",
    )

    # RAG Configuration (Qdrant)
    qdrant_url: str = Field(
        default="http://localhost:6333",
        description="Qdrant vector database URL",
    )
    qdrant_api_key: str | None = Field(
        default=None,
        description="Qdrant API key (optional for local)",
    )
    qdrant_collection: str = Field(
        default="physical_ai_docs",
        description="Qdrant collection name for RAG retrieval",
    )
    qdrant_timeout: int = Field(
        default=10,
        ge=1,
        le=60,
        description="Qdrant connection timeout in seconds",
    )
    rag_enabled: bool = Field(
        default=False,
        description="Enable RAG context retrieval for clarification",
    )
    rag_top_k: int = Field(
        default=5,
        ge=1,
        le=20,
        description="Number of documents to retrieve from Qdrant",
    )
    rag_similarity_threshold: float = Field(
        default=0.4,
        ge=0.0,
        le=1.0,
        description="Minimum similarity score for retrieved documents",
    )

    # Application metadata
    app_name: str = Field(
        default="Text Clarifier and Translator",
        description="Application name",
    )
    app_version: str = Field(
        default="1.0.0",
        description="Application version",
    )

    class Config:
        """Pydantic settings configuration."""

        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False
        extra = "ignore"  # Ignore extra env vars from other projects


@lru_cache
def get_settings() -> Settings:
    """Get cached application settings.

    Returns:
        Settings instance with configuration values
    """
    return Settings()


def is_openai_configured() -> bool:
    """Check if OpenAI API is properly configured.

    Returns:
        True if API key is set, False otherwise
    """
    settings = get_settings()
    return bool(settings.openai_api_key and settings.openai_api_key.startswith("sk-"))


def is_rag_enabled() -> bool:
    """Check if RAG is properly configured and enabled.

    Returns:
        True if RAG is enabled and configured, False otherwise
    """
    settings = get_settings()
    return settings.rag_enabled and bool(settings.qdrant_url)
