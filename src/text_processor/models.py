"""Pydantic models for text processing service."""

from enum import Enum
from typing import Any

from pydantic import BaseModel, Field, field_validator


class ErrorType(str, Enum):
    """Types of errors that can occur during processing."""

    VALIDATION_ERROR = "VALIDATION_ERROR"
    UNSUPPORTED_LANGUAGE = "UNSUPPORTED_LANGUAGE"
    PROCESSING_ERROR = "PROCESSING_ERROR"
    SERVICE_UNAVAILABLE = "SERVICE_UNAVAILABLE"


class Language(BaseModel):
    """A supported language configuration."""

    code: str = Field(
        ...,
        description="ISO 639-1 language code",
    )
    name: str = Field(
        ...,
        description="English name of language",
    )
    native_name: str = Field(
        ...,
        description="Native name of language",
    )


class LanguageListResponse(BaseModel):
    """Response containing list of supported languages."""

    languages: list[Language] = Field(
        ...,
        description="List of supported languages",
    )


class HealthStatus(str, Enum):
    """Service health status values."""

    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNHEALTHY = "unhealthy"


class HealthResponse(BaseModel):
    """Health check response."""

    status: HealthStatus = Field(
        ...,
        description="Service health status",
    )
    version: str = Field(
        ...,
        description="Service version",
    )


class Citation(BaseModel):
    """Reference to a source document used in RAG retrieval."""

    source_index: int = Field(..., description="Index of the source in retrieved list")
    title: str = Field(..., description="Document title")
    source_url: str | None = Field(
        default=None, description="URL to source document"
    )
    file_path: str | None = Field(
        default=None, description="File path of source document"
    )
    snippet: str = Field(
        ..., description="Relevant text snippet from document"
    )
    relevance_score: float = Field(
        ..., ge=0.0, le=1.0, description="Relevance score (0-1)"
    )


class RAGContext(BaseModel):
    """Retrieved context from Qdrant vector database."""

    query: str = Field(..., description="Original query used for retrieval")
    documents: list[Citation] = Field(
        default_factory=list, description="Retrieved documents with metadata"
    )
    retrieval_time_ms: int = Field(
        default=0, description="Time spent on retrieval in milliseconds"
    )
    formatted_context: str = Field(
        default="", description="Formatted context for LLM prompt"
    )


class TextProcessRequest(BaseModel):
    """Request to process text for clarification and/or translation."""

    user_text: str = Field(
        ...,
        min_length=1,
        max_length=5000,
        description="The text to clarify and/or translate",
    )
    target_language: str = Field(
        default="en",
        description="Target language code (ISO 639-1) or full name",
    )
    use_rag: bool = Field(
        default=False,
        description="Enable RAG context retrieval for clarification",
    )

    @field_validator("user_text")
    @classmethod
    def validate_not_whitespace(cls, v: str) -> str:
        """Validate that text is not whitespace-only."""
        if not v.strip():
            raise ValueError("Text input cannot be whitespace only")
        return v


class TextProcessResponse(BaseModel):
    """Response containing original and processed text."""

    original_text: str = Field(
        ...,
        description="The original user input (unchanged)",
    )
    translated_text: str = Field(
        ...,
        description="Clarified and translated text",
    )
    rag_context: RAGContext | None = Field(
        default=None, description="RAG context if enabled and retrieved"
    )
    citations: list[Citation] = Field(
        default_factory=list, description="Citations from RAG retrieval"
    )


class ErrorResponse(BaseModel):
    """Error response for failed processing."""

    error_type: ErrorType = Field(
        ...,
        description="Category of error",
    )
    message: str = Field(
        ...,
        description="Human-readable error description",
    )
    details: dict[str, Any] | None = Field(
        default=None,
        description="Additional error context",
    )


class ProcessingMetrics(BaseModel):
    """Internal metrics for request processing (not exposed in API)."""

    request_id: str = Field(..., description="Unique request identifier")
    source_language_detected: str | None = Field(
        default=None, description="Detected source language"
    )
    target_language: str = Field(..., description="Target language for translation")
    input_length: int = Field(..., description="Length of input text")
    output_length: int = Field(..., description="Length of output text")
    processing_time_ms: int = Field(..., description="Processing time in milliseconds")
    llm_model_used: str = Field(..., description="LLM model used for processing")
    rag_enabled: bool = Field(
        default=False, description="Whether RAG was enabled for this request"
    )
    rag_documents_retrieved: int = Field(
        default=0, description="Number of documents retrieved from RAG"
    )
    rag_retrieval_time_ms: int = Field(
        default=0, description="Time spent on RAG retrieval in milliseconds"
    )
