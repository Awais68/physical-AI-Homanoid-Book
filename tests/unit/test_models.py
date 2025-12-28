"""Unit tests for Pydantic models."""

import pytest
from pydantic import ValidationError

from src.text_processor.models import (
    ErrorResponse,
    ErrorType,
    HealthResponse,
    HealthStatus,
    Language,
    LanguageListResponse,
    ProcessingMetrics,
    TextProcessRequest,
    TextProcessResponse,
)


class TestTextProcessRequest:
    """Tests for TextProcessRequest model."""

    def test_valid_request(self) -> None:
        """Test creating a valid request."""
        request = TextProcessRequest(
            user_text="Hello world",
            target_language="es",
        )
        assert request.user_text == "Hello world"
        assert request.target_language == "es"

    def test_default_language_is_english(self) -> None:
        """Test that default target_language is English."""
        request = TextProcessRequest(user_text="Hello")
        assert request.target_language == "en"

    def test_empty_text_rejected(self) -> None:
        """Test that empty text raises validation error."""
        with pytest.raises(ValidationError):
            TextProcessRequest(user_text="", target_language="en")

    def test_whitespace_only_rejected(self) -> None:
        """Test that whitespace-only text raises validation error."""
        with pytest.raises(ValidationError):
            TextProcessRequest(user_text="   \n\t   ", target_language="en")

    def test_max_length_enforced(self) -> None:
        """Test that text exceeding max length is rejected."""
        with pytest.raises(ValidationError):
            TextProcessRequest(user_text="x" * 5001, target_language="en")

    def test_max_length_allowed(self) -> None:
        """Test that text at max length is accepted."""
        request = TextProcessRequest(user_text="x" * 5000, target_language="en")
        assert len(request.user_text) == 5000


class TestTextProcessResponse:
    """Tests for TextProcessResponse model."""

    def test_valid_response(self) -> None:
        """Test creating a valid response."""
        response = TextProcessResponse(
            original_text="Hello",
            translated_text="Hola",
        )
        assert response.original_text == "Hello"
        assert response.translated_text == "Hola"

    def test_serialization_to_json(self) -> None:
        """Test that response serializes to valid JSON."""
        response = TextProcessResponse(
            original_text="Test",
            translated_text="Prueba",
        )
        json_data = response.model_dump()
        assert "original_text" in json_data
        assert "translated_text" in json_data

    def test_response_requires_both_fields(self) -> None:
        """Test that both fields are required."""
        with pytest.raises(ValidationError):
            TextProcessResponse(original_text="Test")  # type: ignore


class TestErrorResponse:
    """Tests for ErrorResponse model."""

    def test_valid_error_response(self) -> None:
        """Test creating a valid error response."""
        error = ErrorResponse(
            error_type=ErrorType.VALIDATION_ERROR,
            message="Invalid input",
        )
        assert error.error_type == ErrorType.VALIDATION_ERROR
        assert error.message == "Invalid input"
        assert error.details is None

    def test_error_with_details(self) -> None:
        """Test error response with details."""
        error = ErrorResponse(
            error_type=ErrorType.UNSUPPORTED_LANGUAGE,
            message="Language not supported",
            details={"supported_languages": ["en", "es"]},
        )
        assert error.details is not None
        assert "supported_languages" in error.details

    def test_all_error_types(self) -> None:
        """Test all error type values."""
        assert ErrorType.VALIDATION_ERROR.value == "VALIDATION_ERROR"
        assert ErrorType.UNSUPPORTED_LANGUAGE.value == "UNSUPPORTED_LANGUAGE"
        assert ErrorType.PROCESSING_ERROR.value == "PROCESSING_ERROR"
        assert ErrorType.SERVICE_UNAVAILABLE.value == "SERVICE_UNAVAILABLE"


class TestLanguage:
    """Tests for Language model."""

    def test_valid_language(self) -> None:
        """Test creating a valid language."""
        lang = Language(
            code="en",
            name="English",
            native_name="English",
        )
        assert lang.code == "en"
        assert lang.name == "English"
        assert lang.native_name == "English"

    def test_language_requires_all_fields(self) -> None:
        """Test that all fields are required."""
        with pytest.raises(ValidationError):
            Language(code="en", name="English")  # type: ignore


class TestLanguageListResponse:
    """Tests for LanguageListResponse model."""

    def test_valid_language_list(self) -> None:
        """Test creating a valid language list response."""
        languages = [
            Language(code="en", name="English", native_name="English"),
            Language(code="es", name="Spanish", native_name="Espanol"),
        ]
        response = LanguageListResponse(languages=languages)
        assert len(response.languages) == 2


class TestHealthResponse:
    """Tests for HealthResponse model."""

    def test_healthy_response(self) -> None:
        """Test creating a healthy response."""
        response = HealthResponse(
            status=HealthStatus.HEALTHY,
            version="1.0.0",
        )
        assert response.status == HealthStatus.HEALTHY
        assert response.version == "1.0.0"

    def test_all_health_statuses(self) -> None:
        """Test all health status values."""
        assert HealthStatus.HEALTHY.value == "healthy"
        assert HealthStatus.DEGRADED.value == "degraded"
        assert HealthStatus.UNHEALTHY.value == "unhealthy"


class TestProcessingMetrics:
    """Tests for ProcessingMetrics model."""

    def test_valid_metrics(self) -> None:
        """Test creating valid processing metrics."""
        metrics = ProcessingMetrics(
            request_id="test-123",
            source_language_detected=None,
            target_language="es",
            input_length=100,
            output_length=80,
            processing_time_ms=1500,
            llm_model_used="gpt-3.5-turbo",
        )
        assert metrics.request_id == "test-123"
        assert metrics.processing_time_ms == 1500

    def test_metrics_serialization(self) -> None:
        """Test that metrics serialize correctly."""
        metrics = ProcessingMetrics(
            request_id="test-456",
            source_language_detected="en",
            target_language="fr",
            input_length=50,
            output_length=60,
            processing_time_ms=2000,
            llm_model_used="gpt-4",
        )
        json_data = metrics.model_dump()
        assert json_data["request_id"] == "test-456"
        assert json_data["source_language_detected"] == "en"
