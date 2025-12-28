"""Pytest configuration and fixtures for text processing tests."""

import asyncio
from collections.abc import AsyncGenerator, Generator
from typing import Any
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from httpx import ASGITransport, AsyncClient

from src.api.main import app


@pytest.fixture(scope="session")
def event_loop() -> Generator[asyncio.AbstractEventLoop, None, None]:
    """Create event loop for async tests."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest.fixture
async def async_client() -> AsyncGenerator[AsyncClient, None]:
    """Create async HTTP client for API testing."""
    transport = ASGITransport(app=app)
    async with AsyncClient(transport=transport, base_url="http://test") as client:
        yield client


@pytest.fixture
def mock_openai_response() -> dict[str, Any]:
    """Mock OpenAI API response for clarification."""
    return {
        "id": "chatcmpl-test123",
        "object": "chat.completion",
        "created": 1234567890,
        "model": "gpt-3.5-turbo",
        "choices": [
            {
                "index": 0,
                "message": {
                    "role": "assistant",
                    "content": "This is very important.",
                },
                "finish_reason": "stop",
            }
        ],
        "usage": {
            "prompt_tokens": 50,
            "completion_tokens": 10,
            "total_tokens": 60,
        },
    }


@pytest.fixture
def mock_openai_translation_response() -> dict[str, Any]:
    """Mock OpenAI API response for translation."""
    return {
        "id": "chatcmpl-test456",
        "object": "chat.completion",
        "created": 1234567890,
        "model": "gpt-3.5-turbo",
        "choices": [
            {
                "index": 0,
                "message": {
                    "role": "assistant",
                    "content": "Esto es muy importante.",
                },
                "finish_reason": "stop",
            }
        ],
        "usage": {
            "prompt_tokens": 50,
            "completion_tokens": 10,
            "total_tokens": 60,
        },
    }


@pytest.fixture
def mock_openai_client(mock_openai_response: dict[str, Any]) -> MagicMock:
    """Create mock OpenAI client."""
    mock_client = MagicMock()
    mock_completion = MagicMock()
    mock_completion.choices = [
        MagicMock(message=MagicMock(content=mock_openai_response["choices"][0]["message"]["content"]))
    ]

    mock_client.chat.completions.create = AsyncMock(return_value=mock_completion)
    return mock_client


@pytest.fixture
def patch_openai(mock_openai_client: MagicMock) -> Generator[MagicMock, None, None]:
    """Patch OpenAI client for tests."""
    with patch("openai.AsyncOpenAI", return_value=mock_openai_client):
        yield mock_openai_client


# Sample test data fixtures
@pytest.fixture
def verbose_text() -> str:
    """Sample verbose text for clarification testing."""
    return "I want to make sure that you understand that this is very important thing."


@pytest.fixture
def clear_text() -> str:
    """Expected clarified version of verbose text."""
    return "This is very important."


@pytest.fixture
def spanish_translation() -> str:
    """Expected Spanish translation."""
    return "Esto es muy importante."


@pytest.fixture
def sample_request_data(verbose_text: str) -> dict[str, str]:
    """Sample request data for API tests."""
    return {
        "user_text": verbose_text,
        "target_language": "en",
    }


@pytest.fixture
def sample_translation_request(verbose_text: str) -> dict[str, str]:
    """Sample translation request data."""
    return {
        "user_text": verbose_text,
        "target_language": "es",
    }


# Test data for validation tests
@pytest.fixture
def empty_text_request() -> dict[str, str]:
    """Request with empty text."""
    return {
        "user_text": "",
        "target_language": "en",
    }


@pytest.fixture
def whitespace_text_request() -> dict[str, str]:
    """Request with whitespace-only text."""
    return {
        "user_text": "   \n\t   ",
        "target_language": "en",
    }


@pytest.fixture
def too_long_text_request() -> dict[str, str]:
    """Request with text exceeding max length."""
    return {
        "user_text": "x" * 5001,
        "target_language": "en",
    }


@pytest.fixture
def unsupported_language_request(verbose_text: str) -> dict[str, str]:
    """Request with unsupported language."""
    return {
        "user_text": verbose_text,
        "target_language": "xyz",
    }
