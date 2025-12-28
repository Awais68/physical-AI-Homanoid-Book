"""Integration tests for full combined workflow (clarify + translate)."""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from httpx import AsyncClient


class TestCombinedWorkflow:
    """Integration tests for the complete clarify + translate workflow."""

    @pytest.mark.asyncio
    async def test_full_workflow_verbose_to_translated(
        self,
        async_client: AsyncClient,
    ) -> None:
        """Test complete flow: verbose English → clarified → translated Spanish."""
        verbose_input = (
            "I would like to take this opportunity to inform you that "
            "your application has been received and is currently being processed."
        )
        clarified_text = "Your application has been received and is being processed."
        spanish_output = "Su solicitud ha sido recibida y esta siendo procesada."

        request_data = {
            "user_text": verbose_input,
            "target_language": "es",
        }

        mock_clarify_response = MagicMock()
        mock_clarify_response.choices = [
            MagicMock(message=MagicMock(content=clarified_text))
        ]

        mock_translate_response = MagicMock()
        mock_translate_response.choices = [
            MagicMock(message=MagicMock(content=spanish_output))
        ]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
            with patch("src.text_processor.translator.AsyncOpenAI") as mock_translator:
                mock_clarifier_client = MagicMock()
                mock_clarifier_client.chat.completions.create = AsyncMock(
                    return_value=mock_clarify_response
                )
                mock_clarifier.return_value = mock_clarifier_client

                mock_translator_client = MagicMock()
                mock_translator_client.chat.completions.create = AsyncMock(
                    return_value=mock_translate_response
                )
                mock_translator.return_value = mock_translator_client

                response = await async_client.post("/process", json=request_data)

        assert response.status_code == 200
        data = response.json()
        assert data["original_text"] == verbose_input
        # Output should be shorter than the verbose input
        assert len(data["translated_text"]) < len(verbose_input)

    @pytest.mark.asyncio
    async def test_workflow_with_same_source_and_target_language(
        self,
        async_client: AsyncClient,
    ) -> None:
        """Test that same source/target language still works (clarify only)."""
        english_text = "This is a test."

        request_data = {
            "user_text": english_text,
            "target_language": "en",
        }

        mock_response = MagicMock()
        mock_response.choices = [
            MagicMock(message=MagicMock(content=english_text))
        ]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(return_value=mock_response)
            mock_clarifier.return_value = mock_client

            response = await async_client.post("/process", json=request_data)

        assert response.status_code == 200
        data = response.json()
        assert data["original_text"] == english_text

    @pytest.mark.asyncio
    async def test_workflow_preserves_json_structure(
        self,
        async_client: AsyncClient,
        verbose_text: str,
    ) -> None:
        """Test that JSON structure is always preserved."""
        languages = ["en", "es", "fr", "de", "zh"]

        mock_response = MagicMock()
        mock_response.choices = [
            MagicMock(message=MagicMock(content="Processed output"))
        ]

        for lang in languages:
            request_data = {
                "user_text": verbose_text,
                "target_language": lang,
            }

            with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
                with patch(
                    "src.text_processor.translator.AsyncOpenAI"
                ) as mock_translator:
                    mock_client = MagicMock()
                    mock_client.chat.completions.create = AsyncMock(
                        return_value=mock_response
                    )
                    mock_clarifier.return_value = mock_client
                    mock_translator.return_value = mock_client

                    response = await async_client.post("/process", json=request_data)

            assert response.status_code == 200
            data = response.json()
            assert "original_text" in data
            assert "translated_text" in data


class TestConcurrentRequests:
    """Tests for handling concurrent requests."""

    @pytest.mark.asyncio
    async def test_concurrent_requests_handled(
        self,
        async_client: AsyncClient,
        verbose_text: str,
    ) -> None:
        """Test that multiple concurrent requests are handled correctly."""
        import asyncio

        request_data = {
            "user_text": verbose_text,
            "target_language": "es",
        }

        mock_response = MagicMock()
        mock_response.choices = [
            MagicMock(message=MagicMock(content="Processed"))
        ]

        async def make_request() -> int:
            with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
                with patch(
                    "src.text_processor.translator.AsyncOpenAI"
                ) as mock_translator:
                    mock_client = MagicMock()
                    mock_client.chat.completions.create = AsyncMock(
                        return_value=mock_response
                    )
                    mock_clarifier.return_value = mock_client
                    mock_translator.return_value = mock_client

                    response = await async_client.post("/process", json=request_data)
                    return response.status_code

        # Make 5 concurrent requests
        tasks = [make_request() for _ in range(5)]
        results = await asyncio.gather(*tasks)

        # All should succeed
        assert all(status == 200 for status in results)


class TestMetricsLogging:
    """Tests for metrics and logging in combined workflow."""

    @pytest.mark.asyncio
    async def test_request_id_in_response_header(
        self,
        async_client: AsyncClient,
        sample_request_data: dict[str, str],
    ) -> None:
        """Test that request ID is included in response headers."""
        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content="Output"))]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(return_value=mock_response)
            mock_clarifier.return_value = mock_client

            response = await async_client.post("/process", json=sample_request_data)

        assert "x-request-id" in response.headers
        # Request ID should be a valid UUID-like string
        request_id = response.headers["x-request-id"]
        assert len(request_id) > 0
        assert "-" in request_id  # UUID format

    @pytest.mark.asyncio
    async def test_process_time_tracked(
        self,
        async_client: AsyncClient,
        sample_request_data: dict[str, str],
    ) -> None:
        """Test that processing time is tracked and reported."""
        mock_response = MagicMock()
        mock_response.choices = [MagicMock(message=MagicMock(content="Output"))]

        with patch("src.text_processor.clarifier.AsyncOpenAI") as mock_clarifier:
            mock_client = MagicMock()
            mock_client.chat.completions.create = AsyncMock(return_value=mock_response)
            mock_clarifier.return_value = mock_client

            response = await async_client.post("/process", json=sample_request_data)

        assert "x-process-time-ms" in response.headers
        process_time = float(response.headers["x-process-time-ms"])
        assert process_time >= 0  # Should be a positive number
