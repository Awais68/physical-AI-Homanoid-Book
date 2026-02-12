# Quickstart: AI Text Clarifier and Translator

**Feature**: 005-text-clarify-translate
**Date**: 2025-12-21

## Overview

This guide covers common integration scenarios for the Text Clarifier and Translator API.

## Prerequisites

- Python 3.11+
- OpenAI API key
- HTTP client (curl, httpx, requests)

## Setup

### 1. Install Dependencies

```bash
pip install fastapi uvicorn openai pydantic python-dotenv httpx
```

### 2. Configure Environment

Create `.env` file:
```env
OPENAI_API_KEY=sk-your-api-key-here
MAX_INPUT_LENGTH=5000
MAX_CONCURRENT_REQUESTS=10
LOG_LEVEL=INFO
```

### 3. Start Server

```bash
uvicorn src.api.main:app --host 0.0.0.0 --port 8000
```

## Integration Scenarios

### Scenario 1: Basic Text Clarification (English)

**Use Case**: Clean up verbose or unclear text

```bash
curl -X POST http://localhost:8000/process \
  -H "Content-Type: application/json" \
  -d '{
    "user_text": "I wanted to let you know that I think that maybe we should consider possibly rescheduling the meeting that we had planned.",
    "target_language": "en"
  }'
```

**Expected Response**:
```json
{
  "original_text": "I wanted to let you know that I think that maybe we should consider possibly rescheduling the meeting that we had planned.",
  "translated_text": "We should consider rescheduling the planned meeting."
}
```

### Scenario 2: Clarify and Translate to Spanish

**Use Case**: Process English text and translate to Spanish

```bash
curl -X POST http://localhost:8000/process \
  -H "Content-Type: application/json" \
  -d '{
    "user_text": "Please be advised that your appointment has been confirmed for next week.",
    "target_language": "es"
  }'
```

**Expected Response**:
```json
{
  "original_text": "Please be advised that your appointment has been confirmed for next week.",
  "translated_text": "Su cita ha sido confirmada para la proxima semana."
}
```

### Scenario 3: Multi-language Translation

**Use Case**: Translate to multiple languages from same source

```python
import httpx
import asyncio

async def translate_to_multiple_languages(text: str, languages: list[str]):
    async with httpx.AsyncClient() as client:
        tasks = [
            client.post(
                "http://localhost:8000/process",
                json={"user_text": text, "target_language": lang}
            )
            for lang in languages
        ]
        responses = await asyncio.gather(*tasks)
        return {
            lang: resp.json()["translated_text"]
            for lang, resp in zip(languages, responses)
        }

# Usage
text = "Welcome to our platform. We are happy to have you here."
languages = ["es", "fr", "de", "ja"]
results = asyncio.run(translate_to_multiple_languages(text, languages))
```

### Scenario 4: Error Handling

**Use Case**: Handle validation and processing errors

```python
import httpx

def process_text_safely(text: str, target_language: str = "en"):
    try:
        response = httpx.post(
            "http://localhost:8000/process",
            json={"user_text": text, "target_language": target_language},
            timeout=10.0
        )

        if response.status_code == 200:
            return {"success": True, "data": response.json()}

        error = response.json()
        if error["error_type"] == "VALIDATION_ERROR":
            return {"success": False, "error": f"Invalid input: {error['message']}"}
        elif error["error_type"] == "UNSUPPORTED_LANGUAGE":
            supported = error.get("details", {}).get("supported_languages", [])
            return {"success": False, "error": f"Use one of: {supported}"}
        else:
            return {"success": False, "error": error["message"]}

    except httpx.TimeoutException:
        return {"success": False, "error": "Request timed out"}
    except httpx.RequestError as e:
        return {"success": False, "error": f"Connection error: {e}"}

# Usage
result = process_text_safely("Hello world", "es")
if result["success"]:
    print(result["data"]["translated_text"])
else:
    print(f"Error: {result['error']}")
```

### Scenario 5: Get Supported Languages

**Use Case**: Display available languages to users

```bash
curl http://localhost:8000/languages
```

**Expected Response**:
```json
{
  "languages": [
    {"code": "en", "name": "English", "native_name": "English"},
    {"code": "es", "name": "Spanish", "native_name": "Espanol"},
    {"code": "fr", "name": "French", "native_name": "Francais"},
    {"code": "de", "name": "German", "native_name": "Deutsch"},
    {"code": "zh", "name": "Chinese", "native_name": "Chinese"},
    {"code": "ja", "name": "Japanese", "native_name": "Japanese"},
    {"code": "ar", "name": "Arabic", "native_name": "Arabic"},
    {"code": "hi", "name": "Hindi", "native_name": "Hindi"},
    {"code": "pt", "name": "Portuguese", "native_name": "Portugues"},
    {"code": "ru", "name": "Russian", "native_name": "Russian"},
    {"code": "ko", "name": "Korean", "native_name": "Korean"},
    {"code": "it", "name": "Italian", "native_name": "Italiano"}
  ]
}
```

### Scenario 6: Health Check for Monitoring

**Use Case**: Monitor service availability

```bash
curl http://localhost:8000/health
```

**Expected Response**:
```json
{
  "status": "healthy",
  "version": "1.0.0"
}
```

## Testing Contracts

### Verify Request Schema

```python
import pytest
from pydantic import ValidationError
from src.text_processor.models import TextProcessRequest

def test_valid_request():
    request = TextProcessRequest(
        user_text="Hello world",
        target_language="es"
    )
    assert request.user_text == "Hello world"
    assert request.target_language == "es"

def test_empty_text_rejected():
    with pytest.raises(ValidationError):
        TextProcessRequest(user_text="", target_language="en")

def test_exceeds_max_length():
    with pytest.raises(ValidationError):
        TextProcessRequest(user_text="x" * 5001, target_language="en")
```

### Verify Response Schema

```python
def test_response_has_required_fields():
    response = httpx.post(
        "http://localhost:8000/process",
        json={"user_text": "Test", "target_language": "en"}
    )
    data = response.json()
    assert "original_text" in data
    assert "translated_text" in data
    assert data["original_text"] == "Test"
```

## Performance Testing

```python
import asyncio
import httpx
import time

async def benchmark_concurrent_requests(num_requests: int = 100):
    async with httpx.AsyncClient() as client:
        start = time.time()
        tasks = [
            client.post(
                "http://localhost:8000/process",
                json={"user_text": f"Test message {i}", "target_language": "es"}
            )
            for i in range(num_requests)
        ]
        responses = await asyncio.gather(*tasks)
        elapsed = time.time() - start

        success_count = sum(1 for r in responses if r.status_code == 200)
        print(f"Completed {num_requests} requests in {elapsed:.2f}s")
        print(f"Success rate: {success_count}/{num_requests}")
        print(f"Requests/sec: {num_requests/elapsed:.2f}")

asyncio.run(benchmark_concurrent_requests(100))
```

## Docker Deployment

```dockerfile
FROM python:3.11-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY src/ ./src/
EXPOSE 8000

CMD ["uvicorn", "src.api.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

```bash
docker build -t text-clarify-translate .
docker run -p 8000:8000 -e OPENAI_API_KEY=$OPENAI_API_KEY text-clarify-translate
```
