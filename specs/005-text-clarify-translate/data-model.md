# Data Model: AI Text Clarifier and Translator

**Feature**: 005-text-clarify-translate
**Date**: 2025-12-21

## Overview

This document defines the data structures for the text clarification and translation service. All models use Pydantic for validation and serialization.

## Entities

### 1. TextProcessRequest

**Purpose**: Incoming request payload from client

```python
class TextProcessRequest(BaseModel):
    """Request to process text for clarification and/or translation."""

    user_text: str
    target_language: str | None = "en"
```

**Fields**:
| Field | Type | Required | Constraints | Description |
|-------|------|----------|-------------|-------------|
| user_text | string | Yes | 1-5000 chars, UTF-8 | The text to clarify and/or translate |
| target_language | string | No | Valid language code | Target language (defaults to "en") |

**Validation Rules**:
- `user_text` must not be empty or whitespace-only
- `user_text` must not exceed 5000 characters (FR-010)
- `target_language` must be a supported language code

### 2. TextProcessResponse

**Purpose**: Successful processing result returned to client

```python
class TextProcessResponse(BaseModel):
    """Response containing original and processed text."""

    original_text: str
    translated_text: str
```

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| original_text | string | Yes | The original user input (unchanged) |
| translated_text | string | Yes | Clarified and translated text |

**Invariants**:
- `original_text` always matches `user_text` from request
- `translated_text` is never empty for successful processing
- Response is valid JSON (FR-005)

### 3. ErrorResponse

**Purpose**: Structured error information for failed requests

```python
class ErrorResponse(BaseModel):
    """Error response for failed processing."""

    error_type: ErrorType
    message: str
    details: dict | None = None
```

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| error_type | ErrorType enum | Yes | Category of error |
| message | string | Yes | Human-readable error description |
| details | object | No | Additional context (e.g., supported languages) |

### 4. ErrorType (Enum)

**Purpose**: Categorize error types for consistent handling

```python
class ErrorType(str, Enum):
    """Types of errors that can occur during processing."""

    VALIDATION_ERROR = "VALIDATION_ERROR"
    UNSUPPORTED_LANGUAGE = "UNSUPPORTED_LANGUAGE"
    PROCESSING_ERROR = "PROCESSING_ERROR"
    SERVICE_UNAVAILABLE = "SERVICE_UNAVAILABLE"
```

**Values**:
| Value | HTTP Status | Description |
|-------|-------------|-------------|
| VALIDATION_ERROR | 400 | Invalid input (empty, too long, bad format) |
| UNSUPPORTED_LANGUAGE | 400 | Target language not supported |
| PROCESSING_ERROR | 500 | LLM processing failed |
| SERVICE_UNAVAILABLE | 503 | Service temporarily unavailable |

### 5. LanguageConfig

**Purpose**: Define supported languages and their properties

```python
class Language(BaseModel):
    """A supported language configuration."""

    code: str
    name: str
    native_name: str
```

**Supported Languages**:
| Code | Name | Native Name |
|------|------|-------------|
| en | English | English |
| es | Spanish | Español |
| fr | French | Français |
| de | German | Deutsch |
| zh | Chinese | 中文 |
| ja | Japanese | 日本語 |
| ar | Arabic | العربية |
| hi | Hindi | हिन्दी |
| pt | Portuguese | Português |
| ru | Russian | Русский |
| ko | Korean | 한국어 |
| it | Italian | Italiano |

### 6. ProcessingMetrics (Internal)

**Purpose**: Track processing for observability (not exposed in API)

```python
class ProcessingMetrics(BaseModel):
    """Internal metrics for request processing."""

    request_id: str
    source_language_detected: str | None
    target_language: str
    input_length: int
    output_length: int
    processing_time_ms: int
    llm_model_used: str
```

## Relationships

```text
TextProcessRequest ──────┐
                         │
                         ▼
                   ┌───────────┐
                   │ Processor │
                   └───────────┘
                         │
            ┌────────────┼────────────┐
            ▼            ▼            ▼
  TextProcessResponse  ErrorResponse  ProcessingMetrics
      (success)         (failure)      (logging)
```

## Validation Rules Summary

### Input Validation (FR-007)

1. **Empty text**: Reject with `VALIDATION_ERROR`, message "Text input cannot be empty"
2. **Whitespace-only**: Reject with `VALIDATION_ERROR`, message "Text input cannot be whitespace only"
3. **Exceeds length**: Reject with `VALIDATION_ERROR`, message "Text exceeds maximum length of 5000 characters"
4. **Invalid encoding**: Reject with `VALIDATION_ERROR`, message "Text must be valid UTF-8"

### Language Validation

1. **Unknown language**: Reject with `UNSUPPORTED_LANGUAGE`, include list of supported languages in details
2. **Case insensitive**: Accept "EN", "en", "En" as equivalent
3. **Full name support**: Accept both "en" and "English" (normalize to code)

## State Transitions

The service is stateless - no state transitions to document. Each request is processed independently.

## JSON Schema Examples

### Success Request/Response

**Request**:
```json
{
  "user_text": "I want to make sure that you understand that this is very important thing.",
  "target_language": "es"
}
```

**Response** (200 OK):
```json
{
  "original_text": "I want to make sure that you understand that this is very important thing.",
  "translated_text": "Quiero asegurarme de que entiendas que esto es muy importante."
}
```

### Error Response

**Request** (empty text):
```json
{
  "user_text": "",
  "target_language": "en"
}
```

**Response** (400 Bad Request):
```json
{
  "error_type": "VALIDATION_ERROR",
  "message": "Text input cannot be empty",
  "details": null
}
```

**Request** (unsupported language):
```json
{
  "user_text": "Hello world",
  "target_language": "xyz"
}
```

**Response** (400 Bad Request):
```json
{
  "error_type": "UNSUPPORTED_LANGUAGE",
  "message": "Language 'xyz' is not supported",
  "details": {
    "supported_languages": ["en", "es", "fr", "de", "zh", "ja", "ar", "hi", "pt", "ru", "ko", "it"]
  }
}
```
