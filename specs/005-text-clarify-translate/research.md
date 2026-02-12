# Research: AI Text Clarifier and Translator

**Feature**: 005-text-clarify-translate
**Date**: 2025-12-21

## Research Tasks

### 1. LLM Provider Selection

**Decision**: OpenAI API (GPT-4/GPT-3.5-turbo)

**Rationale**:
- Industry-standard API with excellent multilingual support
- Well-documented SDK for Python (`openai` package)
- Supports both text clarification and translation in single prompt
- Robust rate limiting and error handling built-in
- Cost-effective with tiered pricing

**Alternatives Considered**:
| Alternative | Pros | Cons | Why Rejected |
|-------------|------|------|--------------|
| Anthropic Claude | Excellent reasoning, long context | Newer API, less translation benchmarks | OpenAI has more established multilingual benchmarks |
| Google Gemini | Strong multilingual, generous free tier | API still maturing | Less established ecosystem for production |
| Local LLM (Ollama) | No API costs, privacy | Requires GPU, variable quality | Doesn't meet <3s response time for quality models |
| Cohere | Good for enterprise | Limited language support | Fewer supported languages than OpenAI |

### 2. API Framework Selection

**Decision**: FastAPI with Pydantic v2

**Rationale**:
- Native async support for concurrent LLM API calls
- Automatic OpenAPI documentation generation
- Built-in request validation via Pydantic
- High performance with uvicorn ASGI server
- Strong typing support for JSON schema compliance

**Alternatives Considered**:
| Alternative | Pros | Cons | Why Rejected |
|-------------|------|------|--------------|
| Flask | Simple, well-known | No native async, manual validation | Performance bottleneck for concurrent requests |
| Django REST Framework | Full-featured, ORM | Overkill for stateless service | Unnecessary complexity for simple API |
| Starlette | Lightweight, async | Less validation tooling | FastAPI provides better DX with same base |

### 3. Language Detection Strategy

**Decision**: Use LLM for language detection (combined with processing)

**Rationale**:
- LLM can detect source language as part of clarification prompt
- No additional API calls or dependencies
- Handles mixed-language text naturally
- More accurate for context-dependent detection

**Alternatives Considered**:
| Alternative | Pros | Cons | Why Rejected |
|-------------|------|------|--------------|
| langdetect library | Fast, offline | Limited accuracy for short text | Unreliable for mixed-language content |
| Google Cloud Translation API | Very accurate | Additional API cost/dependency | Adds latency and complexity |
| Separate detection step | Explicit control | Extra API call overhead | Combined approach is more efficient |

### 4. Rate Limiting and Concurrency

**Decision**: Application-level rate limiting with semaphore for LLM calls

**Rationale**:
- Prevent LLM API quota exhaustion
- Control concurrent request processing
- Provide clear error messages when limits reached
- Simple implementation without external dependencies

**Implementation Approach**:
- Use `asyncio.Semaphore` for concurrent LLM calls (max 10)
- Implement token bucket for per-client rate limiting (optional)
- Return 429 status with retry-after header when limited

### 5. Error Handling Strategy

**Decision**: Structured error responses with JSON schema

**Rationale**:
- Consistent error format for all failure scenarios
- Actionable error messages per FR-005, FR-008
- Includes error type, message, and optional details
- HTTP status codes aligned with error types

**Error Categories**:
| HTTP Status | Error Type | Scenarios |
|-------------|------------|-----------|
| 400 | VALIDATION_ERROR | Empty text, exceeds 5000 chars, invalid JSON |
| 400 | UNSUPPORTED_LANGUAGE | Target language not in supported list |
| 500 | PROCESSING_ERROR | LLM API failure, timeout |
| 503 | SERVICE_UNAVAILABLE | LLM provider down, rate limited |

### 6. Supported Languages

**Decision**: 12 languages at launch, expandable

**Supported Languages** (FR-009):
| Code | Language | ISO 639-1 |
|------|----------|-----------|
| en | English | en |
| es | Spanish | es |
| fr | French | fr |
| de | German | de |
| zh | Chinese (Simplified) | zh |
| ja | Japanese | ja |
| ar | Arabic | ar |
| hi | Hindi | hi |
| pt | Portuguese | pt |
| ru | Russian | ru |
| ko | Korean | ko |
| it | Italian | it |

**Rationale**: Covers 8 required languages plus 4 additional high-demand languages. All well-supported by GPT-4.

### 7. Prompt Engineering Approach

**Decision**: Two-phase prompt with structured output

**Clarification Prompt Template**:
```text
Analyze the following text and provide a clearer, more concise version.
- Fix grammatical errors
- Remove redundancy
- Preserve the original meaning exactly
- Do not add new information
- Maintain the tone (formal/informal)

Text: {user_text}
```

**Translation Prompt Template**:
```text
Translate the following text to {target_language}.
- Use natural, idiomatic phrasing
- Preserve the original meaning
- Do not add or remove information
- Maintain formatting (paragraphs, lists)

Text: {clarified_text}
```

**Combined Prompt** (for efficiency):
```text
Process the following text in two steps:
1. Clarify: Make it clearer and more concise while preserving meaning
2. Translate: Convert to {target_language} using natural phrasing

Return JSON: {"clarified_text": "...", "translated_text": "..."}

Text: {user_text}
```

### 8. Testing Strategy

**Decision**: Three-tier testing pyramid

**Unit Tests** (text_processor/):
- Input validation (empty, length, encoding)
- Language code validation
- Model serialization/deserialization
- Mock LLM responses

**Integration Tests** (api/):
- Full API endpoint tests with mock LLM
- Error handling scenarios
- Rate limiting behavior
- Concurrent request handling

**Contract Tests**:
- JSON request schema validation
- JSON response schema validation
- OpenAPI spec compliance

## Resolved Clarifications

All technical decisions have been resolved. No outstanding NEEDS CLARIFICATION items.

## Dependencies Summary

```text
# requirements.txt
fastapi>=0.109.0
pydantic>=2.5.0
uvicorn[standard]>=0.27.0
openai>=1.10.0
httpx>=0.26.0
python-dotenv>=1.0.0

# dev dependencies
pytest>=8.0.0
pytest-asyncio>=0.23.0
pytest-cov>=4.1.0
```

## Configuration Requirements

**Environment Variables**:
- `OPENAI_API_KEY`: Required for LLM API access
- `MAX_INPUT_LENGTH`: Optional, defaults to 5000
- `MAX_CONCURRENT_REQUESTS`: Optional, defaults to 10
- `LOG_LEVEL`: Optional, defaults to INFO
