# Implementation Plan: AI Text Clarifier and Translator

**Branch**: `005-text-clarify-translate` | **Date**: 2025-12-21 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-text-clarify-translate/spec.md`

## Summary

Build an AI-powered text processing service that clarifies/specifies unclear text and translates it to target languages, returning structured JSON output with both original and processed text. The system uses an LLM backend (OpenAI SDK) for natural language processing with a REST API interface for client integration, and integrates with the existing RAG chatbot (qdrant-rag-responder) to enhance clarification with domain-specific knowledge from the Physical AI documentation stored in Qdrant vector database.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: FastAPI (web framework), Pydantic (validation), OpenAI SDK (LLM integration using gpt-4o-mini), qdrant-client (vector database for RAG), uvicorn (ASGI server)
**Storage**: Qdrant vector database (read-only access for RAG context retrieval)
**Testing**: pytest, pytest-asyncio, httpx (async client testing)
**Target Platform**: Linux server / Docker container / Cloud deployment
**Project Type**: single (API service)
**Performance Goals**: <3s response time for <1000 char input, 100 concurrent requests
**Constraints**: 5,000 character max input, UTF-8 encoding, JSON response format
**Scale/Scope**: Stateless microservice supporting 8+ languages

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Test-First | PASS | Unit tests for text processing, integration tests for API endpoints, contract tests for JSON schema |
| II. Library-First Design | PASS | Core text processing logic as standalone module (`text_processor/`), API layer separate |
| III. CLI Interface | PASS | CLI interface MANDATORY per Constitution; provides stdin/args → stdout, errors → stderr with JSON + human-readable formats |
| IV. Integration Testing | PASS | Integration tests for LLM API calls, API endpoint tests with mock LLM responses |
| V. Observability | PASS | Structured logging for all requests/responses, error tracking with request IDs |
| VI. Graceful Degradation | PASS | Handle LLM API failures with error responses, validate input before processing |

**Quality Gates**:
- Unit test coverage: 80% minimum
- All API endpoints documented via OpenAPI
- Contract tests for JSON request/response schemas

## Project Structure

### Documentation (this feature)

```text
specs/005-text-clarify-translate/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (OpenAPI spec)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
src/
├── text_processor/          # Core library (Library-First)
│   ├── __init__.py
│   ├── clarifier.py         # Text clarification logic
│   ├── translator.py        # Translation logic
│   ├── rag_client.py       # RAG integration (Qdrant, context retrieval)
│   ├── language_config.py   # Supported languages config
│   └── models.py            # Pydantic models
├── api/                     # FastAPI application
│   ├── __init__.py
│   ├── main.py              # FastAPI app entry point
│   ├── routes.py            # API routes
│   └── middleware.py        # Logging, error handling
├── cli/                     # CLI interface (MANDATORY - Constitution III)
│   ├── __init__.py
│   ├── main.py              # CLI entry point - stdin/args → stdout, errors → stderr
│   └── formatters.py        # JSON + human-readable output formatters
└── config.py                # App configuration

tests/
├── unit/
│   ├── test_clarifier.py
│   ├── test_translator.py
│   ├── test_rag_client.py   # RAG client tests
│   └── test_models.py
├── integration/
│   ├── test_api.py
│   ├── test_llm_integration.py
│   ├── test_cli.py          # CLI integration tests (stdin/args validation)
│   └── test_rag_integration.py  # RAG integration tests
└── contract/
    ├── test_json_schema.py
    └── test_rag_clarify.py # RAG-enhanced clarification contract tests
```

**Structure Decision**: Single project structure selected. Core text processing logic isolated in `text_processor/` module following Library-First principle, with separate API layer for web interface and MANDATORY CLI per Constitution Principle III (stdin/args → stdout protocol).

## Complexity Tracking

> No violations - all constitution principles satisfied with standard architecture.
