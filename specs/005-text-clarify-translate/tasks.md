# Tasks: AI Text Clarifier and Translator

**Input**: Design documents from `/specs/005-text-clarify-translate/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Tests are included based on constitution requirement for Test-First development and 80% coverage.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root (per plan.md)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create project structure per implementation plan (src/text_processor/, src/api/, src/cli/, tests/)
- [X] T002 Initialize Python 3.11+ project with pyproject.toml and dependencies (fastapi, pydantic, openai, uvicorn, httpx)
- [X] T003 [P] Create .env.example with required environment variables (OPENAI_API_KEY, MAX_INPUT_LENGTH, etc.)
- [X] T004 [P] Configure pytest and pytest-asyncio in pyproject.toml
- [X] T005 [P] Create .gitignore with Python patterns (venv/, __pycache__/, .env, etc.)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Create Pydantic models in src/text_processor/models.py (TextProcessRequest, TextProcessResponse, ErrorResponse, ErrorType, Language)
- [X] T007 [P] Create language configuration in src/text_processor/language_config.py (12 supported languages with codes, names, native_names)
- [X] T008 [P] Create application config in src/config.py (env vars, settings with defaults)
- [X] T009 Implement error handling middleware in src/api/middleware.py (request ID, structured logging, exception handlers)
- [X] T010 Create FastAPI app entry point in src/api/main.py (app instance, middleware registration, router includes)
- [X] T011 [P] Create base test fixtures in tests/conftest.py (async client, mock LLM responses, test app)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Text Clarification (Priority: P1) - MVP

**Goal**: Users submit unclear/verbose text and receive clear, concise, grammatically correct text (English only)

**Independent Test**: Submit verbose English text → Receive clarified English text preserving original meaning

### Tests for User Story 1

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [X] T012 [P] [US1] Unit test for text clarifier in tests/unit/test_clarifier.py (verbose input → concise output, grammar fix, meaning preservation)
- [X] T013 [P] [US1] Contract test for /process endpoint (clarify-only mode) in tests/contract/test_process_clarify.py
- [X] T014 [P] [US1] Integration test for clarification flow in tests/integration/test_clarification.py

### Implementation for User Story 1

- [X] T015 [US1] Implement text clarifier service in src/text_processor/clarifier.py (LLM prompt for clarification, OpenAI SDK integration)
- [X] T016 [US1] Implement /process endpoint for clarification in src/api/routes.py (POST /process, target_language="en" default)
- [X] T017 [US1] Add input validation in src/text_processor/clarifier.py (empty check, length check, whitespace check)
- [X] T018 [US1] Add structured logging for clarification requests in src/text_processor/clarifier.py

**Checkpoint**: User Story 1 complete - text clarification works independently with English target

---

## Phase 4: User Story 2 - Text Translation (Priority: P2)

**Goal**: Users submit text with target language and receive natural, idiomatic translation

**Independent Test**: Submit English text with target_language="es" → Receive natural Spanish translation

### Tests for User Story 2

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [X] T019 [P] [US2] Unit test for translator in tests/unit/test_translator.py (language detection, translation quality, idiomatic output)
- [X] T020 [P] [US2] Contract test for /process endpoint (translation mode) in tests/contract/test_process_translate.py
- [X] T021 [P] [US2] Integration test for translation flow in tests/integration/test_translation.py

### Implementation for User Story 2

- [X] T022 [US2] Implement translator service in src/text_processor/translator.py (LLM prompt for translation, language code handling)
- [X] T023 [US2] Add language validation in src/text_processor/translator.py (supported language check, case normalization, full name support)
- [X] T024 [US2] Update /process endpoint to support translation in src/api/routes.py (call translator when target_language != source)
- [X] T025 [US2] Implement /languages endpoint in src/api/routes.py (GET /languages, return LanguageListResponse)
- [X] T026 [US2] Add structured logging for translation requests in src/text_processor/translator.py

**Checkpoint**: User Story 2 complete - text translation works for all 12 supported languages

---

## Phase 5: User Story 3 - Combined Clarify and Translate (Priority: P3)

**Goal**: Users submit unclear text + target language → Receive JSON with original_text and clarified/translated text

**Independent Test**: Submit verbose English text with target_language="fr" → Receive JSON with both fields, French clarified output

### Tests for User Story 3

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [X] T027 [P] [US3] Unit test for combined processor in tests/unit/test_processor.py (clarify then translate, JSON schema compliance)
- [X] T028 [P] [US3] Contract test for combined /process endpoint in tests/contract/test_process_combined.py
- [X] T029 [P] [US3] Integration test for full workflow in tests/integration/test_combined_flow.py

### Implementation for User Story 3

- [X] T030 [US3] Create text processor orchestrator in src/text_processor/__init__.py (combine clarifier + translator, async coordination)
- [X] T031 [US3] Update /process endpoint for combined workflow in src/api/routes.py (clarify → translate → JSON response)
- [X] T032 [US3] Add error handling for LLM failures in src/text_processor/__init__.py (retry logic, graceful degradation)
- [X] T033 [US3] Implement rate limiting with semaphore in src/api/middleware.py (MAX_CONCURRENT_REQUESTS)
- [X] T034 [US3] Add ProcessingMetrics logging in src/text_processor/__init__.py (request_id, timing, model used)

**Checkpoint**: User Story 3 complete - full clarify+translate workflow operational with JSON output

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T035 [P] Implement /health endpoint in src/api/routes.py (GET /health, return HealthResponse)
- [X] T036 [P] Add unit tests for models in tests/unit/test_models.py (validation rules, serialization)
- [X] T037 [P] Create CLI interface in src/cli/main.py (argparse, stdin support, JSON output)
- [X] T038 Run all tests and verify 80% coverage minimum
- [X] T039 [P] Create Dockerfile for containerized deployment
- [X] T040 Validate against quickstart.md scenarios (all 6 integration scenarios)
- [X] T041 [P] Add OpenAPI documentation customizations in src/api/main.py (descriptions, examples)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Independent, uses same models
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Integrates US1+US2 but testable independently

### Within Each User Story (TDD Flow)

1. Tests written FIRST and verified to FAIL
2. Implementation to make tests PASS
3. Refactor if needed while tests stay GREEN

### Parallel Opportunities

**Phase 1 (Setup)**:
- T003, T004, T005 can run in parallel

**Phase 2 (Foundational)**:
- T007, T008 can run in parallel (after T006)
- T011 can run in parallel with T009, T010

**Phase 3-5 (User Stories)**:
- All test tasks within a story (T012-T014, T019-T021, T027-T029) can run in parallel
- Different user stories can be worked on in parallel by different team members

**Phase 6 (Polish)**:
- T035, T036, T037, T039, T041 can run in parallel

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together:
Task: "Unit test for text clarifier in tests/unit/test_clarifier.py"
Task: "Contract test for /process endpoint in tests/contract/test_process_clarify.py"
Task: "Integration test for clarification flow in tests/integration/test_clarification.py"

# After tests written and failing, implementation is sequential:
Task: "Implement text clarifier service in src/text_processor/clarifier.py"
Task: "Implement /process endpoint for clarification in src/api/routes.py"
Task: "Add input validation..."
Task: "Add structured logging..."
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T011)
3. Complete Phase 3: User Story 1 (T012-T018)
4. **STOP and VALIDATE**: Test clarification independently
5. Deploy/demo if ready - users can clarify English text

### Incremental Delivery

| Increment | Delivers | Can Demo |
|-----------|----------|----------|
| Setup + Foundational | Infrastructure | Project compiles |
| + User Story 1 | Text clarification (English) | Clarify verbose text |
| + User Story 2 | Multi-language translation | Translate to 12 languages |
| + User Story 3 | Combined workflow + JSON | Full API integration |
| + Polish | Production-ready | Dockerized, documented |

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (clarification)
   - Developer B: User Story 2 (translation)
   - Developer C: User Story 3 tests (prepare integration)
3. Stories complete and integrate independently

---

## Task Summary

| Phase | Tasks | Parallel Tasks |
|-------|-------|----------------|
| Setup | 5 | 3 |
| Foundational | 6 | 3 |
| User Story 1 (P1) | 7 | 3 |
| User Story 2 (P2) | 8 | 3 |
| User Story 3 (P3) | 8 | 3 |
| Polish | 7 | 5 |
| **Total** | **41** | **20** |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story is independently completable and testable
- Verify tests FAIL before implementing (TDD)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Constitution requires Test-First and 80% coverage
