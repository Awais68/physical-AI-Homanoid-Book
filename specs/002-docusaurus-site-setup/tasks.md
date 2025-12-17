# Tasks: Docusaurus Static Site Setup

**Input**: Design documents from `/specs/002-docusaurus-site-setup/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Test tasks are NOT explicitly requested in the feature specification, so they are not included.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- Paths shown below assume single project, matching the Docusaurus structure.

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Initialize Docusaurus project in the root directory (using `npx create-docusaurus@latest . classic --typescript`)
- [ ] T002 Install project dependencies in `frontend/docs-site` (using `npm install` or `yarn install`)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T003 Configure `docusaurus.config.js` for basic site metadata, theme, and plugins: `docusaurus.config.js`
- [ ] T004 Setup `sidebars.js` for initial content structure: `sidebars.js`
- [ ] T005 [P] Implement `src/css/custom.css` for initial custom styling: `src/css/custom.css`
- [ ] T006 Configure dark/light theme support in `docusaurus.config.js`: `docusaurus.config.js`
- [ ] T007 Add basic homepage content: `src/pages/index.js` (or `src/pages/index.mdx`)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Create and Publish Educational Content (Priority: P1) 🎯 MVP

**Goal**: As an educator, create and publish educational content using Markdown, include interactive React components, and embed diagrams.

**Independent Test**: Can create a new Markdown file, add a React component, embed a Mermaid diagram, build the site locally, and view the content in a browser.

### Implementation for User Story 1

- [ ] T008 [US1] Organize `docs/` content by modules and weeks: `docs/`
- [ ] T009 [US1] Create an example MDX file with Markdown content: `docs/example/intro.mdx`
- [ ] T010 [P] [US1] Create an example custom React component: `src/components/MyInteractiveComponent.js`
- [ ] T011 [US1] Integrate the custom React component into an MDX file: `docs/example/intro.mdx`
- [ ] T012 [US1] Add an example Mermaid diagram to an MDX file: `docs/example/intro.mdx`
- [ ] T013 [US1] Verify local build and content rendering by running `npm run build` and serving `build/` directory locally.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Search and Navigate Content (Priority: P2)

**Goal**: As a student, easily search for specific topics and navigate through the textbook content.

**Independent Test**: Can search for a keyword and find relevant pages, and navigate between pages using the provided navigation elements.

### Implementation for User Story 2

- [ ] T014 [US2] Configure Algolia DocSearch integration: `docusaurus.config.js`
- [ ] T015 [US2] Set up Algolia application ID and API key as environment variables.
- [ ] T016 [US2] Verify search functionality on locally built site.
- [ ] T017 [US2] Ensure sidebar navigation is correctly generated and linked: `sidebars.js`
- [ ] T018 [US2] Verify breadcrumb navigation is functional (automatic with Docusaurus).
- [ ] T019 [US2] Verify previous/next page navigation is functional (automatic with Docusaurus).
- [ ] T020 [US2] Ensure each content page has a table of contents (automatic with Docusaurus).

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Customize Site Appearance and Deploy (Priority: P3)

**Goal**: As a developer, customize the site's styling with custom CSS, support dark/light themes, and deploy the site to GitHub Pages with automated CI/CD.

**Independent Test**: Can modify CSS, toggle dark/light theme, and push changes to trigger an automated deployment to GitHub Pages.

### Implementation for User Story 3

- [ ] T021 [P] [US3] Refine custom CSS for educational content (if needed): `src/css/custom.css`
- [ ] T022 [US3] Implement GitHub Actions workflow for automated CI/CD: `.github/workflows/deploy.yml`
- [ ] T023 [US3] Configure GitHub Pages deployment settings: `docusaurus.config.js` and GitHub repository settings.
- [ ] T024 [US3] Test automated deployment by pushing changes to a test branch.
- [ ] T025 [US3] (Optional) Setup custom domain support (requires DNS configuration).

**Checkpoint**: All user stories should now be independently functional

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T026 [P] Review all content for accessibility (color contrast, font sizes, keyboard navigation)
- [ ] T027 [P] Ensure all code examples have proper syntax highlighting (Prism)
- [ ] T028 [P] Verify tabs for different code examples (Python, C++) are implemented correctly (if custom)
- [ ] T029 [P] Validate admonitions for tips, warnings, and notes are rendered correctly
- [ ] T030 Run `quickstart.md` validation by following the steps outlined in the document.
- [ ] T031 Perform a final end-to-end test of the deployed site.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tasks within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Example of parallel tasks for User Story 1:
Task: "Create an example custom React component: src/components/MyInteractiveComponent.js"
Task: "Organize docs/ content by modules and weeks: docs/"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
