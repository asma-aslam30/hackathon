# Tasks: Urdu Translation Feature

**Input**: Design documents from `/specs/001-urdu-translation/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are included as this is a TDD approach per constitution (Principle III).

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md structure - Web application with backend + frontend + docs:
- **Backend**: `backend/src/`, `backend/tests/`
- **Docusaurus**: `docs/src/`
- **Frontend**: `frontend/src/`, `frontend/tests/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and environment setup

- [X] T001 Add translation dependencies to backend/requirements.txt (openai>=1.3.5 already present, ensure hashlib, beautifulsoup4)
- [X] T002 [P] Add Noto Nastaliq Urdu font link to docs/docusaurus.config.ts head section
- [X] T003 [P] Create RTL stylesheet in docs/src/css/urdu-rtl.css with RTL styling from research.md
- [X] T004 [P] Create environment variable documentation for TRANSLATION_CACHE_TTL_HOURS, TRANSLATION_MAX_CONTENT_LENGTH, TRANSLATION_RATE_LIMIT_PER_MINUTE

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T005 Create database migration for TranslationCache table in backend/migrations/001_add_translation_cache.sql (per data-model.md)
- [X] T006 Run database migration to create translation_cache table and add preferred_language to users
- [X] T007 [P] Create TranslationCache model in backend/src/models/translation.py (per data-model.md)
- [X] T008 [P] Extend User model with preferred_language field in backend/src/models/user.py
- [X] T009 [P] Create Pydantic schemas (TranslationRequest, TranslationResponse, TranslationError, TranslationMetadata) in backend/src/models/translation_schemas.py
- [X] T010 [P] Create translation settings in backend/src/config/settings.py (TRANSLATION_CACHE_TTL_HOURS=24, MAX_CONTENT_LENGTH=100000, RATE_LIMIT_PER_MINUTE=10)
- [X] T011 Register TranslationCache model in backend/src/models/__init__.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Urdu Translation Button (Priority: P1) 🎯 MVP

**Goal**: Logged-in users can translate chapter content into Urdu by clicking a translation button

**Independent Test**: Click "Translate to Urdu" button → content displays in Urdu with RTL layout → click "Show Original" → content reverts

### Tests for User Story 1 ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T012 [P] [US1] Contract test for POST /translation/translate-to-urdu in backend/tests/contract/test_translation_contract.py
- [ ] T013 [P] [US1] Unit test for TranslationService.translate_to_urdu() in backend/tests/unit/test_translation_service.py
- [ ] T014 [P] [US1] Component test for UrduTranslationButton in docs/src/components/UrduTranslationButton/__tests__/index.test.js

### Implementation for User Story 1

- [X] T015 [P] [US1] Create RTL helper utilities in backend/src/utils/rtl_helper.py (add dir='rtl', lang='ur' attributes)
- [X] T016 [P] [US1] Create HTML content processor in backend/src/utils/html_processor.py (preserve code blocks, extract translatable text)
- [X] T017 [US1] Create TranslationService class in backend/src/services/translation_service.py with translate_to_urdu() method
- [X] T018 [US1] Implement Gemini translation call in TranslationService using URDU_TRANSLATION_PROMPT from research.md
- [X] T019 [US1] Implement cache lookup and storage in TranslationService (check TranslationCache, store on miss)
- [X] T020 [US1] Create translation router in backend/src/api/translation.py with POST /translate-to-urdu endpoint
- [X] T021 [US1] Add authentication dependency (get_current_user) to translation endpoint
- [X] T022 [US1] Register translation router in backend/src/api/main.py (prefix: /translation)
- [X] T023 [P] [US1] Create translationService.js in frontend/src/services/translationService.js with translateToUrdu() function
- [X] T024 [P] [US1] Create translationService.js in docs/src/services/translationService.js (copy from frontend)
- [X] T025 [US1] Create UrduTranslationButton component in docs/src/components/UrduTranslationButton/index.js
- [X] T026 [US1] Add UrduTranslationButton styles in docs/src/components/UrduTranslationButton/styles.module.css
- [X] T027 [US1] Create useTranslation hook in docs/src/components/UrduTranslationButton/hooks/useTranslation.js
- [X] T028 [US1] Implement "Show Original" button functionality in UrduTranslationButton component
- [X] T029 [US1] Add translation state management (IDLE, LOADING, TRANSLATED, ERROR) per data-model.md

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Translation Quality and Accuracy (Priority: P2)

**Goal**: Translated content maintains accuracy and readability in Urdu with proper technical terminology handling

**Independent Test**: Translate content with technical terms and idioms → verify meaning preserved and grammar correct

### Tests for User Story 2 ⚠️

- [ ] T030 [P] [US2] Unit test for technical terminology translation in backend/tests/unit/test_translation_quality.py
- [ ] T031 [P] [US2] Unit test for HTML structure preservation in backend/tests/unit/test_html_processor.py

### Implementation for User Story 2

- [ ] T032 [US2] Enhance URDU_TRANSLATION_PROMPT with technical terminology guidelines in backend/src/services/translation_service.py
- [ ] T033 [US2] Add code block detection and exclusion in backend/src/utils/html_processor.py
- [ ] T034 [US2] Implement inline code preservation (`<code>` tags) in html_processor.py
- [ ] T035 [US2] Add mathematical formula handling (preserve LaTeX/MathJax content) in html_processor.py
- [ ] T036 [US2] Add link preservation (translate text, keep href) in html_processor.py
- [ ] T037 [US2] Add table cell content translation while preserving structure in html_processor.py
- [ ] T038 [US2] Add image alt text translation in html_processor.py

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Translation Performance and User Experience (Priority: P2)

**Goal**: Smooth translation experience with loading indicators, caching, and graceful error handling

**Independent Test**: Initiate translation → see loading indicator within 500ms → verify <3s for 5000 words → test error scenarios

### Tests for User Story 3 ⚠️

- [ ] T039 [P] [US3] Unit test for cache hit/miss scenarios in backend/tests/unit/test_translation_cache.py
- [ ] T040 [P] [US3] Integration test for translation endpoint performance in backend/tests/integration/test_translation_performance.py
- [ ] T041 [P] [US3] Component test for loading indicator in docs/src/components/UrduTranslationButton/__tests__/loading.test.js

### Implementation for User Story 3

- [ ] T042 [US3] Add loading spinner component in docs/src/components/UrduTranslationButton/LoadingSpinner.js
- [ ] T043 [US3] Implement loading indicator display within 500ms in UrduTranslationButton/index.js
- [ ] T044 [US3] Add error message display component in docs/src/components/UrduTranslationButton/ErrorMessage.js
- [ ] T045 [US3] Implement graceful error handling with retry button in UrduTranslationButton/index.js
- [ ] T046 [US3] Add cache TTL expiration check in backend/src/services/translation_service.py
- [ ] T047 [US3] Implement cache hit count increment in TranslationService
- [ ] T048 [US3] Add GET /translation/status health check endpoint in backend/src/api/translation.py
- [ ] T049 [US3] Add DELETE /translation/cache/clear endpoint for user cache clearing in backend/src/api/translation.py
- [ ] T050 [US3] Add rate limiting middleware for translation endpoint (10 req/min per user) in backend/src/api/translation.py
- [ ] T051 [US3] Implement chunked translation for large content (>5000 words) in TranslationService
- [ ] T052 [US3] Add retry-after header on rate limit response (per contracts/translation-api.yaml)

**Checkpoint**: At this point, User Stories 1, 2 AND 3 should all work independently

---

## Phase 6: User Story 4 - Accessibility and Multi-language Support (Priority: P3)

**Goal**: Translation controls accessible via keyboard and screen readers with WCAG 2.1 AA compliance

**Independent Test**: Navigate with keyboard only → verify screen reader announces state changes → toggle languages without losing scroll position

### Tests for User Story 4 ⚠️

- [ ] T053 [P] [US4] Accessibility test for keyboard navigation in docs/src/components/UrduTranslationButton/__tests__/a11y.test.js
- [ ] T054 [P] [US4] Integration test for scroll position preservation in docs/src/components/UrduTranslationButton/__tests__/scroll.test.js

### Implementation for User Story 4

- [ ] T055 [US4] Add aria-label and aria-describedby to UrduTranslationButton in docs/src/components/UrduTranslationButton/index.js
- [ ] T056 [US4] Add aria-busy and aria-live attributes for loading state in UrduTranslationButton
- [ ] T057 [US4] Add lang="ur" attribute to translated content container in UrduTranslationButton
- [ ] T058 [US4] Implement keyboard event handlers (Enter, Space) in UrduTranslationButton
- [ ] T059 [US4] Add screen reader announcements for translation completion ("Content translated to Urdu")
- [ ] T060 [US4] Implement scroll position preservation in useTranslation hook
- [ ] T061 [US4] Add focus management after translation completes in UrduTranslationButton
- [ ] T062 [US4] Add visible focus indicators in docs/src/components/UrduTranslationButton/styles.module.css
- [ ] T063 [US4] Add Urdu language direction indicator in UI (visual cue for RTL mode)

**Checkpoint**: All user stories should now be independently functional

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T064 [P] Add structured logging for translation requests in backend/src/services/translation_service.py
- [ ] T065 [P] Add translation analytics (track usage per chapter_id) in TranslationService
- [ ] T066 [P] Create TranslatedChapter wrapper component in docs/src/components/TranslatedChapter/index.js
- [ ] T067 [P] Add TranslatedChapter styles in docs/src/components/TranslatedChapter/styles.module.css
- [ ] T068 [P] Copy UrduTranslationButton component to frontend/src/components/UrduTranslationButton/
- [ ] T069 Integrate UrduTranslationButton into Docusaurus chapter pages
- [ ] T070 Run quickstart.md validation - verify all setup steps work
- [ ] T071 Add translation feature documentation to project docs
- [ ] T072 Security review: verify API key protection, input sanitization, XSS prevention
- [ ] T073 Performance optimization: verify 3s target for 5000 words (SC-001)
- [ ] T074 Create cache cleanup job/script for expired translations

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User stories can proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P2 → P3)
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Enhances US1 but independently testable
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - Enhances US1 but independently testable
- **User Story 4 (P3)**: Can start after Foundational (Phase 2) - Enhances US1 but independently testable

### Within Each User Story

- Tests MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Backend before frontend
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Frontend and backend tasks marked [P] within same story can run in parallel

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together:
Task: "Contract test for POST /translation/translate-to-urdu in backend/tests/contract/test_translation_contract.py"
Task: "Unit test for TranslationService.translate_to_urdu() in backend/tests/unit/test_translation_service.py"
Task: "Component test for UrduTranslationButton in docs/src/components/UrduTranslationButton/__tests__/index.test.js"

# Launch parallel backend utilities:
Task: "Create RTL helper utilities in backend/src/utils/rtl_helper.py"
Task: "Create HTML content processor in backend/src/utils/html_processor.py"

# Launch parallel frontend service files:
Task: "Create translationService.js in frontend/src/services/translationService.js"
Task: "Create translationService.js in docs/src/services/translationService.js"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
   - Login → View chapter → Click "Translate to Urdu" → Verify Urdu content
   - Click "Show Original" → Verify revert works
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo (quality improvements)
4. Add User Story 3 → Test independently → Deploy/Demo (performance + UX)
5. Add User Story 4 → Test independently → Deploy/Demo (accessibility)
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Backend - TranslationService, API)
   - Developer B: User Story 1 (Frontend - UrduTranslationButton)
   - After US1: Parallel work on US2, US3, US4
3. Stories complete and integrate independently

---

## Summary

| Phase | Tasks | Parallel Opportunities |
|-------|-------|------------------------|
| Phase 1: Setup | 4 | 3 |
| Phase 2: Foundational | 7 | 5 |
| Phase 3: US1 (P1 MVP) | 18 | 8 |
| Phase 4: US2 (P2) | 9 | 2 |
| Phase 5: US3 (P2) | 14 | 3 |
| Phase 6: US4 (P3) | 11 | 2 |
| Phase 7: Polish | 11 | 5 |
| **Total** | **74** | **28** |

### MVP Scope (Recommended)

Complete through User Story 1 (Phases 1-3) for minimum viable product:
- **Tasks**: 29 tasks
- **Deliverable**: Working translation button with Urdu output and revert functionality

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
