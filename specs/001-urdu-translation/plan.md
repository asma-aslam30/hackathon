# Implementation Plan: Urdu Translation

**Branch**: `001-urdu-translation` | **Date**: 2025-12-20 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-urdu-translation/spec.md`

## Summary

Enable logged-in users to translate chapter content into Urdu by pressing a button. The system will detect authenticated users, display a "Translate to Urdu" button, fetch chapter content on click, translate using AI (OpenAI or Claude), and render Urdu content dynamically with RTL support. The implementation leverages existing ContentTransformer and Personalization patterns already established in the codebase.

## Technical Context

**Language/Version**: Python 3.11+ (backend), JavaScript/TypeScript (frontend), React 18.2.0
**Primary Dependencies**: FastAPI 0.104.1, OpenAI 1.3.5, SQLAlchemy 2.0.23, Axios 1.13.2, Docusaurus 3.9.2
**Storage**: PostgreSQL (Neon) via asyncpg for translation cache, existing user tables
**Testing**: pytest 7.4.3 (backend), Jest (frontend)
**Target Platform**: Web application (Docusaurus documentation site + React frontend)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: Translation within 3 seconds for chapters under 5,000 words (SC-001)
**Constraints**: <3s p95 translation latency, RTL text rendering, WCAG 2.1 AA accessibility
**Scale/Scope**: 20% of logged-in users expected within first month (SC-006)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Gate Check | Status |
|-----------|------------|--------|
| I. AI-First Architecture | Translation uses AI (OpenAI/Claude) for quality translation | ✅ PASS |
| II. MCP Integration Standard | Endpoint follows stdin/stdout pattern with JSON responses | ✅ PASS |
| III. Documentation-First | TDD approach - tests written before implementation | ✅ PASS |
| IV. Multi-Modal Tool Access | Integrates with existing MCP patterns for AI agents | ✅ PASS |
| V. Context-Driven Decision Making | Structured logging for translation requests and errors | ✅ PASS |
| VI. Human-AI Collaboration | Simple initial implementation, gradual enhancement | ✅ PASS |
| VII. Urdu Translation Capability | Core feature - button-triggered, preserves meaning/structure, RTL support | ✅ PASS |

**Additional Constraints Check**:
- Unicode and RTL text rendering: Required for Urdu script ✅
- Linguistic validation tests: Required before deployment ✅
- Translation services integration: OpenAI/Claude API ✅

## Project Structure

### Documentation (this feature)

```text
specs/001-urdu-translation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── translation.py       # NEW: TranslationCache model
│   │   └── user.py              # EXTEND: Add preferred_language field
│   ├── services/
│   │   └── translation_service.py  # NEW: UrduTranslator service
│   ├── api/
│   │   └── translation.py       # NEW: Translation endpoints
│   └── utils/
│       └── rtl_helper.py        # NEW: RTL text rendering utilities
└── tests/
    ├── unit/
    │   └── test_translation_service.py
    └── integration/
        └── test_translation_api.py

docs/
├── src/
│   ├── components/
│   │   ├── UrduTranslationButton/  # NEW: Translation trigger button
│   │   │   ├── index.js
│   │   │   └── styles.module.css
│   │   └── TranslatedChapter/      # NEW: RTL-aware chapter display
│   │       ├── index.js
│   │       └── styles.module.css
│   └── css/
│       └── urdu-rtl.css           # NEW: RTL-specific styles
└── docusaurus.config.ts           # EXTEND: i18n for Urdu locale

frontend/
├── src/
│   ├── components/
│   │   └── UrduTranslationButton/ # NEW: Translation button component
│   └── services/
│       └── translationService.js  # NEW: Translation API client
└── tests/
    └── translation.test.js
```

**Structure Decision**: Web application structure selected. Frontend components added to both `/docs/src/components/` (Docusaurus) and `/frontend/src/components/` (React app) to support translation across both platforms. Backend follows existing `/backend/src/` structure with new translation service and API endpoints.

## Complexity Tracking

> No constitution violations requiring justification.

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |

## Implementation Steps (User-Provided)

1. **Detect logged-in user**: Leverage existing JWT authentication via `authService` and `BetterAuth`
2. **Add 'Translate to Urdu' button**: Create `UrduTranslationButton` component with RTL-aware styling
3. **Fetch chapter content on click**: Call new translation endpoint with chapter content
4. **Translate content using AI**: Use OpenAI or Claude API for high-quality Urdu translation
5. **Render Urdu content dynamically**: Display translated content with RTL direction and proper font rendering

## Key Technical Decisions

1. **Translation Provider**: OpenAI GPT-4 API (already integrated) with Claude as fallback
2. **Caching Strategy**: PostgreSQL-based cache with 24-hour TTL to avoid redundant API calls
3. **RTL Implementation**: CSS `direction: rtl` with Urdu-optimized font stack (Noto Nastaliq Urdu)
4. **State Management**: React useState for translation toggle, similar to personalization pattern
5. **Error Handling**: Graceful degradation - show original content on translation failure

## Next Steps

After Phase 1 artifacts are complete, run `/sp.tasks` to generate actionable task breakdown.
