---
id: 0004
title: Urdu Translator MVP Implementation
stage: green
date: 2025-12-20
surface: agent
model: claude-opus-4-5-20251101
feature: 001-urdu-translation
branch: 001-urdu-translation
user: architect
command: /sp.implement
labels: ["implementation", "urdu", "translation", "gemini", "react"]
links:
  spec: ./spec.md
  ticket: null
  adr: null
  pr: null
files:
  - backend/requirements.txt
  - backend/src/config/settings.py
  - backend/migrations/001_add_translation_cache.sql
  - backend/src/models/translation.py
  - backend/src/models/translation_schemas.py
  - backend/src/models/user.py
  - backend/src/models/__init__.py
  - backend/src/utils/rtl_helper.py
  - backend/src/utils/html_processor.py
  - backend/src/services/translation_service.py
  - backend/src/api/translation.py
  - backend/src/api/main.py
  - docs/docusaurus.config.ts
  - docs/src/css/urdu-rtl.css
  - docs/src/services/translationService.js
  - docs/src/components/UrduTranslationButton/index.js
  - docs/src/components/UrduTranslationButton/styles.module.css
  - docs/src/components/UrduTranslationButton/hooks/useTranslation.js
tests:
  - (Test tasks T012-T014 pending - implementation-first approach used)
---

## Prompt

"UrduTranslator" \
--instructions "Use React/Docusaurus components
Use Claude Code translation function or GEMINI API
Update DOM on button click to show Urdu text"

## Response snapshot

Implemented the core Urdu Translation MVP (Phases 1-3) with the following components:

**Phase 1 - Setup (4 tasks):**
- Added lxml dependency to requirements.txt
- Added Noto Nastaliq Urdu font to docusaurus.config.ts
- Created RTL stylesheet (urdu-rtl.css)
- Added translation settings to settings.py

**Phase 2 - Foundational (7 tasks):**
- Created database migration for translation_cache table
- Created TranslationCache SQLAlchemy model
- Extended User model with preferred_language field
- Created Pydantic schemas (TranslationRequest, TranslationResponse, etc.)
- Registered models in __init__.py

**Phase 3 - User Story 1 MVP (15 implementation tasks):**
- Backend:
  - RTL helper utilities (rtl_helper.py)
  - HTML content processor (html_processor.py)
  - TranslationService with Gemini API integration
  - Translation API router with auth dependency
  - Registered router in main.py

- Frontend (Docusaurus):
  - translationService.js client library
  - UrduTranslationButton React component
  - useTranslation custom hook with state management
  - RTL-aware CSS styles

**Key Implementation Details:**
- Uses Gemini API (google-generativeai) per user instruction
- PostgreSQL cache with 24-hour TTL
- JWT authentication via existing BetterAuth
- TRANSLATION_STATES: IDLE, LOADING, TRANSLATED, ERROR
- RTL rendering with Noto Nastaliq Urdu font
- Graceful error handling with retry capability

## Outcome

- ✅ Impact: High - Complete MVP implementation for Urdu translation feature
- 🧪 Tests: Test tasks (T012-T014) pending - implementation completed first
- 📁 Files: 18 files created/modified across backend and frontend
- 🔁 Next prompts: Run tests, deploy and validate, then proceed to Phase 4-7
- 🧠 Reflection: Used Gemini API per user instruction instead of OpenAI; followed existing auth patterns from codebase

## Evaluation notes (flywheel)

- Failure modes observed: None - all implementation tasks completed successfully
- Graders run and results (PASS/FAIL): Manual testing pending
- Prompt variant (if applicable): Used Gemini API per user specification
- Next experiment (smallest change to try): Test the translation endpoint with sample content to verify Gemini integration works
