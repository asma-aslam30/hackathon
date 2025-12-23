---
id: 0002
title: Urdu Translator Implementation Plan
stage: plan
date: 2025-12-20
surface: agent
model: claude-opus-4-5-20251101
feature: 001-urdu-translation
branch: 001-urdu-translation
user: architect
command: /sp.plan
labels: ["translation", "urdu", "rtl", "i18n", "openai"]
links:
  spec: ./spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-urdu-translation/plan.md
  - specs/001-urdu-translation/research.md
  - specs/001-urdu-translation/data-model.md
  - specs/001-urdu-translation/contracts/translation-api.yaml
  - specs/001-urdu-translation/quickstart.md
tests:
  - (None - planning phase)
---

## Prompt

"UrduTranslator" \
--steps "1. Detect logged-in user
2. Add 'Translate to Urdu' button
3. Fetch chapter content on click
4. Translate content using Claude Code or OpenAI
5. Render Urdu content dynamically"

## Response snapshot

Completed Phase 0 (Research) and Phase 1 (Design & Contracts) for the Urdu translation feature:

**Phase 0 - Research Findings:**
- Translation API: OpenAI GPT-4 (already integrated in backend)
- RTL rendering: CSS direction + Noto Nastaliq Urdu font
- Caching: PostgreSQL-based with 24h TTL
- Authentication: Leverage existing BetterAuth JWT system
- Component pattern: Follow existing PersonalizationButton pattern
- Content processing: Server-side with HTML preservation

**Phase 1 - Design Artifacts:**
- plan.md: Technical context, constitution check, project structure
- research.md: All technical decisions documented with rationale
- data-model.md: TranslationCache entity, User extension, API schemas
- contracts/translation-api.yaml: OpenAPI 3.1 specification
- quickstart.md: Developer setup and usage guide

**Key Technical Decisions:**
1. OpenAI GPT-4 API for high-quality Urdu translation
2. PostgreSQL cache (24h TTL) to reduce API costs
3. CSS-based RTL with Noto Nastaliq Urdu font
4. Server-side translation preserving HTML structure
5. Progressive chunked loading for large content

## Outcome

- ✅ Impact: High - Complete planning artifacts for Urdu translation feature
- 🧪 Tests: Planning phase - no tests yet
- 📁 Files: 5 files created (plan.md, research.md, data-model.md, translation-api.yaml, quickstart.md)
- 🔁 Next prompts: Run /sp.tasks to generate implementation tasks
- 🧠 Reflection: Leveraged existing patterns (PersonalizationButton, ContentTransformer) for consistency

## Evaluation notes (flywheel)

- Failure modes observed: None - all technical decisions resolved
- Graders run and results (PASS/FAIL): Constitution check PASS (all 7 principles satisfied)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Implement backend translation service first to validate OpenAI integration
