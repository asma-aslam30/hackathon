---
id: "0001"
title: "urdu-translator-constitution"
stage: constitution
date: "2025-12-20"
surface: "agent"
model: "claude-opus-4-5-20251101"
feature: "UrduTranslator"
branch: "main"
user: "xolva"
command: "sp.constitution \"UrduTranslator\" --type \"subagent\" --responsibility \"Adds translation button and translates content to Urdu dynamically\""
labels: ["constitution","governance","translation","urdu"]
links:
  spec: null
  plan: null
  tasks: null
  ticket: null
  adr: null
  pr: null
files:
 - "/.specify/memory/constitution.md"
tests: []
---

## Prompt

sp.constitution "UrduTranslator" --type "subagent" --responsibility "Adds translation button and translates content to Urdu dynamically"

## Response snapshot

Updated project constitution to version 1.1.0 by adding Principle VII: Urdu Translation Capability. The new principle establishes requirements for the UrduTranslator subagent to provide dynamic content translation with user-accessible translation button, seamless integration into content display systems, and preservation of content meaning during translation to Urdu. Additional constraints added for Unicode/RTL support, and development workflow updated to include linguistic validation tests.

## Outcome

- ✅ Impact: Constitution updated with new principle for Urdu translation capability
- 🧪 Tests: No tests specified in constitution stage
- 📁 Files: Updated /.specify/memory/constitution.md with new principle
- 🔁 Next prompts: Ready for spec generation with new constitutional requirements
- 🧠 Reflection: Version bumped from 1.0.0 to 1.1.0 as new principle was added

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate spec for UrduTranslator feature