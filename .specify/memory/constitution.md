<!-- SYNC IMPACT REPORT
Version change: 1.0.0 → 1.1.0
Modified principles: None
Added sections: VII. Urdu Translation Capability
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ updated
  - .specify/templates/spec-template.md ✅ updated
  - .specify/templates/tasks-template.md ✅ updated
  - .specify/templates/commands/*.md ✅ updated
Follow-up TODOs: None
-->

# Enhanced AI Robotics Constitution

## Core Principles

### I. AI-First Architecture
Every feature starts with AI integration in mind; Systems must support AI-assisted development and operation; Clear AI interaction patterns required - no features without AI considerations.

### II. MCP Integration Standard
Every service exposes functionality via Model Context Protocol; Standardized MCP tools: stdin/args → stdout for AI agents, errors → stderr; Support both JSON and human-readable formats for AI and human consumption.

### III. Documentation-First (NON-NEGOTIABLE)
TDD mandatory for all AI integrations: Test documentation access → User approves → Tests fail → Then implement; Documentation-first cycle (Context7) enforced.

### IV. Multi-Modal Tool Access
Focus areas requiring multi-modal tool access: MCP server integrations, Context7 documentation retrieval, Inter-agent communication, Shared context schemas.

### V. Context-Driven Decision Making
Systems must be observable with AI-assist in mind; Structured logging required for AI analysis; Context7 integration ensures debuggability; MAJOR.MINOR.BUILD versioning format with semantic AI integration markers.

### VI. Human-AI Collaboration
Start simple with AI assistance; YAGNI principles apply to AI features; Gradual enhancement of AI capabilities based on context and need.

### VII. Urdu Translation Capability
The UrduTranslator subagent MUST provide dynamic content translation to Urdu with a user-accessible translation button; Translation functionality MUST be seamlessly integrated into content display systems; All translation operations MUST preserve original content meaning and structure while adapting to Urdu linguistic patterns.

## Additional Constraints

Technology stack requirements: Node.js runtime with MCP support, Context7 integration capability, AI agent compatibility; Compliance standards: MCP v1.0 protocol compliance, Context7 documentation standards; Deployment policy: MCP servers must be separately deployable and independently scalable; Translation services must support Unicode and RTL text rendering for Urdu script.

## Development Workflow

Code review requirements: All AI integration code must include MCP tool definitions; Testing gates: AI-assisted tests must pass before human review; Deployment approval process: MCP endpoint validation required; AI interaction patterns must be documented in Context7 format; Translation quality must be verified through linguistic validation tests.

## Governance

Constitution supersedes all other practices; Amendments require documentation, approval, and migration plan; All PRs/reviews must verify MCP compliance; Complexity must be justified with AI-usage scenarios; Use Context7 documentation for runtime development guidance; Translation features must undergo linguistic review before deployment.

**Version**: 1.1.0 | **Ratified**: 2025-12-12 | **Last Amended**: 2025-12-20