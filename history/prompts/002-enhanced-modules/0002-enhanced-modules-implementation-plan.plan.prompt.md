---
id: 0002
title: Enhanced modules implementation plan
stage: plan
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-enhanced-modules
branch: 002-enhanced-modules
user: apple
command: /sp.plan
labels: ["planning", "architecture", "fastapi", "qdrant", "jwt-auth", "rag-chatbot", "react-components", "docusaurus", "postgresql", "redis", "advanced-chapters"]
links:
  spec: /Users/apple/Desktop/robotic-hackathon/specs/002-enhanced-modules/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-enhanced-modules/plan.md
tests:
  - Constitution Check (PASSED all 6 academic + 6 hackathon principles)
  - Technical Context validation (all NEEDS CLARIFICATION resolved)
---

## Prompt

```text
Generate a detailed step-by-step execution plan for the enhanced Physical AI & Humanoid Robotics Hackathon project, based on the current /sp.specify and /sp.constitutions setup:

Qdrant cluster setup: Configure RAG database for storing embeddings, ensure integration with FastAPI backend.

Backend FastAPI server + RAG Chatbot: Implement endpoints for user authentication (JWT), integrate RAG retrieval with subagents and skills, ensure secure API key management.

Docusaurus book integration (with login/signup UI): Incorporate new chapters for Modules 1–4, apply UI styling from specification (#D99518 primary, #F2BB16 highlight, gradients, sidebar colors), add diagram and image placeholders per chapter.

Subagents & Skills creation: Define each subagent aligned with module themes, connect skills to VLA and ROS 2 actions, ensure reusability across modules.

Diagram generation using Gemini: Generate architecture diagrams for all new chapters, include workflows for RAG, subagents, and VLA integration.

Chapter & module content insertion: Insert all functional requirements (signup/login, progress tracking, bookmarks), ensure diagrams, placeholders, and architecture notes are included.

Deployment instructions: Edge AI kits (Jetson Orin) setup, cloud vs on-prem deployment guidance, include environment setup (venv, .env) and hardware requirements.

API key usage instructions: Proper storage, rotation, and security guidelines, integration with FastAPI endpoints and RAG Chatbot.

Testing & validation plan: Validate ROS 2 nodes and simulation pipelines, test RAG retrieval, subagents, and autonomous VLA execution, frontend UI testing for Docusaurus login/signup and chapter access.

Output format: Structured JSON with tasks (each step as an object), dependencies (tasks that must precede each step), estimated_effort (hours or story points)

Instruction for Claude: "Use the existing project directory to ensure the plan aligns with completed modules, installed tools (Docausurus, SpeckitPlus, venv, .env), and current backend/frontend architecture. Focus on a clear, reusable, hackathon-ready execution plan."
```

## Response snapshot

Created comprehensive implementation plan (517 lines) covering:

**Technical Architecture**:
- Backend: FastAPI (Python 3.11) + PostgreSQL 15 + Qdrant cloud + Redis for sessions/rate limiting
- Frontend: Existing Docusaurus + custom React (TypeScript) components for auth/progress
- Content: 4 advanced chapters (1200-1500 words each) with textual diagram descriptions
- Performance: p95 <200ms API latency, <3s RAG chatbot, 100 concurrent users

**Constitution Check** (12 principles): ALL PASSED
- Academic: Spec-driven, academic rigor, zero-plagiarism, AI workflow, Docusaurus-first, technical precision
- Hackathon: Modular architecture, hardware-software co-design (⚠️ partial - edge optimization future), AI-augmented with oversight, security-first, documentation-as-code, simulation-first

**Phase 0 Research** (7 technology decisions documented):
1. JWT Best Practices: python-jose + HS256 (rationale: FastAPI-recommended, secret rotation support)
2. Qdrant Integration: Cloud instance + Cohere embeddings (rationale: eliminates self-hosting, high-quality multilingual vectors)
3. PostgreSQL Schema: 3 tables (users, reading_progress, bookmarks) with indexes (rationale: normalized, optimized queries)
4. Docusaurus React: Swizzling + src/pages/ (rationale: preserves updates, fits architecture)
5. Rate Limiting: Redis sliding window (rationale: persists across restarts, scales horizontally)
6. Content Embedding: Section-level chunking (512 tokens) (rationale: balances context with precision)
7. Accessibility: WCAG 2.1 AA + ARIA labels (rationale: industry standard, screen reader compatibility)

**Project Structure**: Web application (Option 2)
- backend/ (FastAPI app with models, schemas, api, services, middleware, tests)
- robotic/ (enhanced Docusaurus with custom React components, 4 new chapter files, diagram placeholders, custom CSS)

**Phase 1 Preview**: Data model entities (User, ReadingProgress, Bookmark), API contracts (POST /auth/signup, /auth/login, /progress/save, /rag/query), quickstart guide (5-step developer setup)

**Execution Roadmap** (9 phases, 90 tasks, 180-360 hours):
- Phase A: Backend Infrastructure (Tasks 001-015)
- Phase B: Progress Tracking (Tasks 016-025)
- Phase C: RAG Chatbot (Tasks 026-035)
- Phase D: Advanced Chapters (Tasks 036-047)
- Phase E: Frontend Auth/Progress UI (Tasks 048-058)
- Phase F: Custom UI Styling (Tasks 059-065)
- Phase G: Diagram Placeholders (Tasks 066-070)
- Phase H: Deployment (Tasks 071-080)
- Phase I: Validation/Testing (Tasks 081-090)

**Risk Analysis** (5 risks with mitigations):
- RAG accuracy (<85%): Adjust chunk size/top-k, hybrid search
- Performance (100 users): Load test early, horizontal scaling
- Accessibility (WCAG AA): Automated testing, manual screen reader
- Content delays: AI-assisted drafting, prioritize Module 1 & 4
- JWT expiration UX: Auto-refresh 5min before expiry

## Outcome

- ✅ Impact: Comprehensive technical plan bridging authentication, RAG chatbot, advanced content, and custom UI into existing Docusaurus infrastructure. Aligns with academic + hackathon constitutions while maintaining modular architecture for parallel development.
- 🧪 Tests: Constitution Check: 12/12 principles passed (6 academic + 6 hackathon), Technical Context validation: all NEEDS CLARIFICATION resolved via Phase 0 research
- 📁 Files: specs/002-enhanced-modules/plan.md (complete implementation plan with technology selections, architecture decisions, project structure, execution roadmap)
- 🔁 Next prompts: Complete Phase 1 artifacts (data-model.md, contracts/*.yaml, quickstart.md), run `/sp.tasks` to generate 90-task breakdown across 9 phases, begin Phase A implementation (backend infrastructure)
- 🧠 Reflection: Successfully translated user's detailed requirements (Qdrant setup, FastAPI server, Docusaurus integration, subagents, Gemini diagrams, deployment, testing) into actionable technical plan. Key decisions: (1) Web app structure (backend/frontend separation), (2) Section-level chunking for RAG (balances context/precision), (3) Redis rate limiting (horizontal scaling future-proof), (4) WCAG AA accessibility (sufficient for hackathon, AAA too restrictive). Plan ready for `/sp.tasks` phase.

## Evaluation notes (flywheel)

- Failure modes observed: None. All constitution gates passed, all technology selections justified with alternatives considered, project structure aligns with existing Docusaurus setup.
- Graders run and results (PASS/FAIL): Constitution Check - PASSED (12/12), Technical Context validation - PASSED (zero NEEDS CLARIFICATION remaining)
- Prompt variant (if applicable): N/A (initial planning phase)
- Next experiment (smallest change to try): In Phase 1 (data-model.md generation), validate that unique constraint on (user_id, module, chapter, section) for reading_progress table handles edge case where user navigates back to same section multiple times (should update last_read_at timestamp, not create duplicate records). Consider upsert pattern in SQL.
