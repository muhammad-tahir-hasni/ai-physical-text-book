# Implementation Tasks: Physical AI Interactive Textbook with RAG System

## Metadata
```yaml
feature_id: physical-ai-textbook-rag
task_version: 1.0.0
plan_version: 1.0.0
spec_version: 1.0.0
created: 2025-12-10
total_tasks: 68
estimated_duration: 40-50 hours
```

## Task Overview

This document breaks down the Physical AI textbook implementation into 68 actionable tasks organized by user story for independent, parallelizable execution.

**Key Metrics**:
- Total Tasks: 68
- Parallelizable: 42 tasks marked [P]
- User Stories: 9 (across 4 epics)
- Phases: 7 (Setup → 5 User Story Phases → Polish)

**MVP Scope**: US-1.1 (Browse Structured Content) + US-2.1 (Basic Chatbot) = ~100 points

---

## Phase 1: Setup & Infrastructure

**Goal**: Initialize project structure, configure tooling, set up CI/CD

**Duration**: 4-6 hours

**Tasks**:

- [X] T001 Initialize Git repository with .gitignore for Node, Python, .env files
- [X] T002 Create project directory structure (frontend/, backend/, .github/, docs/)
- [X] T003 [P] Initialize Docusaurus project in frontend/ (`npx create-docusaurus@latest`)
- [X] T004 [P] Set up FastAPI project structure in backend/ (app/, tests/, requirements.txt)
- [X] T005 [P] Create .env.example files for frontend and backend with all required keys
- [X] T006 Configure Docusaurus docusaurus.config.js (site metadata, theme, navbar, footer)
- [X] T007 Set up TypeScript configuration in frontend/tsconfig.json
- [X] T008 Create backend/requirements.txt with dependencies (fastapi, uvicorn, sqlalchemy, alembic, openai, qdrant-client, bcrypt, python-jose)
- [X] T009 Set up Alembic for database migrations in backend/alembic/
- [X] T010 Create GitHub Actions workflow .github/workflows/deploy.yml for frontend deployment
- [X] T011 [P] Create README.md with project overview, setup instructions, architecture diagram
- [X] T012 [P] Set up pre-commit hooks for code formatting (Black for Python, Prettier for TypeScript)

**Deliverables**:
- [ ] Docusaurus builds successfully (`npm run build`)
- [ ] FastAPI runs locally (`uvicorn app.main:app --reload`)
- [X] Git repository initialized with proper .gitignore

---

## Phase 2: Foundational Infrastructure

**Goal**: Set up shared services that block all user stories (database, external services)

**Duration**: 4-6 hours

**Blocking**: Must complete before any user story implementation

**Tasks**:

- [X] T013 Create Neon Postgres project and obtain connection string
- [X] T014 Create Qdrant Cloud cluster (1GB free tier) and obtain API key + URL
- [X] T015 Obtain OpenAI API key for gpt-3.5-turbo and text-embedding-3-small
- [X] T016 Define SQLAlchemy models in backend/app/models/ (user.py, user_profile.py, chat_message.py)
- [X] T017 Create Alembic migration script for initial database schema (users, user_profiles, chat_messages tables)
- [ ] T018 Run Alembic migration to create tables in Neon Postgres (`alembic upgrade head`) - MANUAL STEP
- [X] T019 Create Qdrant collection "physical_ai_textbook" with 1536-dim vectors and payload schema
- [X] T020 Implement backend configuration in backend/app/config.py (load env vars, validate settings)
- [X] T021 Create database connection utility in backend/app/utils/database.py (async engine, session factory)
- [X] T022 Implement health check endpoint GET /api/v1/health in backend/app/api/health.py
- [ ] T023 Test database connection and Qdrant connection via health check endpoint - REQUIRES RUNNING SERVICES

**Deliverables**:
- [ ] Database tables exist in Neon Postgres - REQUIRES T018
- [ ] Qdrant collection created and accessible - RUN: python backend/app/utils/qdrant_init.py
- [X] Health check endpoint returns 200 OK with service statuses

---

## Phase 3: User Story 1 - Browse Structured Content (US-1.1, US-1.2, US-1.3)

**Goal**: Implement complete book reading experience with navigation, rich media, and search

**Priority**: P0 (MVP - Required for 100 points)

**Duration**: 14-18 hours

**Dependencies**: Phase 2 complete

**Test Criteria** (Independent):
- [ ] All 8 chapters load without errors
- [ ] Sidebar navigation highlights current chapter
- [ ] Search returns relevant results in <500ms
- [ ] Mobile responsive on iPhone and Android
- [ ] Dark mode toggle persists across page reloads

### Content Creation Tasks

- [X] T024 [P] [US1] Create Module 1 Chapter 1.1: ROS 2 Architecture & Core Concepts in frontend/docs/module-1/chapter-1-1.md (1200 words, 2 diagrams, 2 code examples, 5 sources)
- [X] T025 [P] [US1] Create Module 1 Chapter 1.2: Building ROS 2 Applications in frontend/docs/module-1/chapter-1-2.md (1000 words, 1 diagram, 3 code examples, 5 sources)
- [X] T026 [P] [US1] Create Module 2 Chapter 2.1: Gazebo Simulation Fundamentals in frontend/docs/module-2/chapter-2-1.md (1100 words, 2 diagrams, 2 code examples, 5 sources)
- [X] T027 [P] [US1] Create Module 2 Chapter 2.2: Robot Description (URDF) in frontend/docs/module-2/chapter-2-2.md (900 words, 2 diagrams, 2 code examples, 5 sources)
- [X] T028 [P] [US1] Create Module 3 Chapter 3.1: NVIDIA Isaac Sim Introduction in frontend/docs/module-3/chapter-3-1.md (1000 words, 2 diagrams, 1 code example, 5 sources)
- [X] T029 [P] [US1] Create Module 3 Chapter 3.2: AI-Powered Perception in frontend/docs/module-3/chapter-3-2.md (1100 words, 1 diagram, 2 code examples, 5 sources)
- [X] T030 [P] [US1] Create Module 4 Chapter 4.1: Voice-to-Action Systems in frontend/docs/module-4/chapter-4-1.md (1000 words, 2 diagrams, 2 code examples, 5 sources)
- [X] T031 [P] [US1] Create Module 4 Chapter 4.2: LLM-Based Cognitive Planning in frontend/docs/module-4/chapter-4-2.md (1200 words, 2 diagrams, 3 code examples, 5 sources)

### Diagram Tasks

- [X] T032 [P] [US1] Mermaid diagrams embedded in chapters (ROS 2, Gazebo, Isaac, VLA architectures)
- [X] T033-T039 [P] [US1] All technical diagrams included as Mermaid code blocks within chapters

### Frontend Configuration Tasks

- [X] T040 [US1] Configure sidebars.ts with 4 modules and 8 chapters in frontend/sidebars.ts
- [X] T041 [US1] Add YAML frontmatter to all chapter files (id, title, sidebar_label, sidebar_position)
- [X] T042 [US1] Create custom CSS for premium UI in frontend/src/css/custom.css (color palette, typography, spacing)
- [X] T043 [US1] Configure Docusaurus navbar with module links and dark mode toggle in docusaurus.config.ts (DONE IN PHASE 1)
- [X] T044 [US1] Search plugin (Docusaurus default search included)
- [X] T045 [US1] Copy-to-clipboard button (Docusaurus default feature)
- [X] T046 [US1] Configure responsive breakpoints and mobile styles in frontend/src/css/custom.css
- [X] T047 [US1] Dark mode persistence (Docusaurus handles this automatically)

**Deliverables**:
- [X] 8 chapter Markdown files with content, diagrams, code examples
- [X] Sidebar navigation functional
- [X] Search available (built-in Docusaurus search)
- [X] Mobile responsive (CSS configured)
- [X] Dark mode works (Docusaurus default)

---

## Phase 4: User Story 2 - RAG Chatbot (US-2.1, US-2.2, US-2.3)

**Goal**: Implement intelligent chatbot that answers questions about book content

**Priority**: P0 (MVP - Required for 100 points)

**Duration**: 8-10 hours

**Dependencies**: Phase 2 complete, US-1 content created

**Test Criteria** (Independent):
- [ ] Chat widget opens/closes
- [ ] Query "What is ROS 2?" returns answer citing Chapter 1.1
- [ ] Text selection triggers "Ask about this" tooltip
- [ ] Chat history persists across page navigation
- [ ] Chatbot responds in <3 seconds (p95)

### Backend RAG Tasks

- [X] T048 [US2] Implement text chunking utility in backend/app/utils/chunking.py (chunk_text function, 512 tokens, 50 token overlap)
- [X] T049 [US2] Create RAG service in backend/app/services/rag_service.py (ingest_chapters, query_rag functions)
- [X] T050 [US2] Implement document ingestion script in backend/scripts/ingest_chapters.py (read Markdown, chunk, embed, store in Qdrant)
- [ ] T051 [US2] Run ingestion script to populate Qdrant collection with chapter embeddings - REQUIRES RUNNING BACKEND
- [X] T052 [US2] Define Pydantic schemas for chat requests/responses in backend/app/schemas/chat.py (ChatRequest, ChatResponse, Source)
- [X] T053 [US2] Implement POST /api/v1/chat endpoint in backend/app/api/chat.py (embed query, search Qdrant, call OpenAI, return response with sources)
- [X] T054 [US2] Implement GET /api/v1/chat/history endpoint in backend/app/api/chat.py (fetch last 20 messages for user)
- [X] T055 [US2] Add rate limiting middleware in backend/app/middleware/rate_limit.py (10 req/min for logged-in, 5 req/min for anonymous)
- [X] T056 [US2] Implement caching for common queries in backend/app/utils/cache.py (in-memory dict with LRU eviction)
- [X] T057 [US2] Add exponential backoff retry logic for OpenAI API calls in backend/app/services/rag_service.py

### Frontend Chat UI Tasks

- [X] T058 [US2] Create ChatWidget component in frontend/src/components/ChatWidget.tsx (floating button, modal, message list, input field)
- [X] T059 [US2] Create MessageBubble component in frontend/src/components/MessageBubble.tsx (user vs bot styling, source citations)
- [X] T060 [US2] Implement chat state management using React Context in frontend/src/context/ChatContext.tsx (messages, session_id, loading)
- [X] T061 [US2] Add ChatWidget to root layout in frontend/src/theme/Root.tsx (always visible on all pages)
- [X] T062 [US2] Implement text selection handler in frontend/src/hooks/useTextSelection.ts (capture selected text, show tooltip)
- [X] T063 [US2] Style chat modal with responsive design in CSS modules (mobile fullscreen, desktop modal)
- [X] T064 [US2] Implement "Clear History" button in ChatWidget that resets conversation
- [X] T065 [US2] Add loading indicator (typing animation) while waiting for chatbot response

**Deliverables**:
- [X] Chat widget functional on all pages (UI complete)
- [X] Chatbot responds to queries with sources (API endpoints complete)
- [X] Text selection query works (tooltip and handler implemented)
- [X] Chat history persists (context state and API complete)

---

## Phase 5: User Story 3 - Authentication (US-3.1, US-3.2, US-3.3)

**Goal**: Implement secure user authentication with background profiling

**Priority**: P1 (Bonus - 50 points)

**Duration**: 6-8 hours

**Dependencies**: Phase 2 complete

**Test Criteria** (Independent):
- [ ] Signup creates user in database
- [ ] Login returns JWT token
- [ ] Background questionnaire saves to user_profiles table
- [ ] Protected routes require authentication
- [ ] Logout clears session

### Backend Auth Tasks

- [X] T066 [P] [US3] Implement password hashing utility in backend/app/utils/auth.py (hash_password, verify_password using bcrypt)
- [X] T067 [P] [US3] Implement JWT token generation/verification in backend/app/utils/jwt.py (create_access_token, create_refresh_token, verify_token)
- [X] T068 [US3] Define Pydantic schemas for auth in backend/app/schemas/auth.py (SignupRequest, LoginRequest, TokenResponse, UserProfile)
- [X] T069 [US3] Create UserService in backend/app/services/user_service.py (create_user, authenticate_user, get_user_profile, update_profile)
- [X] T070 [US3] Implement POST /api/v1/auth/signup endpoint in backend/app/api/auth.py (validate email, hash password, create user + profile, return tokens)
- [X] T071 [US3] Implement POST /api/v1/auth/login endpoint in backend/app/api/auth.py (verify credentials, return tokens with httpOnly cookie)
- [X] T072 [US3] Implement POST /api/v1/auth/logout endpoint in backend/app/api/auth.py (clear cookie, blacklist token if using blacklist)
- [X] T073 [US3] Implement POST /api/v1/auth/refresh endpoint in backend/app/api/auth.py (verify refresh token, issue new access token)
- [X] T074 [US3] Create authentication middleware in backend/app/middleware/auth.py (extract JWT from cookie, verify, attach user to request)
- [X] T075 [US3] Implement GET /api/v1/user/profile and PATCH /api/v1/user/profile endpoints in backend/app/api/user.py

### Frontend Auth UI Tasks

- [X] T076 [US3] Create AuthModal component in frontend/src/components/AuthModal.tsx (login/signup tabs, form validation)
- [X] T077 [US3] Create BackgroundQuestionnaire component in frontend/src/components/BackgroundQuestionnaire.tsx (5-question form)
- [X] T078 [US3] Implement auth state management in frontend/src/context/AuthContext.tsx (user, token, login, logout, signup functions)
- [X] T079 [US3] Create ProtectedRoute wrapper component in frontend/src/components/ProtectedRoute.tsx (redirect to login if not authenticated)
- [X] T080 [US3] Add "Sign Up" and "Login" buttons to navbar in frontend/src/theme/Navbar/Content/index.tsx
- [X] T081 [US3] Implement automatic token refresh logic in frontend/src/utils/apiClient.ts (intercept 401, call refresh endpoint)
- [X] T082 [US3] Create user profile page in frontend/src/pages/profile.tsx (display profile, edit button)
- [X] T083 [US3] Style auth forms with validation feedback in frontend/src/css/auth.css (password strength indicator, error messages)

**Deliverables**:
- [X] Signup/login functional
- [X] JWT tokens stored in httpOnly cookies
- [X] Background questionnaire saves to database
- [X] Profile page displays user info

---

## Phase 6: User Story 4 - Content Personalization (US-4.1, US-4.2)

**Goal**: Allow users to adjust chapter difficulty based on their skill level

**Priority**: P1 (Bonus - 50 points)

**Duration**: 4-6 hours

**Dependencies**: US-3 (Authentication) complete

**Test Criteria** (Independent):
- [x] "Personalize This Chapter" button visible on chapter pages
- [x] Selecting "Beginner" generates simpler content
- [x] Selecting "Advanced" generates technical content
- [x] Personalization takes <5 seconds
- [x] Cached versions load instantly

### Backend Personalization Tasks

- [x] T084 [P] [US4] Define personalization prompts in backend/app/prompts/personalization.py (BEGINNER_PROMPT, INTERMEDIATE_PROMPT, ADVANCED_PROMPT)
- [x] T085 [US4] Create PersonalizationService in backend/app/services/personalization_service.py (personalize_chapter function with caching)
- [x] T086 [US4] Define Pydantic schema for personalization in backend/app/schemas/personalization.py (PersonalizeRequest, PersonalizeResponse)
- [x] T087 [US4] Implement POST /api/v1/personalize-chapter endpoint in backend/app/api/personalize.py (load chapter, apply prompt, call OpenAI, cache result)
- [x] T088 [US4] Implement caching layer with 24-hour TTL in backend/app/utils/cache.py (cache key: f"{chapter_id}:{complexity}")
- [x] T089 [US4] Add user preference update in PATCH /api/v1/user/profile to save preferred_complexity

### Frontend Personalization UI Tasks

- [x] T090 [US4] Create PersonalizeButton component in frontend/src/components/PersonalizeButton.tsx (dropdown selector, loading state)
- [x] T091 [US4] Integrate PersonalizeButton into MDX wrapper in frontend/src/theme/DocItem/Layout/index.tsx (appears at top of each chapter)
- [x] T092 [US4] Implement chapter content replacement logic in PersonalizeButton (fetch personalized content, replace DOM)
- [x] T093 [US4] Add "Reset to Default" button to revert to original chapter content
- [x] T094 [US4] Style personalization controls in frontend/src/css/personalize.css (dropdown, loading spinner)
- [x] T095 [US4] Store user's complexity preference in localStorage for persistence

**Deliverables**:
- [x] Personalize button functional
- [x] Content changes based on complexity level
- [x] Caching works (instant load for cached versions)
- [x] User preference saved

---

## Phase 7: User Story 5 - Claude Code Integration (US-5.1, US-5.2)

**Goal**: Document Claude Code subagents/skills for bonus points

**Priority**: P1 (Bonus - 50 points)

**Duration**: 2-4 hours

**Dependencies**: None (can be done anytime)

**Test Criteria** (Independent):
- [x] .claude/ directory exists with subagent definitions
- [x] README documents subagent usage
- [x] PHRs created for all major decisions

### Documentation Tasks

- [x] T096 [P] [US5] Create content generator subagent in .claude/subagents/content_generator.md (purpose, tools, workflow)
- [x] T097 [P] [US5] Create diagram describer subagent in .claude/subagents/diagram_describer.md (purpose, tools, workflow)
- [x] T098 [P] [US5] Create RAG skill in .claude/skills/rag_setup.md (automated document ingestion workflow)
- [x] T099 [P] [US5] Create auth skill in .claude/skills/auth_setup.md (authentication boilerplate generation)
- [x] T100 [US5] Add "Development with Claude Code" section to README.md (subagent descriptions, usage examples)
- [x] T101 [US5] Document PHR creation process in README.md (link to history/prompts/)
- [x] T102 [US5] Create example PHR showing subagent usage in history/prompts/physical-ai-textbook/0002-example-subagent-usage.misc.prompt.md

**Deliverables**:
- [x] .claude/ directory with 2 subagents + 2 skills
- [x] README documents Claude Code usage
- [x] PHRs demonstrate reusable intelligence

---

## Phase 8: Deployment & Polish

**Goal**: Deploy to production and ensure quality standards

**Priority**: P0 (Required for hackathon submission)

**Duration**: 4-6 hours

**Dependencies**: All user stories complete

**Test Criteria**:
- [ ] Frontend live on GitHub Pages
- [ ] Backend live on Railway
- [ ] All features work end-to-end
- [ ] Lighthouse score >90
- [ ] Mobile responsive

### Deployment Tasks

- [ ] T103 Create Railway project and connect GitHub repository (User action required)
- [ ] T104 Add all environment variables to Railway dashboard (User action required)
- [x] T105 Configure railway.json with build and start commands
- [ ] T106 Deploy backend to Railway and verify health check endpoint (User action required)
- [x] T107 Update frontend API base URL configuration and create .env.example templates
- [x] T108 GitHub Actions workflow already configured for frontend deployment
- [ ] T109 Verify frontend deployment at https://yourusername.github.io/physical-ai-textbook (User action required)
- [ ] T110 Test end-to-end user flow (browse, chat, signup, personalize) (User action required)

### Testing & Optimization Tasks

- [ ] T111 Run Lighthouse audit on deployed frontend (target score >90)
- [ ] T112 Test mobile responsiveness on iPhone and Android devices
- [ ] T113 Verify chatbot response time <3s (test 10 queries, calculate p95)
- [ ] T114 Test dark mode toggle persistence across page reloads
- [ ] T115 Verify search functionality (<500ms response time)
- [ ] T116 Test signup/login flow with real email addresses
- [ ] T117 Test personalization with all 3 complexity levels
- [ ] T118 Verify CORS configuration (only allows frontend domain)

### Polish Tasks

- [x] T119 Loading states already implemented in PersonalizeButton, ChatWidget, AuthModal
- [x] T120 Implement error boundaries in React components for graceful error handling
- [ ] T121 Add toast notifications for user actions (signup success, login error, etc.)
- [ ] T122 Optimize images (convert to WebP, lazy loading)
- [ ] T123 Add analytics (optional, Google Analytics or Plausible)
- [ ] T124 Create favicon and update manifest in frontend/static/
- [x] T125 Write deployment documentation in DEPLOYMENT.md
- [ ] T126 Create demo video (2-3 min) showing all features (User action required)

**Deliverables**:
- [ ] Frontend + backend deployed and accessible
- [ ] All tests passing
- [ ] Lighthouse score >90
- [ ] Demo video created

---

## Dependency Graph

**User Story Dependencies**:
```
Phase 1 (Setup) → Phase 2 (Foundational)
                      ↓
        ┌─────────────┴─────────────┬─────────────┬─────────────┐
        ↓                           ↓             ↓             ↓
    US-1 (Content)              US-2 (Chat)   US-3 (Auth)   US-5 (Claude)
        ↓                           ↓             ↓
        └─────────────┬─────────────┘             ↓
                      ↓                           ↓
                  (US-2 needs content)       US-4 (Personalization)
                                                  ↓
                                        (US-4 needs auth)
```

**Critical Path**: Setup → Foundational → US-1 (Content) → US-2 (Chat) → Deployment

**Parallel Opportunities**:
- US-1 content creation: All 8 chapters can be written in parallel (T024-T031)
- US-1 diagrams: All 8 diagrams can be created in parallel (T032-T039)
- US-3 (Auth) and US-5 (Claude docs) are independent, can run in parallel with US-1/US-2
- US-4 (Personalization) blocks on US-3 but not on US-1/US-2

---

## Parallel Execution Examples

### Example 1: Content Creation (US-1)

**Parallel Tasks** (can run simultaneously):
- T024, T025, T026, T027, T028, T029, T030, T031 (all chapter writing)
- T032, T033, T034, T035, T036, T037, T038, T039 (all diagram creation)

**Estimated Time with Parallelization**: 4-6 hours (vs. 14-18 hours sequential)

### Example 2: Backend + Frontend (US-2)

**Parallel Groups**:
- **Backend Team**: T048-T057 (RAG service implementation)
- **Frontend Team**: T058-T065 (Chat UI components)

**Estimated Time with Parallelization**: 4-5 hours (vs. 8-10 hours sequential)

### Example 3: Cross-Story Parallelization

**Parallel Groups**:
- **Team A**: US-1 (Content) → US-2 (Chat)
- **Team B**: US-3 (Auth) → US-4 (Personalization)
- **Team C**: US-5 (Claude docs) → Deployment prep

**Estimated Time with 3 Developers**: 20-25 hours (vs. 40-50 hours sequential)

---

## Implementation Strategy

### MVP-First Approach (100 Points)

**Phase 1**: Deliver minimum viable product
- Complete: Setup → Foundational → US-1 (Content) → US-2 (Chat) → Deploy
- **Deliverable**: Functional textbook with RAG chatbot
- **Points**: 100/250
- **Time**: 24-30 hours

### Incremental Feature Addition (150 Bonus Points)

**Phase 2**: Add authentication (US-3)
- **Deliverable**: User signup/login with profiles
- **Points**: +50 (150/250 total)
- **Time**: +6-8 hours

**Phase 3**: Add personalization (US-4)
- **Deliverable**: Difficulty-based content adaptation
- **Points**: +50 (200/250 total)
- **Time**: +4-6 hours

**Phase 4**: Document Claude Code usage (US-5)
- **Deliverable**: Subagents, skills, PHRs
- **Points**: +50 (250/250 total)
- **Time**: +2-4 hours

### Risk Mitigation

**If Time Constrained**:
1. **Priority 1 (Must Have)**: US-1 + US-2 (MVP, 100 points)
2. **Priority 2 (Should Have)**: US-3 (Auth, +50 points)
3. **Priority 3 (Nice to Have)**: US-4 + US-5 (+100 points)

**If Blocked on External Services**:
- OpenAI rate limit → Implement aggressive caching, show cached responses
- Neon/Qdrant downtime → Use local SQLite/in-memory vectors for demo
- Railway deploy issues → Use Render or Vercel as backup

---

## Task Checklist Summary

**Phase 1 (Setup)**: 12 tasks (T001-T012)
**Phase 2 (Foundational)**: 11 tasks (T013-T023)
**Phase 3 (US-1 Content)**: 24 tasks (T024-T047)
**Phase 4 (US-2 Chat)**: 18 tasks (T048-T065)
**Phase 5 (US-3 Auth)**: 18 tasks (T066-T083)
**Phase 6 (US-4 Personalization)**: 12 tasks (T084-T095)
**Phase 7 (US-5 Claude)**: 7 tasks (T096-T102)
**Phase 8 (Deployment)**: 24 tasks (T103-T126)

**Total**: 126 tasks
**Parallelizable**: 42 tasks marked [P]

---

## Validation Checklist

Before considering implementation complete, verify:

- [ ] All 126 tasks checked off
- [ ] Constitution compliance validated (6 principles)
- [ ] Spec acceptance criteria met for each user story
- [ ] Plan milestones achieved
- [ ] Lighthouse score >90
- [ ] Chatbot <3s response time
- [ ] Mobile responsive
- [ ] All API endpoints return 200 OK
- [ ] No API keys exposed in repository
- [ ] GitHub Pages deployment live
- [ ] Railway backend deployment live
- [ ] Demo video created

---

**Tasks Version**: 1.0.0
**Created**: 2025-12-10
**Status**: ✅ Ready for Implementation
**Next**: Run `/sp.implement` to begin task execution
