# Tasks: Enhanced Physical AI & Humanoid Robotics Hackathon Platform

**Input**: Design documents from `/specs/002-enhanced-modules/`
**Prerequisites**: plan.md (complete), spec.md (4 user stories: P1-Authentication, P2-Progress, P3-Content, P4-UI)

**Organization**: Tasks grouped by user story for independent implementation and testing

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: User story label (US1, US2, US3, US4)
- Exact file paths included in descriptions

## Path Conventions

- **Backend**: `/backend/` (FastAPI application)
- **Frontend**: `/robotic/` (existing Docusaurus, enhanced)
- **Content**: `/robotic/docs/module-X/` (advanced chapters)
- **Diagrams**: `/robotic/static/diagrams/` (SVG placeholders)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and directory structure
**Actor**: Claude (automated setup) + Manual (.env configuration)

- [X] T001 Create backend/ directory structure (app/, models/, schemas/, api/, services/, middleware/, db/, tests/)
- [X] T002 Initialize backend/requirements.txt with FastAPI, SQLAlchemy, python-jose, bcrypt, Pydantic, qdrant-client, cohere, pytest
- [X] T003 [P] Create backend/.env.example with placeholders for DATABASE_URL, REDIS_URL, JWT_SECRET_KEY, QDRANT_URL, QDRANT_API_KEY, COHERE_API_KEY, CLAUDE_API_KEY
- [X] T004 [P] Create backend/app/config.py to load environment variables using Pydantic BaseSettings
- [X] T005 [P] Initialize Alembic for database migrations in backend/db/migrations/
- [X] T006 [P] Create robotic/src/components/ directory structure (Auth/, Progress/, RAG/)
- [X] T007 [P] Update robotic/package.json to add React, TypeScript, axios dependencies
- [X] T008 Create robotic/static/diagrams/ directory for SVG placeholders

**Manual Action Required**: Copy backend/.env.example to backend/.env and fill in actual API keys from existing .env file

**Checkpoint**: Directory structure ready for parallel development

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure blocking all user stories
**Actor**: Claude (database, middleware) + Manual (PostgreSQL, Redis setup)

- [ ] T009 Create backend/app/db/database.py with PostgreSQL connection using SQLAlchemy async engine
- [ ] T010 [P] Create backend/app/middleware/jwt_auth.py for JWT token validation middleware
- [ ] T011 [P] Create backend/app/middleware/rate_limit.py for Redis-backed rate limiting (5 attempts/15min)
- [ ] T012 [P] Create backend/app/main.py FastAPI app with CORS, middleware, and OpenAPI docs
- [ ] T013 [P] Create robotic/src/components/Auth/AuthProvider.tsx React context for JWT token management

**Manual Action Required**: Install and start PostgreSQL 15+ and Redis 7+ locally or configure cloud instances

**⚠️ CRITICAL**: Phase 2 must complete before ANY user story implementation

**Checkpoint**: Backend server starts, database connected, Redis configured

---

## Phase 3: User Story 1 - User Account Management and Authentication (Priority: P1) 🎯 MVP

**Goal**: Users can signup, login with JWT tokens, and access protected routes
**Independent Test**: Create account, login, receive JWT, access /progress/get (should succeed), logout, access /progress/get (should fail 401)
**Actor**: Claude (models, services, endpoints, UI) + Manual (test with Postman/curl)

### Database Models

- [ ] T014 [P] [US1] Create User model in backend/app/models/user.py (id, email unique, password_hash, role, created_at, last_login)
- [ ] T015 [US1] Generate Alembic migration for users table in backend/db/migrations/versions/

### Services & Business Logic

- [ ] T016 [P] [US1] Create auth schemas in backend/app/schemas/auth.py (SignupRequest, LoginRequest, TokenResponse, UserResponse)
- [ ] T017 [US1] Implement AuthService in backend/app/services/auth_service.py (signup: bcrypt hash 12 rounds, login: JWT generation HS256 1hr expiry, refresh token 7 days)

### API Endpoints

- [ ] T018 [US1] Implement POST /auth/signup endpoint in backend/app/api/auth.py (validate email/password, hash, create user, return 201)
- [ ] T019 [US1] Implement POST /auth/login endpoint in backend/app/api/auth.py (validate credentials, check rate limit, return JWT + Set-Cookie)
- [ ] T020 [US1] Implement POST /auth/refresh-token endpoint in backend/app/api/auth.py (validate refresh token, issue new access token)

### Frontend UI

- [ ] T021 [P] [US1] Create SignupForm.tsx in robotic/src/components/Auth/ (email/password inputs, client-side validation, POST /auth/signup)
- [ ] T022 [P] [US1] Create LoginForm.tsx in robotic/src/components/Auth/ (email/password inputs, error display, POST /auth/login)
- [ ] T023 [US1] Create signup page in robotic/src/pages/auth/signup.tsx (render SignupForm component)
- [ ] T024 [US1] Create login page in robotic/src/pages/auth/login.tsx (render LoginForm component)
- [ ] T025 [US1] Implement JWT token storage in AuthProvider.tsx (httpOnly cookies via Set-Cookie headers, auto-refresh 5min before expiry)

### Validation & Security

- [ ] T026 [US1] Add rate limiting decorator to login endpoint (Redis-backed, 5 attempts per IP per 15 minutes)
- [ ] T027 [US1] Add authentication event logging in backend/app/services/auth_service.py (login, logout, failed attempts with timestamps, user_id, IP)

**Manual Action Required**: Test signup/login flow in browser, verify JWT in cookies (DevTools → Application → Cookies)
**API Keys Needed**: JWT_SECRET_KEY (generate with `openssl rand -hex 32`)

**Checkpoint**: User Story 1 complete and independently testable

---

## Phase 4: User Story 2 - Reading Progress and Bookmark Persistence (Priority: P2)

**Goal**: Auto-save reading position every 30s, bookmark sections, display progress dashboard
**Independent Test**: Login, read Module 2 for 30s, close browser, reopen, verify "Resume Reading" shows Module 2 position
**Actor**: Claude (models, services, endpoints, UI components)

### Database Models

- [ ] T028 [P] [US2] Create ReadingProgress model in backend/app/models/progress.py (id, user_id FK, module int, chapter str, section str, last_read_at, completion_pct, unique constraint on user_id+module+chapter+section)
- [ ] T029 [P] [US2] Create Bookmark model in backend/app/models/bookmark.py (id, user_id FK, module int, chapter str, section str, section_title, created_at)
- [ ] T030 [US2] Generate Alembic migration for reading_progress and bookmarks tables with indexes on user_id

### Services

- [ ] T031 [P] [US2] Create progress schemas in backend/app/schemas/progress.py (ProgressSaveRequest, ProgressResponse, BookmarkRequest, BookmarkListResponse)
- [ ] T032 [US2] Implement ProgressService in backend/app/services/progress_service.py (save_progress: upsert with last_read_at update, calculate_completion, get_user_progress, add/remove_bookmark)

### API Endpoints

- [ ] T033 [US2] Implement POST /progress/save endpoint in backend/app/api/progress.py (validate JWT, save progress, return 200)
- [ ] T034 [US2] Implement GET /progress/get endpoint in backend/app/api/progress.py (validate JWT, return user's current progress and "resume reading" link)
- [ ] T035 [US2] Implement GET /progress/bookmarks endpoint in backend/app/api/progress.py (validate JWT, return bookmarks list organized by module)
- [ ] T036 [US2] Implement DELETE /progress/bookmarks/{bookmark_id} endpoint (validate JWT, remove bookmark, return 204)

### Frontend UI

- [ ] T037 [P] [US2] Create ProgressTracker.tsx in robotic/src/components/Progress/ (useEffect hook: auto-save every 30s or on beforeunload, POST /progress/save)
- [ ] T038 [P] [US2] Create BookmarkButton.tsx in robotic/src/components/Progress/ (bookmark icon, filled/outline state, POST /progress/save for bookmarks)
- [ ] T039 [US2] Create Dashboard.tsx in robotic/src/components/Progress/ (display progress bars, "Resume Reading" link, bookmarks sidebar)
- [ ] T040 [US2] Create dashboard page in robotic/src/pages/dashboard.tsx (render Dashboard component)
- [ ] T041 [US2] Inject BookmarkButton into Docusaurus theme (swizzle DocItem or Layout, add bookmark icon next to section headings)
- [ ] T042 [US2] Wrap Docusaurus app with ProgressTracker component in robotic/src/theme/Root.tsx (swizzle Root component)

**Checkpoint**: User Story 2 complete - progress saves automatically, bookmarks work, dashboard displays

---

## Phase 5: User Story 3 - Enhanced Module Content with Advanced Chapters (Priority: P3)

**Goal**: Four 1200-1500 word chapters with textual diagrams, integration notes, RAG chatbot support
**Independent Test**: Navigate to robotic/docs/module-1/advanced-lifecycle.md, verify 3+ textual diagrams, APA citations, integration notes
**Actor**: Claude (content generation with RAG embedding) + Manual (peer review for academic tone)

### Content Creation

- [X] T043 [P] [US3] Write Module 1 advanced chapter in robotic/docs/module-1/advanced-lifecycle.md (1200-1500 words: ROS 2 Lifecycle states, node composition, real-time control, 3+ textual diagrams, APA citations, integration notes)
- [X] T044 [P] [US3] Write Module 2 advanced chapter in robotic/docs/module-2/advanced-sync.md (1200-1500 words: Gazebo-Unity sync, multi-simulator interop, clock sync, 3+ textual diagrams, APA citations)
- [X] T045 [P] [US3] Write Module 3 advanced chapter in robotic/docs/module-3/advanced-rl.md (1200-1500 words: RL for humanoid control, Isaac Gym, reward shaping, sim-to-real, 3+ textual diagrams, APA citations)
- [X] T046 [P] [US3] Write Module 4 advanced chapter in robotic/docs/module-4/advanced-vla.md (1200-1500 words: VLA-ROS integration, Whisper→LLM→actions, failure recovery, 3+ textual diagrams, APA citations)

### Diagram Placeholders

- [X] T047 [P] [US3] Create 3 SVG placeholders for Module 1 in robotic/static/diagrams/ (module-1-lifecycle-states.svg, module-1-node-composition.svg, module-1-realtime-pipeline.svg with alt-text and Gemini prompt metadata)
- [X] T048 [P] [US3] Create 3 SVG placeholders for Module 2 in robotic/static/diagrams/ (module-2-multi-sim-arch.svg, module-2-clock-sync.svg, module-2-sensor-replication.svg with alt-text)
- [ ] T049 [P] [US3] Create 3 SVG placeholders for Module 3 in robotic/static/diagrams/ (module-3-rl-pipeline.svg, module-3-domain-randomization.svg, module-3-sim-to-real.svg with alt-text)
- [ ] T050 [P] [US3] Create 3 SVG placeholders for Module 4 in robotic/static/diagrams/ (module-4-vla-integration.svg, module-4-whisper-llm-flow.svg, module-4-failure-recovery.svg with alt-text)

### Navigation Integration

- [ ] T051 [US3] Update robotic/sidebars.js to add 4 advanced chapter links under respective modules
- [ ] T052 [US3] Add YAML frontmatter to all 4 advanced chapters (title, description, keywords for SEO and progress tracking)

### RAG Chatbot Integration

- [ ] T053 [US3] Embed all 4 new chapters into Qdrant collection (use Cohere embed-english-v3.0, chunk by H2 sections, include metadata: module, chapter, section, title)

**Manual Action Required**: Peer review content for academic tone, APA citation accuracy, technical precision
**API Keys Needed**: COHERE_API_KEY (for embeddings), QDRANT_URL, QDRANT_API_KEY

**Checkpoint**: User Story 3 complete - 4 advanced chapters published, diagrams placeholders created, RAG-enabled

---

## Phase 6: RAG Chatbot Service (Blocking User Story 3 RAG queries)

**Purpose**: Enable context-aware Q&A over course content
**Actor**: Claude (RAG service, endpoint) + Manual (test queries)

### Qdrant Setup

- [ ] T054 Create backend/app/services/rag_service.py (initialize Qdrant client, create collection if not exists, embed_text using Cohere, search_similar with top-k=5)

### RAG Endpoint

- [ ] T055 Create RAG schemas in backend/app/schemas/rag.py (RAGQueryRequest with query and context_filter, RAGQueryResponse with answer, sources list, latency_ms)
- [ ] T056 Implement POST /rag/query endpoint in backend/app/api/rag.py (validate JWT, retrieve top-k chunks from Qdrant, generate answer with Claude/GPT-4, return sources, log latency)

### Frontend Chatbot Widget

- [ ] T057 [P] Create ChatbotWidget.tsx in robotic/src/components/RAG/ (chat UI, POST /rag/query, display answer with source citations, code syntax highlighting)
- [ ] T058 Inject ChatbotWidget into Docusaurus layout (swizzle Layout, add floating chatbot icon in bottom-right)

**Manual Action Required**: Test RAG accuracy with 10-20 diverse questions about course content, verify >85% relevance
**API Keys Needed**: CLAUDE_API_KEY or OPENAI_API_KEY (for answer generation), COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY

**Checkpoint**: RAG chatbot functional, answers questions about all course content including new chapters

---

## Phase 7: User Story 4 - Custom UI Styling and Branding (Priority: P4)

**Goal**: Apply gold/yellow hackathon branding with WCAG AA accessibility compliance
**Independent Test**: Inspect UI elements, verify primary color #D99518, highlight #F2BB16, sidebar active #B67C15, run Lighthouse accessibility audit (score >90)
**Actor**: Claude (CSS updates) + Manual (accessibility validation)

### CSS Styling

- [X] T059 [P] [US4] Update robotic/src/css/custom.css with CSS custom properties (--color-primary: #D99518, --color-highlight: #F2BB16, --color-sidebar-active: #B67C15, --color-page-bg: #FAF8F3)
- [X] T060 [P] [US4] Style primary buttons with gradient (linear-gradient(to right, #D99518, #F2BB16), hover: scale 1.05 transition)
- [X] T061 [P] [US4] Style text links with hover underline (border-bottom: 2px solid #F2BB16)
- [X] T062 [P] [US4] Style sidebar active items (background-color: #B67C15, ensure contrast ratio ≥4.5:1 with white text)
- [X] T063 [P] [US4] Style form input focus state (border-bottom: 2px solid #F2BB16)
- [X] T064 [US4] Apply page background color (background-color: #FAF8F3 for main content area)

### Accessibility Validation

- [ ] T065 [US4] Run Lighthouse CI accessibility audit (target score >90, verify WCAG 2.1 AA contrast ratios)
- [ ] T066 [US4] Add ARIA labels to custom components (BookmarkButton, ChatbotWidget, Dashboard navigation)

**Manual Action Required**: Manual screen reader testing (NVDA or JAWS), verify all interactive elements accessible via keyboard navigation
**Checkpoint**: User Story 4 complete - custom UI styling applied, accessibility validated

---

## Phase 8: Deployment & Documentation

**Purpose**: Prepare for production deployment with environment setup guidance
**Actor**: Claude (documentation) + Manual (deployment testing)

### Backend Deployment

- [ ] T067 Create backend/README.md with setup instructions (venv creation, pip install -r requirements.txt, alembic upgrade head, uvicorn app.main:app --reload)
- [ ] T068 Create backend/Dockerfile (optional: Python 3.11 slim, copy requirements.txt and app/, expose port 8000)
- [ ] T069 Document environment variables in backend/.env.example with descriptions (DATABASE_URL format, Redis URL, JWT secret generation command)

### Frontend Deployment

- [ ] T070 Update robotic/README.md with enhanced setup (npm install for React dependencies, npm start for dev, npm run build for production)
- [ ] T071 Validate Docusaurus build (run npm run build in robotic/, verify zero errors, check static/diagrams/ included)

### Edge AI Kit Guidance (Informational)

- [ ] T072 Create docs/deployment-guide.md with sections: Cloud deployment (backend on Ubuntu server), Edge considerations (Jetson Orin future notes), API key rotation procedures

### Integration Testing

- [ ] T073 Test full user flow: signup → login → read chapter → auto-save progress → bookmark section → RAG query → logout → login → resume reading → verify bookmark persisted

**Manual Action Required**: Deploy backend to staging server, deploy Docusaurus to GitHub Pages or Vercel, test with multiple concurrent users

**Checkpoint**: Platform deployment-ready, documentation complete

---

## Phase 9: Validation & Performance Testing

**Purpose**: Verify success criteria from spec (p95 <200ms, 100 concurrent users, >85% RAG accuracy, WCAG AA)
**Actor**: Manual (load testing, security audit)

- [ ] T074 Load test backend with Locust or k6 (simulate 100 concurrent users, measure p95 latency for /progress/save, /auth/login, /rag/query endpoints)
- [ ] T075 Benchmark RAG retrieval accuracy (curate 20 test questions with expected answers, measure >85% relevance threshold)
- [ ] T076 Security audit (verify bcrypt 12 rounds in code, JWT 1hr/7day expiration in config, rate limiting active on /auth/login, CORS restricted origins)
- [ ] T077 Cross-browser testing (Chrome, Firefox, Safari latest versions, verify UI styling consistent, auth flows work)
- [ ] T078 Accessibility audit (run axe DevTools on all pages, verify zero critical issues, color contrast ratios pass WCAG AA)

**Checkpoint**: All success criteria validated, platform production-ready

---

## Dependencies & Execution Strategy

### User Story Dependencies

- **US1 (Authentication)**: No dependencies - can start immediately after Phase 2
- **US2 (Progress Tracking)**: Depends on US1 (requires authenticated users)
- **US3 (Advanced Content)**: Independent of US1/US2 (can develop in parallel), but RAG chatbot (Phase 6) depends on US3 completion
- **US4 (UI Styling)**: Independent (can apply styling anytime), but best after US1/US2 UI components exist

### Suggested MVP Scope

**MVP = Phase 1 + Phase 2 + Phase 3 (User Story 1 only)**
- Delivers core value: users can create accounts and login securely
- Validates authentication infrastructure before building on it
- Estimated effort: ~40-60 hours for single developer

### Parallel Execution Opportunities

**After Phase 2 completes, these can run in parallel**:
- User Story 1 (T014-T027) - Team Member 1
- User Story 3 content creation (T043-T046) - Team Member 2 (or Claude with peer review)
- User Story 4 CSS styling (T059-T064) - Team Member 3

**After US1 completes**:
- User Story 2 (T028-T042) - builds on US1 authentication
- RAG Chatbot Phase 6 (T054-T058) - can start after US3 content available

### Total Effort Estimate

**78 tasks total**:
- Phase 1 Setup: 8 tasks (~8 hours)
- Phase 2 Foundational: 5 tasks (~10 hours)
- Phase 3 US1 Authentication: 14 tasks (~30 hours)
- Phase 4 US2 Progress Tracking: 15 tasks (~25 hours)
- Phase 5 US3 Advanced Content: 11 tasks (~35 hours with peer review)
- Phase 6 RAG Chatbot: 5 tasks (~15 hours)
- Phase 7 US4 UI Styling: 8 tasks (~12 hours)
- Phase 8 Deployment: 7 tasks (~10 hours)
- Phase 9 Validation: 5 tasks (~15 hours)

**Total: 160-180 hours** (4-5 weeks single developer, 2-3 weeks small team with parallel execution)

---

## Implementation Notes

**Manual Actions Summary**:
1. **T003**: Copy .env.example to .env, fill API keys
2. **PostgreSQL/Redis**: Install locally or provision cloud instances before T009
3. **US1 Testing**: Test signup/login in browser after T025
4. **Content Peer Review**: Review T043-T046 for academic rigor, APA citations
5. **RAG Testing**: Test query accuracy after T056 (10-20 diverse questions)
6. **Accessibility**: Screen reader testing after T066
7. **Deployment**: Deploy to staging, test full user flow (T073)
8. **Load Testing**: Simulate 100 concurrent users (T074)

**API Keys Needed**:
- `JWT_SECRET_KEY`: Generate with `openssl rand -hex 32`
- `CLAUDE_API_KEY`: From existing .env
- `COHERE_API_KEY`: From existing .env
- `QDRANT_URL`, `QDRANT_API_KEY`: From existing .env
- `DATABASE_URL`: PostgreSQL connection string (local or cloud)
- `REDIS_URL`: Redis connection string (local or cloud)

**Output Formats**:
- **Backend**: Python files (.py), YAML migrations, JSON OpenAPI docs
- **Frontend**: TypeScript React (.tsx), CSS (.css), Markdown (.md)
- **Content**: Markdown (.md) with YAML frontmatter, SVG placeholders
- **Diagrams**: SVG files with alt-text attributes and metadata comments

---

**Task List Status**: ✅ COMPLETE - Ready for `/sp.implement` execution

**Format Validation**: All 78 tasks follow checklist format `- [ ] [ID] [P?] [Story?] Description with file path`
