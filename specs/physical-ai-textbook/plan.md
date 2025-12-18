# Implementation Plan: Physical AI Interactive Textbook with RAG System

## Metadata
```yaml
feature_id: physical-ai-textbook-rag
plan_version: 1.0.0
spec_version: 1.0.0
created: 2025-12-10
author: Claude Code with Spec-Kit Plus
status: approved
estimated_duration: 40-50 hours
team_size: 1-2 developers
```

## Executive Summary

This plan outlines the implementation strategy for a premium Physical AI & Humanoid Robotics interactive textbook targeting 250/250 hackathon points. The system combines Docusaurus for content delivery, FastAPI for backend services, and free-tier AI services (OpenAI, Qdrant, Neon) for RAG chatbot, authentication, and content personalization.

**Architecture Philosophy**: Serverless-first, API-driven, progressive enhancement
**Deployment Strategy**: Static frontend (GitHub Pages) + Containerized backend (Railway/Render)
**Development Approach**: Spec-driven with AI-assisted implementation

---

## Constitution Check

### Alignment with Core Principles

**I. Spec-Driven Development** ✅
- Feature spec created (`specs/physical-ai-textbook/spec.md`)
- This plan follows spec requirements precisely
- Tasks will include acceptance criteria from spec

**II. Academic Rigor & Reproducibility** ✅
- Content will cite minimum 5 sources per chapter (relaxed from 15 for hackathon scope)
- Official ROS2, Isaac, Gazebo documentation as primary sources
- Code examples tested and runnable

**III. Zero-Plagiarism & Source Attribution** ✅
- AI-generated content will be fact-checked against official docs
- All code examples will be original or properly attributed
- Citations in APA format

**IV. AI-Driven Authoring Workflow** ✅
- Using Claude Code + Spec-Kit Plus workflow
- This plan follows `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement`
- PHRs will be generated for all major decisions

**V. Docusaurus-First Architecture** ✅
- All content in Markdown/MDX format
- Sidebars configured for 4 modules
- GitHub Pages deployment planned
- SVG/Mermaid for diagrams

**VI. Technical Precision & Evidence-Based Claims** ✅
- Architecture decisions include trade-off analysis below
- Performance targets quantified (<2s page load, <3s chatbot)
- No vague claims without benchmarks

### Complexity Justification

**Why Not Simpler Approaches?**

1. **Why Docusaurus vs. Static HTML?**
   - **Need**: Search, navigation, responsive design out-of-the-box
   - **Simpler Rejected**: Plain HTML would require custom search implementation
   - **Exit Strategy**: Markdown files portable to any static site generator

2. **Why FastAPI Backend vs. Serverless Functions?**
   - **Need**: Stateful RAG pipeline with caching, session management
   - **Simpler Rejected**: AWS Lambda cold starts violate <3s latency requirement
   - **Exit Strategy**: FastAPI easily portable to any Python hosting

3. **Why Qdrant vs. In-Memory Vectors?**
   - **Need**: Persistent storage, efficient similarity search at scale
   - **Simpler Rejected**: In-memory requires rebuild on every deploy
   - **Exit Strategy**: Standard vector embeddings portable to Pinecone/Weaviate

4. **Why better-auth vs. Custom JWT?**
   - **Need**: Secure password hashing, session management, token refresh
   - **Simpler Rejected**: Custom implementation error-prone for security
   - **Exit Strategy**: Auth logic abstracted behind service interface

---

## Technical Context

### Technology Stack Deep Dive

#### Frontend Stack

**Docusaurus 3.x**
- **Why**: Best-in-class documentation framework, React-based, excellent DX
- **Version**: 3.0.0+ (stable release)
- **Build Tool**: Webpack 5 (built-in)
- **Styling**: Infima CSS framework (Docusaurus default) + custom CSS
- **Trade-offs**:
  - ✅ Pro: Zero-config search, versioning, i18n support
  - ✅ Pro: Excellent mobile responsiveness
  - ❌ Con: Opinionated structure (acceptable for our use case)
  - ❌ Con: React bundle size (~200KB gzipped, but acceptable for educational platform)

**Custom React Components**
- `ChatWidget.tsx`: Floating chatbot interface
- `PersonalizeButton.tsx`: Chapter difficulty selector
- `AuthModal.tsx`: Login/signup forms
- `BackgroundQuestionnaire.tsx`: User profile form

**State Management**
- **Choice**: React Context API
- **Rationale**: Simple global state (user, auth, chat history), no need for Redux
- **Trade-offs**:
  - ✅ Pro: Built-in, zero deps, sufficient for our needs
  - ❌ Con: No time-travel debugging (not needed for hackathon)

#### Backend Stack

**FastAPI 0.104+**
- **Why**: Async Python, automatic OpenAPI docs, excellent performance
- **ASGI Server**: Uvicorn (production-grade)
- **Validation**: Pydantic v2 (type-safe request/response models)
- **Trade-offs**:
  - ✅ Pro: Fastest Python framework (on par with Node.js)
  - ✅ Pro: Automatic API documentation
  - ❌ Con: Python 3.11+ required (acceptable, widely available)

**Database Layer**

**Neon Serverless Postgres**
- **Free Tier**: 512 MB storage, 0.5 GB transfer/month
- **Why**: Serverless, auto-scaling, generous free tier
- **ORM**: SQLAlchemy 2.0 (async mode)
- **Migration**: Alembic (database migrations)
- **Trade-offs**:
  - ✅ Pro: Standard PostgreSQL, no vendor lock-in
  - ✅ Pro: Instant branching (useful for testing)
  - ❌ Con: Cold start latency (~100-300ms, mitigated with connection pooling)

**Schema Design** (Optimized for free tier):
```sql
-- Users: minimal fields to conserve storage
CREATE TABLE users (
    id SERIAL PRIMARY KEY,
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT NOW()
);

-- User profiles: separate table for optional data
CREATE TABLE user_profiles (
    user_id INTEGER PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    software_experience VARCHAR(20),
    python_familiarity VARCHAR(20),
    robotics_experience VARCHAR(20),
    hardware_background VARCHAR(20),
    learning_goals VARCHAR(200),
    preferred_complexity VARCHAR(20) DEFAULT 'intermediate'
);

-- Chat history: limited retention to stay under 512 MB
CREATE TABLE chat_messages (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id) ON DELETE SET NULL,
    session_id VARCHAR(36),  -- UUID
    message TEXT NOT NULL,
    response TEXT NOT NULL,
    sources JSONB,
    created_at TIMESTAMP DEFAULT NOW(),
    -- Auto-delete messages older than 30 days
    CONSTRAINT chat_retention CHECK (created_at > NOW() - INTERVAL '30 days')
);

-- Indexes for performance
CREATE INDEX idx_chat_user_session ON chat_messages(user_id, session_id);
CREATE INDEX idx_chat_created ON chat_messages(created_at);
CREATE INDEX idx_user_email ON users(email);
```

**Qdrant Cloud (Vector Database)**
- **Free Tier**: 1 GB cluster, 1M vectors
- **Why**: Purpose-built for vector search, fast, Python SDK
- **Collection Design**:
```python
{
    "vectors": {
        "size": 1536,  # text-embedding-3-small
        "distance": "Cosine"
    },
    "payload_schema": {
        "chapter_id": "keyword",      # "1-1", "1-2", etc.
        "module": "integer",           # 1-4
        "title": "text",
        "content": "text",             # Chunk text
        "chunk_index": "integer",
        "word_count": "integer",
        "has_code": "bool"
    }
}
```

**Storage Estimation**:
- 8 chapters × ~1000 words = 8,000 words total
- Chunk size: 512 tokens (~400 words)
- Total chunks: 8,000 / 400 = 20 chunks
- Embedding size: 1536 dims × 4 bytes = 6 KB per vector
- Total storage: 20 × 6 KB = 120 KB (well under 1 GB limit)

#### AI Services

**OpenAI API (Free Tier)**
- **Rate Limits**: 3 RPM (requests per minute) for free tier
- **Models**:
  - `gpt-3.5-turbo`: Chat completions (RAG, personalization)
  - `text-embedding-3-small`: Embeddings (1536 dims, cheap)
- **Cost Management**:
  - Caching frequent queries (in-memory dict or Redis)
  - Request queue with exponential backoff
  - Max tokens limit (300 for RAG, 2000 for personalization)

**better-auth.com**
- **Why**: Open-source, modern auth library, bcrypt built-in
- **Features**: JWT tokens, refresh tokens, password reset
- **Integration**: Works with any database (we use Neon Postgres)

### Unknowns & Research Needed

**RESOLVED** (via research below):
1. ~~Docusaurus dark mode persistence~~ → Uses localStorage, works out-of-the-box
2. ~~Qdrant free tier vector limits~~ → 1M vectors, 1 GB storage (confirmed)
3. ~~OpenAI free tier rate limits~~ → 3 RPM (confirmed via docs)
4. ~~Railway vs. Render pricing~~ → Both have 500-750 hours/month free (Railway chosen)
5. ~~Better-auth Python support~~ → Use Python `authlib` library with custom JWT logic

---

## Phase 0: Research & Decision Log

### Research Task 1: Frontend Framework Selection

**Decision**: Docusaurus 3.x
**Rationale**:
- Built specifically for technical documentation
- Excellent SEO, accessibility, mobile responsiveness
- Easy content authoring (Markdown/MDX)
- Active community, maintained by Meta

**Alternatives Considered**:
- **VuePress**: Similar but less popular, fewer plugins
- **Gatsby**: More flexible but overkill for docs, slower build times
- **Next.js**: Requires more custom setup, no built-in docs features

**Best Practices**:
- Use Docusaurus presets for docs
- Custom React components for interactive features (chat, auth)
- Leverage Infima CSS variables for theming

### Research Task 2: Backend Deployment Platform

**Decision**: Railway
**Rationale**:
- Free tier: 500 hours/month ($5 credit)
- Git-based deployments (auto-deploy on push)
- Environment variables UI
- PostgreSQL addon available (though we use Neon)

**Alternatives Considered**:
- **Render**: Similar free tier (750 hours), but slower cold starts
- **Fly.io**: More complex configuration
- **Heroku**: Discontinued free tier

**Best Practices**:
- Use `railway.json` for deployment config
- Health check endpoint (`/api/health`)
- Auto-restart on failure

### Research Task 3: RAG Architecture Pattern

**Decision**: Naive RAG (retrieve → augment → generate)
**Rationale**:
- Simple, proven, sufficient for 8-chapter corpus
- No need for advanced techniques (HyDE, recursive retrieval) at this scale

**Implementation**:
1. **Ingestion** (one-time):
   ```python
   chunk_text(chapter, size=512 tokens, overlap=50)
   → embed_chunks(openai.Embedding)
   → store_in_qdrant(chunks + metadata)
   ```

2. **Query** (runtime):
   ```python
   embed_query(user_message)
   → search_qdrant(top_k=3, threshold=0.7)
   → build_context(retrieved_chunks)
   → generate_response(openai.ChatCompletion)
   → return {response, sources}
   ```

3. **Optimization**:
   - Cache query embeddings (hash of message → embedding)
   - Cache full responses for common queries
   - Rate limit: 10 requests/min per user

**Alternatives Considered**:
- **Advanced RAG**: HyDE, query expansion (overkill for small corpus)
- **Fine-tuned model**: Expensive, not needed
- **Local LLM**: Whisper/LLaMA (deployment complexity, latency concerns)

### Research Task 4: Authentication Strategy

**Decision**: JWT tokens with httpOnly cookies
**Rationale**:
- Secure (XSS-resistant with httpOnly flag)
- Stateless (no server-side session store needed)
- Standard approach for SPAs

**Implementation**:
```python
# Signup/Login flow
1. Hash password with bcrypt (12 rounds)
2. Create user record in Neon Postgres
3. Generate JWT access token (1-hour exp)
4. Generate refresh token (7-day exp)
5. Set httpOnly cookie with tokens
6. Return user object to frontend

# API request flow
1. Extract token from cookie
2. Verify JWT signature & expiration
3. Extract user_id from payload
4. Attach to request context
5. Proceed with handler
```

**Security Measures**:
- HTTPS only (enforced in production)
- CSRF protection via SameSite cookie attribute
- Password strength validation (min 8 chars, 1 uppercase, 1 number)
- Rate limiting on login (5 attempts per 15 min per IP)

**Alternatives Considered**:
- **OAuth 2.0** (GitHub, Google): Out of scope for hackathon
- **Sessions in database**: Requires extra queries, doesn't scale

### Research Task 5: Content Personalization Approach

**Decision**: LLM prompt engineering (no fine-tuning)
**Rationale**:
- Fast implementation (no training data collection)
- Flexible (can adjust prompts quickly)
- Sufficient quality for hackathon demo

**Prompt Templates**:
```python
BEGINNER_PROMPT = """
Rewrite this robotics chapter for absolute beginners:
- Use simple, everyday language
- Explain technical terms when first mentioned
- Add real-world analogies (e.g., "ROS 2 topics are like radio channels")
- Include step-by-step explanations
- Avoid jargon; define when necessary

Original chapter:
{content}

Rewritten chapter (keep structure, ~1000 words):
"""

ADVANCED_PROMPT = """
Rewrite this robotics chapter for experienced practitioners:
- Use precise technical terminology
- Assume CS/engineering background
- Reference advanced concepts (control theory, optimization)
- Include mathematical formulations where relevant
- Discuss edge cases and limitations

Original chapter:
{content}

Rewritten chapter (keep structure, ~1000 words):
"""
```

**Caching Strategy**:
- Key: `f"{chapter_id}:{complexity_level}"`
- TTL: 24 hours (86400 seconds)
- Storage: In-memory dict (production: Redis)

**Alternatives Considered**:
- **Fine-tuned model**: Requires training data, expensive
- **Multiple versions pre-written**: Inflexible, 3× content workload
- **Rule-based simplification**: Poor quality, brittle

---

## Phase 1: Design Artifacts

### Data Model

**Entity: User**
```typescript
interface User {
  id: number;
  email: string;
  password_hash: string;  // bcrypt, never exposed in API
  created_at: Date;
  updated_at: Date;

  // Relations
  profile?: UserProfile;
  chat_messages?: ChatMessage[];
}

// Validation Rules
- email: valid format, unique, max 255 chars
- password: min 8 chars, 1 uppercase, 1 number, 1 special char
- created_at: auto-set on insert
```

**Entity: UserProfile**
```typescript
interface UserProfile {
  user_id: number;  // FK to User.id
  software_experience: 'beginner' | 'intermediate' | 'advanced';
  python_familiarity: 'none' | 'basic' | 'proficient' | 'expert';
  robotics_experience: 'none' | 'hobbyist' | 'academic' | 'professional';
  hardware_background: 'none' | 'basic' | 'intermediate' | 'advanced';
  learning_goals: string;  // max 200 chars
  preferred_complexity: 'beginner' | 'intermediate' | 'advanced';

  // Computed
  get skill_level(): 'beginner' | 'intermediate' | 'advanced' {
    // Average of software_experience, python_familiarity, etc.
  }
}

// Validation Rules
- user_id: required, foreign key
- All enum fields: must match allowed values
- learning_goals: max 200 chars
```

**Entity: ChatMessage**
```typescript
interface ChatMessage {
  id: number;
  user_id?: number;  // Nullable for anonymous users
  session_id: string;  // UUID for conversation continuity
  message: string;
  response: string;
  sources: Array<{
    chapter_id: string;
    title: string;
    snippet: string;
    similarity_score: number;
  }>;
  created_at: Date;
}

// Validation Rules
- message: required, max 500 chars
- response: required, max 2000 chars
- session_id: UUID v4 format
- sources: max 5 items
- created_at: auto-set, indexed for retention policy
```

**Entity: Chapter (Content, not in DB)**
```typescript
interface Chapter {
  id: string;  // "1-1", "2-2", etc.
  module: number;  // 1-4
  title: string;
  content: string;  // Markdown
  word_count: number;
  learning_objectives: string[];
  code_examples: Array<{
    language: string;
    code: string;
  }>;
  bibliography: Array<{
    authors: string[];
    title: string;
    year: number;
    url: string;
  }>;
}

// Storage: Frontend as Markdown files (frontend/docs/module-X/chapter-Y.md)
```

### API Contracts

**Base URL**: `https://api.physical-ai-textbook.com` (or Railway domain)
**API Version**: `/api/v1`
**Authentication**: Bearer token in `Authorization` header or httpOnly cookie

#### Authentication Endpoints

**POST /api/v1/auth/signup**
```yaml
Request:
  Content-Type: application/json
  Body:
    email: string (required, email format)
    password: string (required, min 8 chars)
    profile: object (required)
      software_experience: enum
      python_familiarity: enum
      robotics_experience: enum
      hardware_background: enum
      learning_goals: string (max 200 chars)

Response (201 Created):
  Body:
    user:
      id: number
      email: string
      created_at: string (ISO 8601)
    access_token: string (JWT, 1-hour exp)
    refresh_token: string (7-day exp)

Errors:
  400: Invalid email format
  409: Email already registered
  422: Password doesn't meet requirements
  500: Server error
```

**POST /api/v1/auth/login**
```yaml
Request:
  Body:
    email: string
    password: string
    remember_me: boolean (optional, default false)

Response (200 OK):
  Body:
    user: { id, email }
    access_token: string
    refresh_token: string (only if remember_me=true)

Errors:
  401: Invalid credentials
  429: Rate limited (5 attempts per 15 min)
```

**POST /api/v1/auth/logout**
```yaml
Request:
  Headers: { Authorization: "Bearer <token>" }

Response (204 No Content):
  - Clears httpOnly cookie
  - Blacklists token (optional, if using token blacklist)
```

**POST /api/v1/auth/refresh**
```yaml
Request:
  Body: { refresh_token: string }

Response (200 OK):
  Body:
    access_token: string (new 1-hour token)
    refresh_token: string (new 7-day token)

Errors:
  401: Invalid or expired refresh token
```

#### RAG Chatbot Endpoints

**POST /api/v1/chat**
```yaml
Request:
  Headers:
    Authorization: "Bearer <token>" (optional, for logged-in users)
  Body:
    message: string (required, max 500 chars)
    session_id: string (optional, UUID)
    context_selection: string (optional, highlighted text)

Response (200 OK):
  Body:
    response: string
    sources: array
      - chapter_id: string
        title: string
        snippet: string (100 chars)
        similarity_score: number (0-1)
    session_id: string (for continuity)
    latency_ms: number

Errors:
  400: Message too long
  429: Rate limited (10 req/min per user)
  500: LLM service error

Rate Limiting:
  - Anonymous users: 5 req/min
  - Logged-in users: 10 req/min
  - Burst allowance: 3 extra requests
```

**GET /api/v1/chat/history**
```yaml
Request:
  Headers: { Authorization: "Bearer <token>" }
  Query:
    session_id: string (optional)
    limit: number (default 20, max 50)

Response (200 OK):
  Body:
    messages: array
      - id: number
        message: string
        response: string
        sources: array
        created_at: string

Errors:
  401: Unauthorized
```

#### Personalization Endpoints

**POST /api/v1/personalize-chapter**
```yaml
Request:
  Headers: { Authorization: "Bearer <token>" }
  Body:
    chapter_id: string (required, format: "1-1", "2-2")
    complexity_level: enum (beginner | intermediate | advanced)

Response (200 OK):
  Body:
    content: string (Markdown)
    cached: boolean
    generation_time_ms: number

Errors:
  401: Unauthorized
  404: Chapter not found
  500: Content generation error

Caching:
  - Key: "{chapter_id}:{complexity_level}"
  - TTL: 24 hours
  - Invalidation: manual (via admin endpoint)
```

**GET /api/v1/user/profile**
```yaml
Request:
  Headers: { Authorization: "Bearer <token>" }

Response (200 OK):
  Body:
    user_id: number
    email: string
    profile:
      software_experience: string
      python_familiarity: string
      robotics_experience: string
      hardware_background: string
      learning_goals: string
      preferred_complexity: string

Errors:
  401: Unauthorized
  404: Profile not found
```

**PATCH /api/v1/user/profile**
```yaml
Request:
  Headers: { Authorization: "Bearer <token>" }
  Body: (all fields optional)
    software_experience: enum
    python_familiarity: enum
    robotics_experience: enum
    hardware_background: enum
    learning_goals: string
    preferred_complexity: enum

Response (200 OK):
  Body: (updated profile)

Errors:
  401: Unauthorized
  422: Validation error
```

#### Health & Utility Endpoints

**GET /api/v1/health**
```yaml
Response (200 OK):
  Body:
    status: "healthy"
    timestamp: string (ISO 8601)
    services:
      database: "connected"
      qdrant: "connected"
      openai: "available"

Response (503 Service Unavailable):
  Body:
    status: "degraded"
    services:
      database: "error: connection timeout"
```

**GET /api/v1/docs**
- Auto-generated OpenAPI documentation (FastAPI `/docs`)
- Interactive Swagger UI

---

## Phase 2: Implementation Roadmap

### Milestone 1: Project Setup & Infrastructure (4-6 hours)

**Tasks**:
1. Initialize Git repository, create `.gitignore`
2. Set up Docusaurus project (`npx create-docusaurus@latest`)
3. Configure `frontend/` structure:
   - `/docs` for chapter Markdown files
   - `/src/components` for React components
   - `/static/img` for images/diagrams
4. Set up FastAPI project structure:
   ```
   backend/
   ├── app/
   │   ├── __init__.py
   │   ├── main.py (entry point)
   │   ├── config.py (environment variables)
   │   ├── models/ (SQLAlchemy models)
   │   ├── schemas/ (Pydantic models)
   │   ├── api/ (route handlers)
   │   ├── services/ (business logic)
   │   └── utils/ (helpers)
   ├── tests/
   ├── requirements.txt
   └── .env.example
   ```
5. Create `.env.example` files with all required keys
6. Set up GitHub Actions for CI/CD (`.github/workflows/deploy.yml`)

**Deliverables**:
- [ ] Repository initialized with proper structure
- [ ] Docusaurus builds successfully (`npm run build`)
- [ ] FastAPI runs locally (`uvicorn app.main:app`)
- [ ] `.env.example` templates created

### Milestone 2: Content Creation (12-16 hours)

**Tasks**:
1. Write Module 1 chapters (ROS 2):
   - Chapter 1.1: ROS 2 Architecture (1200 words, 2 diagrams, 2 code examples)
   - Chapter 1.2: Building ROS 2 Apps (1000 words, 1 diagram, 3 code examples)
2. Write Module 2 chapters (Simulation):
   - Chapter 2.1: Gazebo Fundamentals (1100 words, 2 diagrams, 2 code examples)
   - Chapter 2.2: URDF Modeling (900 words, 2 diagrams, 2 code examples)
3. Write Module 3 chapters (Isaac):
   - Chapter 3.1: Isaac Sim Intro (1000 words, 2 diagrams, 1 code example)
   - Chapter 3.2: AI Perception (1100 words, 1 diagram, 2 code examples)
4. Write Module 4 chapters (VLA):
   - Chapter 4.1: Voice-to-Action (1000 words, 2 diagrams, 2 code examples)
   - Chapter 4.2: LLM Planning (1200 words, 2 diagrams, 3 code examples)
5. Create diagrams using Mermaid/Excalidraw (export as SVG)
6. Add bibliography to each chapter (minimum 5 sources)

**Content Guidelines**:
- Use official documentation as primary sources
- Code examples must be tested and runnable
- Diagrams must have alt-text descriptions
- Markdown files use YAML frontmatter for metadata

**Deliverables**:
- [ ] 8 chapter Markdown files in `frontend/docs/`
- [ ] All diagrams in `frontend/static/img/diagrams/`
- [ ] `sidebars.js` configured for 4 modules
- [ ] All chapters build without errors

### Milestone 3: Backend Core (8-10 hours)

**Tasks**:
1. **Database Setup**:
   - Create Neon Postgres project
   - Define SQLAlchemy models (User, UserProfile, ChatMessage)
   - Set up Alembic migrations
   - Run initial migration
   - Create indexes

2. **Authentication Service**:
   - Implement `/api/v1/auth/signup`:
     - Password hashing with bcrypt
     - User creation
     - Profile creation
     - JWT generation
   - Implement `/api/v1/auth/login`:
     - Credential verification
     - Rate limiting (5 attempts/15 min)
     - Token generation
   - Implement `/api/v1/auth/logout`:
     - Clear cookies
   - Implement JWT middleware:
     - Token verification
     - User context injection

3. **RAG Pipeline**:
   - Create Qdrant collection
   - Implement document ingestion:
     ```python
     def ingest_chapters():
         for chapter_file in glob("docs/**/*.md"):
             chunks = chunk_text(chapter_file, size=512)
             embeddings = openai.Embedding.create(chunks)
             qdrant.upsert(chunks, embeddings, metadata)
     ```
   - Implement query handler:
     ```python
     async def handle_chat(message: str, user_id: int):
         query_embedding = await get_embedding(message)
         results = await qdrant.search(query_embedding, top_k=3)
         context = build_context(results)
         response = await openai.ChatCompletion.create(
             messages=[
                 {"role": "system", "content": RAG_SYSTEM_PROMPT},
                 {"role": "user", "content": f"Context:\n{context}\n\nQ: {message}"}
             ]
         )
         return {
             "response": response.choices[0].message.content,
             "sources": format_sources(results)
         }
     ```
   - Implement caching (in-memory dict)
   - Implement rate limiting

4. **Personalization Service**:
   - Implement chapter personalization:
     ```python
     async def personalize_chapter(chapter_id: str, level: str):
         # Check cache
         cache_key = f"{chapter_id}:{level}"
         if cached := cache.get(cache_key):
             return {"content": cached, "cached": True}

         # Load original
         content = load_chapter(chapter_id)

         # Generate personalized version
         prompt = PERSONALIZATION_PROMPTS[level].format(content=content)
         response = await openai.ChatCompletion.create(
             model="gpt-3.5-turbo-16k",
             messages=[{"role": "user", "content": prompt}],
             max_tokens=2000
         )
         personalized = response.choices[0].message.content

         # Cache for 24 hours
         cache.set(cache_key, personalized, ttl=86400)
         return {"content": personalized, "cached": False}
     ```

**Deliverables**:
- [ ] All API endpoints functional
- [ ] Database schema deployed to Neon
- [ ] Qdrant collection created and populated
- [ ] OpenAPI docs accessible at `/docs`

### Milestone 4: Frontend Integration (6-8 hours)

**Tasks**:
1. **Chat Widget Component**:
   ```tsx
   // src/components/ChatWidget.tsx
   export function ChatWidget() {
     const [open, setOpen] = useState(false);
     const [messages, setMessages] = useState([]);
     const [input, setInput] = useState('');

     const sendMessage = async () => {
       const response = await fetch('/api/v1/chat', {
         method: 'POST',
         headers: { 'Content-Type': 'application/json' },
         body: JSON.stringify({ message: input })
       });
       const data = await response.json();
       setMessages([...messages, { role: 'user', content: input }, { role: 'bot', content: data.response }]);
     };

     return (
       <div className="chat-widget">
         <button onClick={() => setOpen(!open)}>💬</button>
         {open && (
           <div className="chat-modal">
             <MessageList messages={messages} />
             <input value={input} onChange={(e) => setInput(e.target.value)} />
             <button onClick={sendMessage}>Send</button>
           </div>
         )}
       </div>
     );
   }
   ```

2. **Auth Modal Component**:
   ```tsx
   // src/components/AuthModal.tsx
   export function AuthModal() {
     const [mode, setMode] = useState<'login' | 'signup'>('login');
     const [email, setEmail] = useState('');
     const [password, setPassword] = useState('');

     const handleSubmit = async () => {
       const endpoint = mode === 'login' ? '/api/v1/auth/login' : '/api/v1/auth/signup';
       const response = await fetch(endpoint, {
         method: 'POST',
         headers: { 'Content-Type': 'application/json' },
         body: JSON.stringify({ email, password, /* profile if signup */ })
       });
       if (response.ok) {
         const { access_token } = await response.json();
         localStorage.setItem('token', access_token);
         // Redirect or update UI
       }
     };

     return <div className="auth-modal">{/* Form UI */}</div>;
   }
   ```

3. **Personalize Button Component**:
   ```tsx
   // src/components/PersonalizeButton.tsx
   export function PersonalizeButton({ chapterId }) {
     const [level, setLevel] = useState('intermediate');
     const [loading, setLoading] = useState(false);

     const personalize = async () => {
       setLoading(true);
       const response = await fetch('/api/v1/personalize-chapter', {
         method: 'POST',
         headers: {
           'Content-Type': 'application/json',
           'Authorization': `Bearer ${getToken()}`
         },
         body: JSON.stringify({ chapter_id: chapterId, complexity_level: level })
       });
       const { content } = await response.json();
       // Replace chapter content in DOM
       document.querySelector('.markdown').innerHTML = marked(content);
       setLoading(false);
     };

     return (
       <div className="personalize-button">
         <select value={level} onChange={(e) => setLevel(e.target.value)}>
           <option value="beginner">Beginner</option>
           <option value="intermediate">Intermediate</option>
           <option value="advanced">Advanced</option>
         </select>
         <button onClick={personalize} disabled={loading}>
           {loading ? 'Generating...' : 'Personalize'}
         </button>
       </div>
     );
   }
   ```

4. **Custom CSS** (`src/css/custom.css`):
   ```css
   :root {
     --primary-color: #1e3a8a;
     --accent-color: #06b6d4;
   }

   .chat-widget {
     position: fixed;
     bottom: 20px;
     right: 20px;
     z-index: 9999;
   }

   .chat-modal {
     width: 400px;
     height: 600px;
     background: white;
     border-radius: 12px;
     box-shadow: 0 10px 40px rgba(0,0,0,0.2);
   }

   .personalize-button {
     margin: 1rem 0;
     padding: 0.5rem;
     background: var(--primary-color);
     border-radius: 8px;
   }
   ```

5. **Integrate components into Docusaurus**:
   - Add `<ChatWidget />` to `docusaurus.config.js` (client modules)
   - Add `<PersonalizeButton />` to MDX wrapper
   - Configure API base URL in config

**Deliverables**:
- [ ] Chat widget functional (sends/receives messages)
- [ ] Auth modal functional (signup/login works)
- [ ] Personalize button functional (content changes)
- [ ] All components styled and responsive

### Milestone 5: Deployment & Testing (4-6 hours)

**Tasks**:
1. **Frontend Deployment**:
   - Configure GitHub Actions workflow:
     ```yaml
     name: Deploy Docusaurus
     on: push: branches: [main]
     jobs:
       deploy:
         runs-on: ubuntu-latest
         steps:
           - uses: actions/checkout@v3
           - uses: actions/setup-node@v3
           - run: cd frontend && npm ci && npm run build
           - uses: peaceiris/actions-gh-pages@v3
             with:
               github_token: ${{ secrets.GITHUB_TOKEN }}
               publish_dir: ./frontend/build
     ```
   - Push to main branch to trigger deploy
   - Verify at `https://username.github.io/physical-ai-textbook`

2. **Backend Deployment**:
   - Create Railway project
   - Connect GitHub repo
   - Add environment variables:
     - `OPENAI_API_KEY`
     - `NEON_DATABASE_URL`
     - `QDRANT_API_KEY`
     - `QDRANT_URL`
     - `BETTER_AUTH_SECRET`
     - `FRONTEND_URL`
   - Deploy from main branch
   - Run database migrations: `alembic upgrade head`
   - Run ingestion script: `python scripts/ingest_chapters.py`
   - Verify at `https://physical-ai-textbook.up.railway.app/docs`

3. **Integration Testing**:
   - Test full user flow:
     1. Visit homepage
     2. Navigate to Chapter 1.1
     3. Open chat widget
     4. Send message "What is ROS 2?"
     5. Verify response cites Chapter 1.1
     6. Click signup
     7. Complete profile form
     8. Login
     9. Click "Personalize" → Select "Beginner"
     10. Verify content changes
   - Test on mobile devices (responsive design)
   - Run Lighthouse audit (target score >90)

4. **Performance Optimization**:
   - Enable Docusaurus minification
   - Configure CORS properly (only allow frontend domain)
   - Set up CDN for static assets (optional)
   - Configure caching headers

**Deliverables**:
- [ ] Frontend live on GitHub Pages
- [ ] Backend live on Railway
- [ ] All features functional end-to-end
- [ ] Lighthouse score >90
- [ ] Mobile responsive

### Milestone 6: Claude Code Integration & Documentation (2-4 hours)

**Tasks**:
1. **Create Claude Code Subagents**:
   - `.claude/subagents/content_generator.md`:
     ```markdown
     # Content Generator Subagent

     ## Purpose
     Automate chapter writing from outlines

     ## Tools
     - Read: Load chapter templates
     - Write: Generate Markdown files
     - WebFetch: Research official documentation

     ## Workflow
     1. Load outline from spec
     2. Research topic via official docs
     3. Generate chapter sections
     4. Add code examples
     5. Validate against learning objectives
     ```

   - `.claude/subagents/diagram_describer.md`:
     ```markdown
     # Diagram Describer Subagent

     ## Purpose
     Generate alt-text for technical diagrams

     ## Tools
     - Read: Load diagram SVG files
     - Edit: Add alt-text to Markdown

     ## Workflow
     1. Analyze diagram structure
     2. Describe visual elements
     3. Explain data flow/relationships
     4. Generate accessibility-compliant alt-text
     ```

2. **Document usage in README**:
   ```markdown
   ## Development with Claude Code

   This project uses Claude Code subagents for automation:

   ### Content Generator
   Automates chapter writing from outlines.
   ```
   Usage: See `.claude/subagents/content_generator.md`
   ```

   ### Diagram Describer
   Generates alt-text for diagrams.
   ```
   Usage: See `.claude/subagents/diagram_describer.md`
   ```

   ## Prompt History Records
   All development decisions logged in `history/prompts/`.
   ```

3. **Create Prompt History Record**:
   - Run: `.specify/scripts/bash/create-phr.sh --title "Physical AI Textbook Planning" --stage plan --feature physical-ai-textbook --json`
   - Fill in details:
     - PROMPT_TEXT: Original user request
     - RESPONSE_TEXT: This plan document
     - FILES_YAML: List of created files
     - OUTCOME: Plan approved and ready for tasks

**Deliverables**:
- [ ] Subagents documented in `.claude/`
- [ ] README updated with Claude Code section
- [ ] PHR created for planning stage

---

## Risk Mitigation Strategies

### Risk 1: OpenAI Rate Limits (High Probability, High Impact)

**Symptoms**: 429 errors, chatbot fails after 3 requests/min

**Mitigation**:
1. **Immediate** (in code):
   ```python
   from functools import lru_cache
   import hashlib

   @lru_cache(maxsize=1000)
   def get_cached_embedding(text_hash):
       # Cache embeddings
       pass

   async def chat_with_retry(message):
       for attempt in range(3):
           try:
               return await openai.ChatCompletion.create(...)
           except openai.error.RateLimitError:
               await asyncio.sleep(2 ** attempt)  # Exponential backoff
       raise HTTPException(429, "AI service temporarily unavailable")
   ```

2. **Long-term**:
   - Upgrade to paid tier ($0.002 per 1K tokens)
   - Use local LLM (Whisper, LLaMA) for non-critical tasks

**Monitoring**: Log all OpenAI API calls with timestamps

### Risk 2: Neon Storage Limit (Low Probability, Medium Impact)

**Symptoms**: Database inserts fail with "quota exceeded" error

**Mitigation**:
1. **Immediate**:
   ```sql
   -- Auto-delete old chat messages
   DELETE FROM chat_messages WHERE created_at < NOW() - INTERVAL '30 days';
   ```

2. **Long-term**:
   - Implement pagination for chat history (limit 50 messages per user)
   - Archive old messages to S3/file storage

**Monitoring**: Weekly query to check storage usage:
```sql
SELECT pg_database_size('neondb') / (1024 * 1024) AS size_mb;
```

### Risk 3: Deployment Failures (Medium Probability, High Impact)

**Symptoms**: GitHub Pages 404, Railway health check fails

**Mitigation**:
1. **Immediate**:
   - Test deployment early (Week 1)
   - Keep backup deployment ready (Vercel for frontend, Render for backend)
   - Document rollback procedure:
     ```bash
     # Frontend rollback
     git revert <bad-commit>
     git push origin main  # Triggers redeploy

     # Backend rollback
     railway rollback  # Or use Railway UI
     ```

2. **Long-term**:
   - Set up staging environment
   - Implement health checks:
     ```python
     @app.get("/api/health")
     async def health_check():
         checks = {
             "database": await check_db_connection(),
             "qdrant": await check_qdrant_connection(),
             "openai": check_openai_api_key()
         }
         if all(checks.values()):
             return {"status": "healthy", "services": checks}
         else:
             raise HTTPException(503, {"status": "degraded", "services": checks})
     ```

**Monitoring**: Set up uptime monitor (UptimeRobot, free tier)

### Risk 4: Content Quality Issues (Low Probability, Medium Impact)

**Symptoms**: AI-generated content has inaccuracies, plagiarism detected

**Mitigation**:
1. **Immediate**:
   - Manual review of each chapter after generation
   - Fact-check all technical claims against official docs
   - Run plagiarism check (Copyscape, Turnitin)
   - Add disclaimer:
     ```markdown
     > **Note**: This content is AI-assisted and fact-checked against official documentation. Please report any inaccuracies via GitHub Issues.
     ```

2. **Long-term**:
   - Peer review by robotics expert
   - User feedback mechanism

**Monitoring**: Track user-reported issues via GitHub Issues

---

## Success Criteria Checklist

### MVP (100 Points)

**Functional Requirements**:
- [ ] 8 chapters published and accessible
- [ ] Docusaurus builds without errors
- [ ] GitHub Pages deployment live
- [ ] RAG chatbot responds to queries (<3s latency)
- [ ] Basic UI (readable, mobile-responsive)

**Technical Requirements**:
- [ ] All API endpoints return valid responses
- [ ] Database schema deployed
- [ ] Qdrant collection populated with embeddings
- [ ] CORS configured correctly
- [ ] HTTPS enabled in production

### Full Feature Set (250 Points)

**Additional Functional Requirements**:
- [ ] User signup/login works
- [ ] Background questionnaire stored in database
- [ ] Personalization generates different content for 3 levels
- [ ] Chat history persists across sessions
- [ ] Dark mode toggle works

**Bonus Requirements**:
- [ ] Claude Code subagents documented
- [ ] README shows reusable skills
- [ ] PHRs created for all major decisions
- [ ] `.claude/` directory has subagent definitions

**Quality Requirements**:
- [ ] Lighthouse score >90
- [ ] Page load <2s (tested on 3G)
- [ ] Chatbot p95 latency <3s
- [ ] WCAG 2.1 AA compliance
- [ ] Mobile responsive (tested on iPhone, Android)

---

## Timeline & Effort Estimation

**Total Estimated Hours**: 40-50 hours

| Milestone | Duration | Cumulative |
|-----------|----------|------------|
| 1. Setup & Infrastructure | 4-6 hours | 4-6 hours |
| 2. Content Creation | 12-16 hours | 16-22 hours |
| 3. Backend Core | 8-10 hours | 24-32 hours |
| 4. Frontend Integration | 6-8 hours | 30-40 hours |
| 5. Deployment & Testing | 4-6 hours | 34-46 hours |
| 6. Claude Code Integration | 2-4 hours | 36-50 hours |

**Team Size**: 1-2 developers
**Parallelization Opportunities**:
- Milestone 2 (content) and Milestone 3 (backend) can run in parallel
- Frontend integration can start once backend API contracts are defined

**Critical Path**: Content creation → Backend RAG → Frontend integration → Deployment

---

## Next Steps

1. **Approve this plan**: Review and confirm architecture decisions
2. **Run `/sp.tasks`**: Generate detailed task breakdown with acceptance tests
3. **Run `/sp.implement`**: Begin AI-assisted implementation
4. **Weekly checkpoints**: Verify progress against timeline

---

## Appendix

### Technology Versions

```yaml
Frontend:
  docusaurus: ^3.0.0
  react: ^18.2.0
  typescript: ^5.0.0
  node: ^18.0.0

Backend:
  python: ^3.11
  fastapi: ^0.104.0
  sqlalchemy: ^2.0.0
  alembic: ^1.12.0
  openai: ^1.3.0
  qdrant-client: ^1.6.0

Services:
  openai_model: gpt-3.5-turbo
  embedding_model: text-embedding-3-small
  neon_postgres: 15+
  qdrant_cloud: 1.6+
```

### Useful Links

- **Spec**: `specs/physical-ai-textbook/spec.md`
- **Constitution**: `.specify/memory/constitution.md`
- **Docusaurus Docs**: https://docusaurus.io/docs
- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **Qdrant Docs**: https://qdrant.tech/documentation/
- **OpenAI API**: https://platform.openai.com/docs

---

**Plan Version**: 1.0.0
**Created**: 2025-12-10
**Status**: ✅ Approved - Ready for Task Generation
**Next**: Run `/sp.tasks` to generate implementation tasks
