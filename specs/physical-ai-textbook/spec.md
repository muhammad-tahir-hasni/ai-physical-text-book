# Feature Specification: Physical AI Interactive Textbook with RAG System

## Metadata
```yaml
feature_id: physical-ai-textbook-rag
feature_name: Physical AI & Humanoid Robotics Interactive Textbook
version: 1.0.0
created: 2025-12-10
status: approved
priority: P0-critical
estimated_effort: 40-50 hours
hackathon_target: 250/250 points
```

## Executive Summary

Build a premium-quality, interactive educational platform teaching Physical AI & Humanoid Robotics fundamentals through a streamlined 8-chapter textbook (4 modules, 4-5 week learning path) with embedded RAG chatbot, user authentication with background profiling, and adaptive content personalization. Deploy as static site (Docusaurus) with FastAPI backend leveraging free-tier AI services.

**Key Differentiators:**
- Token-efficient content design (8 focused chapters vs. 26 full curriculum)
- 100% free-tier infrastructure (OpenAI, Neon, Qdrant, Railway)
- Production-grade RAG with <3s response time
- Intelligent personalization adapting to user background
- Premium UI matching top educational platforms

## Problem Statement

### Current Gap
Students learning Physical AI & Humanoid Robotics face:
1. **Fragmented Resources**: Content scattered across ROS2 docs, Isaac tutorials, academic papers
2. **No Interactive Q&A**: Traditional textbooks lack real-time clarification mechanisms
3. **One-Size-Fits-All**: Content doesn't adapt to learner's existing expertise (software/hardware background)
4. **Poor Accessibility**: Complex robotics concepts presented without visual aids or progressive disclosure

### Target Audience

**Primary**:
- Computer science students (3rd/4th year) with Python basics
- Software engineers transitioning to robotics
- AI researchers exploring embodied intelligence

**Secondary**:
- Hobbyists with electronics background
- Industry professionals upskilling

**Assumptions**:
- Basic programming proficiency (variables, loops, functions)
- Familiarity with Linux command line
- Access to modern web browser (Chrome/Firefox/Safari)
- No robotics or ROS2 prior knowledge required

## Success Metrics

### Hackathon Scoring (250 Total Points)

**Base Features (100 points):**
- ✅ Docusaurus book with 8 well-structured chapters
- ✅ GitHub Pages deployment (public URL)
- ✅ RAG chatbot embedded and functional

**Bonus Features (150 points):**
- ✅ Claude Code Subagents/Skills demonstrating reusable intelligence (50 pts)
- ✅ Authentication system with better-auth.com + background questionnaire (50 pts)
- ✅ Content personalization with 3 difficulty levels (50 pts)

### Quality Metrics
- **Performance**: Lighthouse score >90, page load <2s, chatbot <3s response
- **Accessibility**: WCAG 2.1 AA compliance
- **Content Quality**: 0% plagiarism, minimum 5 credible sources per chapter
- **User Experience**: Intuitive navigation, mobile-responsive, clear visual hierarchy

### Business Metrics (Post-Hackathon)
- User engagement: >20 min average session time
- Chatbot usage: >50% of users interact with chatbot
- Completion rate: >40% users complete all 4 modules

## User Stories

### Epic 1: Content Consumption (Base - 100 pts)

**US-1.1: Browse Structured Content**
*As a learner, I want to navigate through 8 chapters organized by module so I can systematically learn Physical AI fundamentals.*

**Acceptance Criteria:**
- Homepage displays 4 modules with 2 chapters each
- Sidebar navigation shows progress (current chapter highlighted)
- Each chapter takes 30-40 min to read (800-1200 words)
- Mobile-responsive layout (works on phones/tablets)
- Dark mode toggle persists across sessions

**US-1.2: Engage with Rich Media**
*As a visual learner, I want diagrams, code examples, and videos so I can grasp complex robotics concepts.*

**Acceptance Criteria:**
- Minimum 1 technical diagram per chapter (SVG format)
- Code blocks with syntax highlighting (Python, YAML)
- Copy-to-clipboard button for code snippets
- Mermaid diagrams for architecture/data flow
- Image captions and alt-text for accessibility

**US-1.3: Search Content**
*As a returning user, I want to search across all chapters so I can quickly find specific topics.*

**Acceptance Criteria:**
- Search bar in header with autocomplete
- Results show chapter, section, and snippet preview
- Instant search (<500ms response)
- Keyboard shortcuts (Cmd+K / Ctrl+K)

### Epic 2: RAG Chatbot (Base - Included in 100 pts)

**US-2.1: Ask Questions**
*As a confused student, I want to ask the chatbot questions about content so I get instant clarifications without leaving the page.*

**Acceptance Criteria:**
- Floating chat button (bottom-right corner)
- Modal interface with message history
- Responds in <3 seconds (p95 latency)
- Answers cite specific chapters/sections as sources
- Handles follow-up questions with context

**US-2.2: Query Selected Text**
*As a reader encountering unfamiliar terms, I want to highlight text and ask about it so I get targeted explanations.*

**Acceptance Criteria:**
- Text selection triggers "Ask about this" tooltip
- Pre-populates chat with selected text as context
- Response references the selected section
- Works on mobile (long-press selection)

**US-2.3: View Conversation History**
*As a user, I want to review my past conversations so I can revisit explanations.*

**Acceptance Criteria:**
- Chat history persists across page navigations
- Scrollable history (last 20 messages)
- Clear history button
- History resets on logout (privacy)

### Epic 3: Authentication & Profiling (Bonus - 50 pts)

**US-3.1: Create Account**
*As a new user, I want to sign up with email/password so I can access personalized features.*

**Acceptance Criteria:**
- Signup form with email, password, confirm password
- Password strength indicator (minimum 8 chars, 1 uppercase, 1 number)
- Email validation (valid format, no duplicates)
- Success redirect to background questionnaire

**US-3.2: Complete Background Profile**
*As a new user, I want to describe my technical background so the platform can tailor content difficulty.*

**Acceptance Criteria:**
- 5-question form presented after signup:
  1. Software development experience? (Beginner/Intermediate/Advanced)
  2. Python familiarity? (None/Basic/Proficient/Expert)
  3. Robotics experience? (None/Hobbyist/Academic/Professional)
  4. Hardware/electronics background? (None/Basic/Intermediate/Advanced)
  5. Learning goals? (Free text, 200 chars max)
- Skip button (uses default "Intermediate" for all)
- Profile editable later in settings

**US-3.3: Secure Login/Logout**
*As a returning user, I want to securely log in so I access my personalized experience.*

**Acceptance Criteria:**
- Login form with email/password
- "Remember me" checkbox (7-day session)
- Password reset link (email verification)
- Logout clears session and redirects to homepage
- JWT token authentication for API calls

### Epic 4: Content Personalization (Bonus - 50 pts)

**US-4.1: Adjust Chapter Difficulty**
*As a learner, I want to personalize chapter complexity so I learn at my optimal pace.*

**Acceptance Criteria:**
- "Personalize This Chapter" button at top of each chapter
- Dropdown: Beginner, Intermediate (default), Advanced
- Shows loading state (3-5 seconds)
- Content regenerates with adjusted:
  - **Beginner**: More explanations, simpler analogies, step-by-step walkthroughs
  - **Intermediate**: Balanced (as written)
  - **Advanced**: Concise, assumes prerequisites, deeper technical details
- Preference saved for user (applies to future chapter visits)

**US-4.2: Reset Personalization**
*As a user, I want to revert to default content so I can see original chapter.*

**Acceptance Criteria:**
- "Reset to Default" link after personalization applied
- One-click revert (no confirmation needed)
- Updates immediately without page reload

### Epic 5: Claude Code Integration (Bonus - 50 pts)

**US-5.1: Demonstrate Subagents**
*As a hackathon evaluator, I want to see evidence of Claude Code Subagents so I can award bonus points.*

**Acceptance Criteria:**
- Documented in README.md under "Development with Claude Code"
- Minimum 2 custom subagents created:
  1. **Content Generator**: Automates chapter writing from outlines
  2. **Diagram Describer**: Generates alt-text for technical diagrams
- Subagent code/prompts in `.claude/` directory
- PHR (Prompt History Records) showing subagent usage

**US-5.2: Use Agent Skills**
*As a developer, I want reusable skills for common tasks so I accelerate development.*

**Acceptance Criteria:**
- Custom skill for RAG document ingestion pipeline
- Custom skill for better-auth setup and configuration
- Skills documented in `.claude/skills/` directory
- Examples of skill invocation in development logs

## Content Specifications

### Module Structure (8 Chapters, 4-5 Weeks)

#### **Module 1: The Robotic Nervous System (ROS 2)** - Week 1-2

**Chapter 1.1: ROS 2 Architecture & Core Concepts** (1200 words)
- Learning Objectives:
  - Explain the publish-subscribe pattern in distributed robotics systems
  - Differentiate between topics, services, and actions in ROS 2
  - Describe how ROS 2 differs from ROS 1 (DDS middleware, real-time support)
- Content Outline:
  - Why ROS 2? (limitations of ROS 1, need for real-time)
  - Graph architecture: nodes, topics, services, actions
  - Quality of Service (QoS) profiles
  - Practical example: simple publisher-subscriber in Python
- Diagrams: ROS 2 graph diagram, QoS profiles comparison
- Code Example: Minimal publisher/subscriber nodes (rclpy)
- Sources: ROS 2 official docs, Design of ROS 2 (Gerkey et al.), DDS specification

**Chapter 1.2: Building ROS 2 Applications with Python** (1000 words)
- Learning Objectives:
  - Create a ROS 2 package with proper structure
  - Implement nodes using rclpy with timers and callbacks
  - Configure launch files for multi-node systems
- Content Outline:
  - Package anatomy (package.xml, setup.py, CMakeLists.txt)
  - Writing nodes: OOP style vs. functional
  - Launch files: Python launch API
  - Parameters and remapping
- Diagrams: Package structure, node lifecycle
- Code Example: Temperature sensor node with parameter tuning
- Sources: ROS 2 tutorials, rclpy API documentation

#### **Module 2: The Digital Twin (Simulation)** - Week 2-3

**Chapter 2.1: Gazebo Simulation Fundamentals** (1100 words)
- Learning Objectives:
  - Explain the role of physics engines in robotics simulation
  - Set up Gazebo worlds with environmental properties
  - Simulate sensors (LiDAR, cameras, IMU) with realistic noise
- Content Outline:
  - Physics simulation: ODE vs. Bullet vs. DART engines
  - World files: models, lighting, gravity
  - Sensor plugins: configuration and noise injection
  - ROS 2-Gazebo bridge (ros_gz_bridge)
- Diagrams: Gazebo architecture, sensor data flow
- Code Example: Gazebo world with differential drive robot
- Sources: Gazebo documentation, ROS 2 & Gazebo integration guide

**Chapter 2.2: Robot Description with URDF** (900 words)
- Learning Objectives:
  - Author URDF files describing robot kinematics
  - Define links, joints, and collision meshes
  - Load URDF into RViz2 and Gazebo for visualization
- Content Outline:
  - URDF syntax: <robot>, <link>, <joint>, <gazebo>
  - Kinematics: revolute, prismatic, fixed joints
  - Visual vs. collision meshes
  - Xacro for modularity (macros, properties)
- Diagrams: URDF tree structure, joint types
- Code Example: Simple humanoid leg URDF
- Sources: URDF specification, ROS 2 urdf tutorials

#### **Module 3: The AI-Robot Brain (NVIDIA Isaac)** - Week 3-4

**Chapter 3.1: NVIDIA Isaac Sim Introduction** (1000 words)
- Learning Objectives:
  - Describe Isaac Sim's role in photorealistic simulation
  - Set up a basic Isaac environment for robot navigation
  - Generate synthetic training data for perception models
- Content Outline:
  - Isaac platform overview (Sim, ROS, Lab, Gym)
  - Omniverse foundation (USD, RTX ray tracing)
  - Creating scenes: assets, lighting, physics
  - Integration with ROS 2
- Diagrams: Isaac ecosystem, sim-to-real pipeline
- Code Example: Isaac Sim ROS 2 publisher/subscriber
- Sources: NVIDIA Isaac documentation, Omniverse USD guide

**Chapter 3.2: AI-Powered Perception with Isaac ROS** (1100 words)
- Learning Objectives:
  - Implement hardware-accelerated VSLAM for localization
  - Configure Nav2 for humanoid path planning
  - Optimize perception pipelines for edge deployment (Jetson)
- Content Outline:
  - Visual SLAM: feature tracking, loop closure
  - Isaac ROS GEMs: accelerated perception nodes
  - Nav2 architecture: global/local planners, costmaps
  - Edge optimization: TensorRT, INT8 quantization
- Diagrams: VSLAM pipeline, Nav2 architecture
- Code Example: Isaac ROS VSLAM launch file
- Sources: Isaac ROS documentation, Nav2 guides, TensorRT docs

#### **Module 4: Vision-Language-Action (VLA)** - Week 4-5

**Chapter 4.1: Voice-to-Action Systems** (1000 words)
- Learning Objectives:
  - Integrate OpenAI Whisper for speech-to-text on Jetson
  - Parse natural language commands into ROS 2 actions
  - Implement voice confirmation for safety-critical commands
- Content Outline:
  - Speech recognition: Whisper architecture, model sizes
  - Edge deployment: TensorRT optimization for Jetson
  - Command parsing: rule-based vs. LLM-based
  - Safety measures: confirmation prompts, command whitelisting
- Diagrams: Voice-to-action pipeline, Whisper architecture
- Code Example: Whisper + rclpy action client
- Sources: Whisper paper, OpenAI API docs, ROS 2 actions tutorial

**Chapter 4.2: LLM-Based Cognitive Planning** (1200 words)
- Learning Objectives:
  - Use LLMs to decompose high-level goals into action sequences
  - Ground LLM outputs in robot's physical constraints
  - Implement failure recovery and clarification dialogs
- Content Outline:
  - LLMs for robotics: GPT-4, Claude, local LLaMA
  - Prompt engineering: system context, few-shot examples
  - Action grounding: validate feasibility (joint limits, collision)
  - Capstone project walkthrough: "Clean the room" task decomposition
- Diagrams: LLM planning architecture, capstone system diagram
- Code Example: LLM action planner with ROS 2 integration
- Sources: RT-2 paper (Google), SayCan paper, LangChain docs

### Chapter Standards

**Word Count**: 900-1200 words per chapter (total ~8,400 words)
**Reading Time**: 30-40 minutes per chapter
**Sources**: Minimum 5 credible sources per chapter (APA format)
**Code Examples**: 2-3 runnable snippets per chapter (tested)
**Diagrams**: Minimum 1 technical diagram per chapter (SVG/Mermaid)
**Exercises**: 2-3 conceptual questions at end of each chapter

## Technical Architecture

### System Overview

```
┌─────────────────────────────────────────────────────┐
│          User Browser (React + Docusaurus)          │
│  ┌───────────────────────────────────────────────┐  │
│  │  Book UI                                      │  │
│  │  - 8 Chapters, sidebar navigation            │  │
│  │  - Search, dark mode, progress tracking      │  │
│  │  - Chat widget (floating button)             │  │
│  │  - Auth forms (signup/login)                 │  │
│  │  - Personalization controls                  │  │
│  └─────────────────┬─────────────────────────────┘  │
└────────────────────┼────────────────────────────────┘
                     │ HTTPS (REST API + JWT)
          ┌──────────▼──────────────────────────┐
          │   FastAPI Backend (Python 3.11+)    │
          │  ┌──────────────────────────────┐   │
          │  │ API Routes                   │   │
          │  │ - /api/chat (RAG queries)    │   │
          │  │ - /api/auth/* (login/signup) │   │
          │  │ - /api/personalize-chapter   │   │
          │  │ - /api/user/profile          │   │
          │  └───┬──────────┬───────────┬───┘   │
          └──────┼──────────┼───────────┼───────┘
                 │          │           │
        ┌────────▼───┐  ┌───▼────┐  ┌──▼────────┐
        │ OpenAI API │  │ Neon   │  │  Qdrant   │
        │ (GPT-3.5)  │  │Postgres│  │  Cloud    │
        │ Embeddings │  │ Users  │  │  Vectors  │
        │ Generation │  │ Auth   │  │  RAG Docs │
        └────────────┘  └────────┘  └───────────┘
```

### Technology Stack

#### Frontend
- **Framework**: Docusaurus 3.x (React 18+ under the hood)
- **Language**: TypeScript for components, Markdown/MDX for content
- **Styling**: Custom CSS + Infima theme (Docusaurus default)
- **Components**:
  - `ChatWidget.tsx`: Modal chatbot interface with message history
  - `PersonalizeButton.tsx`: Difficulty selector with loading states
  - `AuthModal.tsx`: Signup/login forms with validation
  - `BackgroundQuestionnaire.tsx`: User profile form
- **State Management**: React Context API for global state (auth, chat history)
- **Build**: Static site generation (SSG) with `npm run build`
- **Deployment**: GitHub Pages via GitHub Actions workflow

#### Backend
- **Framework**: FastAPI 0.104+ (async Python web framework)
- **Language**: Python 3.11+
- **ORM**: SQLAlchemy 2.0 (async mode)
- **Authentication**: better-auth.com library (JWT tokens, bcrypt passwords)
- **RAG Stack**:
  - **LLM**: OpenAI GPT-3.5-turbo (via `openai` Python SDK)
  - **Embeddings**: text-embedding-3-small (OpenAI)
  - **Vector DB**: Qdrant Cloud (Python client)
  - **Framework**: LangChain (optional, for RAG orchestration)
- **Caching**: In-memory dict (simple) or Redis (production)
- **Deployment**: Railway or Render (free tier, auto-deploy from Git)

#### Databases

**Neon Serverless Postgres** (Free Tier: 512 MB)
```sql
-- Schema design
CREATE TABLE users (
    id SERIAL PRIMARY KEY,
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

CREATE TABLE user_profiles (
    user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
    software_experience VARCHAR(50),  -- beginner/intermediate/advanced
    python_familiarity VARCHAR(50),   -- none/basic/proficient/expert
    robotics_experience VARCHAR(50),  -- none/hobbyist/academic/professional
    hardware_background VARCHAR(50),  -- none/basic/intermediate/advanced
    learning_goals TEXT,
    preferred_complexity VARCHAR(20) DEFAULT 'intermediate',
    PRIMARY KEY (user_id)
);

CREATE TABLE chat_history (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id) ON DELETE SET NULL,
    session_id VARCHAR(255),
    message TEXT NOT NULL,
    response TEXT NOT NULL,
    sources JSONB,  -- Array of {chapter_id, snippet}
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_chat_user_session ON chat_history(user_id, session_id);
CREATE INDEX idx_user_email ON users(email);
```

**Qdrant Cloud** (Free Tier: 1 GB)
```python
# Collection schema
{
    "vectors": {
        "size": 1536,  # text-embedding-3-small dimension
        "distance": "Cosine"
    },
    "payload_schema": {
        "chapter_id": "keyword",      # e.g., "1-1", "2-2"
        "module": "keyword",           # 1-4
        "title": "text",               # Chapter title
        "content": "text",             # Chunk text (512 tokens)
        "chunk_index": "integer",      # Position in chapter
        "metadata": {
            "word_count": "integer",
            "has_code": "bool",
            "difficulty": "keyword"    # beginner/intermediate/advanced
        }
    }
}
```

### API Specifications

#### Authentication Endpoints

**POST /api/auth/signup**
```typescript
Request:
{
  email: string;              // Valid email format
  password: string;           // Min 8 chars, 1 uppercase, 1 number
  profile: {
    software_experience: "beginner" | "intermediate" | "advanced";
    python_familiarity: "none" | "basic" | "proficient" | "expert";
    robotics_experience: "none" | "hobbyist" | "academic" | "professional";
    hardware_background: "none" | "basic" | "intermediate" | "advanced";
    learning_goals: string;   // Max 200 chars
  }
}

Response (201 Created):
{
  user: {
    id: number;
    email: string;
    created_at: string;       // ISO 8601
  };
  access_token: string;       // JWT, 1-hour expiration
  refresh_token: string;      // 7-day expiration
}

Errors:
400: { error: "invalid_email", message: "Email format invalid" }
409: { error: "email_exists", message: "Email already registered" }
422: { error: "weak_password", message: "Password must meet requirements" }
```

**POST /api/auth/login**
```typescript
Request:
{
  email: string;
  password: string;
  remember_me?: boolean;      // Extends session to 7 days
}

Response (200 OK):
{
  user: { id: number; email: string; };
  access_token: string;
  refresh_token?: string;     // Only if remember_me=true
}

Errors:
401: { error: "invalid_credentials", message: "Incorrect email or password" }
429: { error: "rate_limited", message: "Too many failed attempts. Try again in 15 minutes." }
```

#### RAG Chatbot Endpoints

**POST /api/chat**
```typescript
Request:
{
  message: string;            // User query (max 500 chars)
  user_id?: number;           // Optional, for logged-in users
  session_id?: string;        // For conversation continuity
  context_selection?: string; // Text highlighted by user
}

Response (200 OK):
{
  response: string;           // LLM-generated answer
  sources: Array<{
    chapter_id: string;       // e.g., "2-1"
    title: string;            // Chapter title
    snippet: string;          // Relevant excerpt (100 chars)
    similarity_score: number; // 0-1
  }>;
  session_id: string;         // For follow-up questions
  latency_ms: number;         // Response time
}

Errors:
400: { error: "message_too_long", message: "Query exceeds 500 characters" }
429: { error: "rate_limited", message: "Quota exceeded. Please try again later." }
500: { error: "llm_error", message: "AI service temporarily unavailable" }
```

#### Personalization Endpoint

**POST /api/personalize-chapter**
```typescript
Request:
Headers: { Authorization: "Bearer <jwt_token>" }
{
  chapter_id: string;         // e.g., "3-2"
  complexity_level: "beginner" | "intermediate" | "advanced";
}

Response (200 OK):
{
  content: string;            // Regenerated Markdown
  cached: boolean;            // True if served from cache
  generation_time_ms: number;
}

Errors:
401: { error: "unauthorized", message: "Authentication required" }
404: { error: "chapter_not_found", message: "Chapter ID invalid" }
500: { error: "generation_failed", message: "Content generation error" }
```

### RAG Pipeline Design

**Document Ingestion (One-Time Setup)**
```python
# 1. Chunk each chapter into 512-token segments
chunks = chunk_text(chapter_content, chunk_size=512, overlap=50)

# 2. Generate embeddings
embeddings = openai.Embedding.create(
    model="text-embedding-3-small",
    input=[chunk.text for chunk in chunks]
)

# 3. Store in Qdrant with metadata
qdrant_client.upsert(
    collection_name="physical_ai_textbook",
    points=[
        {
            "id": chunk.id,
            "vector": embedding,
            "payload": {
                "chapter_id": chunk.chapter_id,
                "module": chunk.module,
                "content": chunk.text,
                "chunk_index": chunk.index,
                "metadata": {...}
            }
        }
        for chunk, embedding in zip(chunks, embeddings)
    ]
)
```

**Query Processing (Runtime)**
```python
async def handle_chat_query(message: str, user_id: Optional[int]) -> dict:
    # 1. Embed user query
    query_embedding = await openai.Embedding.acreate(
        model="text-embedding-3-small",
        input=message
    )

    # 2. Retrieve top-3 similar chunks
    results = await qdrant_client.search(
        collection_name="physical_ai_textbook",
        query_vector=query_embedding.data[0].embedding,
        limit=3,
        score_threshold=0.7  # Minimum similarity
    )

    # 3. Build context for LLM
    context = "\n\n".join([
        f"[Chapter {r.payload['chapter_id']}]: {r.payload['content']}"
        for r in results
    ])

    # 4. Generate response with GPT-3.5
    response = await openai.ChatCompletion.acreate(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": RAG_SYSTEM_PROMPT},
            {"role": "user", "content": f"Context:\n{context}\n\nQuestion: {message}"}
        ],
        max_tokens=300,
        temperature=0.7
    )

    # 5. Format response with sources
    return {
        "response": response.choices[0].message.content,
        "sources": [
            {
                "chapter_id": r.payload["chapter_id"],
                "title": r.payload.get("title", ""),
                "snippet": r.payload["content"][:100] + "...",
                "similarity_score": r.score
            }
            for r in results
        ]
    }
```

**Rate Limiting & Caching**
```python
from functools import lru_cache
import hashlib

# In-memory cache for common queries (production: use Redis)
@lru_cache(maxsize=1000)
def get_cached_response(query_hash: str) -> Optional[dict]:
    # Check cache before hitting OpenAI
    pass

async def chat_with_rate_limit(message: str, user_id: int):
    # Check rate limit (10 requests/min per user)
    if not rate_limiter.allow(user_id, max_requests=10, window_seconds=60):
        raise HTTPException(status_code=429, detail="Rate limit exceeded")

    # Check cache
    query_hash = hashlib.md5(message.encode()).hexdigest()
    cached = get_cached_response(query_hash)
    if cached:
        return cached

    # Process query
    result = await handle_chat_query(message, user_id)

    # Cache result
    cache_response(query_hash, result, ttl=3600)  # 1 hour
    return result
```

### Personalization Engine

**Prompt Engineering for Difficulty Levels**
```python
PERSONALIZATION_PROMPTS = {
    "beginner": """
Rewrite the following robotics chapter for absolute beginners:
- Use simple language and everyday analogies
- Explain technical terms in plain English
- Include step-by-step walkthroughs
- Add "Why does this matter?" context for each concept
- Use concrete examples (e.g., "like a thermostat in your home")
- Avoid jargon; when unavoidable, define immediately
Target audience: No prior robotics knowledge, basic programming.

Original chapter:
{chapter_content}

Rewritten chapter (maintain structure, 900-1200 words):
""",
    "intermediate": "{chapter_content}",  # No change, return original
    "advanced": """
Rewrite the following robotics chapter for advanced practitioners:
- Assume strong CS/engineering background
- Use precise technical terminology without over-explanation
- Reference advanced concepts (control theory, optimization, ML)
- Include mathematical formulations where relevant
- Cite cutting-edge research papers (last 3 years)
- Discuss trade-offs, edge cases, and limitations
Target audience: Experienced developers/researchers.

Original chapter:
{chapter_content}

Rewritten chapter (maintain structure, 900-1200 words):
"""
}

async def personalize_chapter(
    chapter_id: str,
    complexity: str,
    user_id: int
) -> str:
    # Check cache first (key: f"{chapter_id}:{complexity}")
    cache_key = f"{chapter_id}:{complexity}"
    cached_content = cache.get(cache_key)
    if cached_content:
        return {"content": cached_content, "cached": True}

    # Load original chapter
    chapter_content = load_chapter_markdown(chapter_id)

    # Generate personalized version
    prompt = PERSONALIZATION_PROMPTS[complexity].format(
        chapter_content=chapter_content
    )

    response = await openai.ChatCompletion.acreate(
        model="gpt-3.5-turbo-16k",  # Longer context for full chapters
        messages=[{"role": "user", "content": prompt}],
        max_tokens=2000,
        temperature=0.7
    )

    personalized_content = response.choices[0].message.content

    # Cache for 24 hours
    cache.set(cache_key, personalized_content, ttl=86400)

    # Save user preference
    await db.update_user_preference(user_id, "complexity", complexity)

    return {"content": personalized_content, "cached": False}
```

## UI/UX Design

### Design System

**Color Palette**
```css
:root {
  /* Primary Colors */
  --primary-blue: #1e3a8a;      /* Trust, technology */
  --primary-purple: #7c3aed;    /* Innovation, AI */
  --accent-cyan: #06b6d4;       /* Futuristic, interactive */

  /* Neutrals */
  --gray-50: #f9fafb;
  --gray-100: #f3f4f6;
  --gray-800: #1f2937;
  --gray-900: #111827;

  /* Semantic Colors */
  --success-green: #10b981;
  --warning-yellow: #f59e0b;
  --error-red: #ef4444;

  /* Dark Mode */
  --dark-bg: #0f172a;
  --dark-surface: #1e293b;
  --dark-text: #e2e8f0;
}
```

**Typography**
```css
/* Headings */
--font-heading: 'Inter', -apple-system, sans-serif;
--heading-weight: 700;

/* Body */
--font-body: 'Source Sans Pro', -apple-system, sans-serif;
--body-weight: 400;
--body-line-height: 1.7;

/* Code */
--font-code: 'Fira Code', 'Monaco', monospace;
--code-size: 0.9em;
```

### Page Layouts

**Homepage (Landing)**
```
┌─────────────────────────────────────────────────┐
│  Header: Logo | Modules | Search | Auth | 🌙   │
├─────────────────────────────────────────────────┤
│                                                 │
│   🤖 Hero Image (Robot in Action)              │
│   ─────────────────────────────────             │
│   Physical AI & Humanoid Robotics              │
│   Master the future of embodied intelligence   │
│                                                 │
│   [Start Learning →]  [View Syllabus]          │
│                                                 │
├─────────────────────────────────────────────────┤
│  📚 What You'll Learn                           │
│  ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐           │
│  │ M1   │ │ M2   │ │ M3   │ │ M4   │           │
│  │ ROS2 │ │ Sim  │ │Isaac │ │ VLA  │           │
│  └──────┘ └──────┘ └──────┘ └──────┘           │
├─────────────────────────────────────────────────┤
│  ✨ Features: RAG Chatbot | Personalization   │
│  📊 4-5 Weeks | 8 Chapters | Interactive      │
├─────────────────────────────────────────────────┤
│  Footer: GitHub | License | Credits            │
└─────────────────────────────────────────────────┘
```

**Chapter Page**
```
┌─────────────────────────────────────────────────┐
│  Header                                    [🔎][👤][🌙]│
├──────┬──────────────────────────────────────────┤
│ 📖   │ Chapter 2.1: Gazebo Fundamentals        │
│ M1   │ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━    │
│ 1.1  │                                          │
│ 1.2 ◀│ 🎨 Personalize: [Beginner][Inter▼][Adv] │
│ M2   │                                          │
│ 2.1 ●│ ## Learning Objectives                   │
│ 2.2  │ - Explain physics simulation...          │
│ M3   │                                          │
│ 3.1  │ ## Introduction                          │
│ 3.2  │ Lorem ipsum robotics simulation...      │
│ M4   │                                          │
│ 4.1  │ ```python                                │
│ 4.2  │ import rclpy                             │
│      │ ```                                      │
│      │                                          │
│      │ [◀ Prev]              [Next ▶]          │
│      │                                     [💬] │
└──────┴──────────────────────────────────────────┘
```

**Chat Modal**
```
┌─────────────────────────────────────────┐
│  💬 AI Assistant              [✕]      │
├─────────────────────────────────────────┤
│                                         │
│  👤 What is a ROS 2 node?               │
│                                         │
│  🤖 A ROS 2 node is a process that...   │
│     📖 Source: Chapter 1.1              │
│                                         │
│  👤 Can you give an example?            │
│                                         │
│  🤖 Sure! Here's a minimal node:        │
│     ```python                           │
│     import rclpy                        │
│     ```                                 │
│     📖 Source: Chapter 1.2              │
│                                         │
├─────────────────────────────────────────┤
│  [Type your question...]       [Send→] │
└─────────────────────────────────────────┘
```

### Responsive Breakpoints

```css
/* Mobile First */
@media (max-width: 640px) {
  /* Stack sidebar, hide on scroll */
  .sidebar { position: fixed; transform: translateX(-100%); }
  .content { width: 100%; padding: 1rem; }
  .chat-widget { bottom: 60px; right: 10px; }
}

@media (min-width: 641px) and (max-width: 1024px) {
  /* Collapsible sidebar */
  .sidebar { width: 200px; }
  .content { margin-left: 200px; }
}

@media (min-width: 1025px) {
  /* Full layout */
  .sidebar { width: 280px; position: sticky; }
  .content { max-width: 900px; margin: 0 auto; }
}
```

### Accessibility Features

- **Keyboard Navigation**:
  - Tab through all interactive elements
  - Esc to close modals
  - Cmd/Ctrl+K for search
- **Screen Readers**:
  - ARIA labels on all icons
  - Alt text for all images
  - Heading hierarchy (H1 → H2 → H3)
- **Color Contrast**:
  - Text: 4.5:1 minimum (WCAG AA)
  - Large text: 3:1 minimum
- **Focus Indicators**: Visible outlines on focused elements
- **Skip Links**: "Skip to main content" for screen readers

## Deployment Strategy

### Frontend Deployment (GitHub Pages)

**GitHub Actions Workflow** (`.github/workflows/deploy.yml`)
```yaml
name: Deploy Docusaurus

on:
  push:
    branches: [main]
  workflow_dispatch:

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      - name: Setup Node.js
        uses: actions/setup-node@v3
        with:
          node-version: '18'
          cache: 'npm'
          cache-dependency-path: frontend/package-lock.json

      - name: Install dependencies
        run: |
          cd frontend
          npm ci

      - name: Build Docusaurus
        run: |
          cd frontend
          npm run build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./frontend/build
          cname: yourdomain.com  # Optional custom domain
```

**Deployment Steps**:
1. Push to `main` branch triggers workflow
2. Actions runner installs dependencies
3. Builds static site (`npm run build`)
4. Deploys to `gh-pages` branch
5. GitHub Pages serves from `gh-pages`
6. Live at: `https://username.github.io/physical-ai-textbook`

### Backend Deployment (Railway)

**railway.json** (Root of repo)
```json
{
  "$schema": "https://railway.app/railway.schema.json",
  "build": {
    "builder": "NIXPACKS",
    "buildCommand": "pip install -r backend/requirements.txt"
  },
  "deploy": {
    "startCommand": "cd backend && uvicorn app.main:app --host 0.0.0.0 --port $PORT",
    "healthcheckPath": "/api/health",
    "healthcheckTimeout": 100,
    "restartPolicyType": "ON_FAILURE",
    "restartPolicyMaxRetries": 10
  }
}
```

**Deployment Steps**:
1. Connect GitHub repo to Railway project
2. Set environment variables in Railway dashboard:
   - `OPENAI_API_KEY`
   - `NEON_DATABASE_URL`
   - `QDRANT_API_KEY`
   - `QDRANT_URL`
   - `BETTER_AUTH_SECRET`
   - `FRONTEND_URL` (GitHub Pages URL)
3. Railway auto-deploys on push to `main`
4. Backend accessible at: `https://physical-ai-textbook-production.up.railway.app`

**Alternative: Render** (If Railway quota exhausted)
- Same environment variables
- Use `render.yaml` instead of `railway.json`
- Free tier: 750 hours/month

### Environment Configuration

**.env.example** (Backend)
```bash
# OpenAI Configuration
OPENAI_API_KEY=sk-proj-xxxxx
OPENAI_MODEL=gpt-3.5-turbo
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

# Neon Postgres
NEON_DATABASE_URL=postgresql://user:pass@ep-xxx.region.aws.neon.tech/dbname?sslmode=require

# Qdrant Cloud
QDRANT_URL=https://xxx-xxx.us-east.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-key
QDRANT_COLLECTION_NAME=physical_ai_textbook

# Better Auth
BETTER_AUTH_SECRET=generate-with-openssl-rand-base64-32
SESSION_MAX_AGE=3600  # 1 hour (seconds)

# Application
FRONTEND_URL=https://yourusername.github.io/physical-ai-textbook
BACKEND_URL=https://your-backend.up.railway.app
CORS_ORIGINS=http://localhost:3000,https://yourusername.github.io

# Rate Limiting
RATE_LIMIT_REQUESTS=10
RATE_LIMIT_WINDOW=60  # seconds

# Logging
LOG_LEVEL=INFO
DEBUG_MODE=false
```

### Database Initialization

**Automated Migration Script** (`backend/init_db.py`)
```python
import asyncio
from sqlalchemy.ext.asyncio import create_async_engine
from app.models import Base
from app.services.rag import ingest_chapters
import os

async def init_database():
    # 1. Create tables
    engine = create_async_engine(os.getenv("NEON_DATABASE_URL"))
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
    print("✅ Database tables created")

    # 2. Ingest chapter content into Qdrant
    await ingest_chapters(chapters_dir="frontend/docs")
    print("✅ RAG embeddings indexed")

    # 3. Create test user (optional)
    # await create_test_user(email="demo@example.com", password="Demo123!")
    # print("✅ Test user created")

if __name__ == "__main__":
    asyncio.run(init_database())
```

**Run once after deployment**:
```bash
python backend/init_db.py
```

## Testing Strategy

### Manual Testing Checklist

**Functional Tests**:
- [ ] All 8 chapters load without errors
- [ ] Sidebar navigation works (click each chapter)
- [ ] Search returns relevant results (test: "ROS 2 nodes")
- [ ] Dark mode toggles correctly
- [ ] Chat widget opens/closes
- [ ] Chatbot responds to query in <3s
- [ ] Text selection triggers "Ask about this"
- [ ] Signup creates user (check database)
- [ ] Login works with correct credentials
- [ ] Login fails with wrong password (shows error)
- [ ] Personalize button generates content
- [ ] Content changes between Beginner/Advanced

**Performance Tests**:
- [ ] Run Lighthouse audit (target score >90)
- [ ] Measure page load time (target <2s)
- [ ] Test chatbot latency 10 times (p95 <3s)
- [ ] Check mobile responsiveness (iPhone, Android)

**Security Tests**:
- [ ] API keys not in git history (`git log -S 'sk-'`)
- [ ] HTTPS enabled in production
- [ ] CORS allows only frontend domain
- [ ] SQL injection test (try `' OR 1=1--` in login)
- [ ] XSS test (try `<script>alert('xss')</script>` in chat)

**Accessibility Tests**:
- [ ] Tab through all interactive elements
- [ ] Test with screen reader (NVDA/VoiceOver)
- [ ] Check color contrast (use browser DevTools)
- [ ] Verify ARIA labels on icons
- [ ] Test keyboard shortcuts (Cmd+K for search)

### Automated Testing (Optional for Hackathon)

**Backend Unit Tests** (pytest)
```python
# backend/tests/test_rag.py
import pytest
from app.services.rag import handle_chat_query

@pytest.mark.asyncio
async def test_chat_query():
    response = await handle_chat_query(
        message="What is ROS 2?",
        user_id=None
    )
    assert "ROS 2" in response["response"]
    assert len(response["sources"]) > 0
    assert response["sources"][0]["chapter_id"] in ["1-1", "1-2"]

@pytest.mark.asyncio
async def test_rate_limiting():
    user_id = 1
    for i in range(10):
        await handle_chat_query(f"Test query {i}", user_id)

    # 11th request should fail
    with pytest.raises(HTTPException) as exc:
        await handle_chat_query("Test query 11", user_id)
    assert exc.value.status_code == 429
```

**Frontend E2E Tests** (Playwright - Optional)
```javascript
// frontend/tests/e2e/chat.spec.ts
import { test, expect } from '@playwright/test';

test('chatbot responds to query', async ({ page }) => {
  await page.goto('http://localhost:3000');

  // Open chat widget
  await page.click('[data-testid="chat-button"]');

  // Send message
  await page.fill('[data-testid="chat-input"]', 'What is ROS 2?');
  await page.click('[data-testid="chat-send"]');

  // Wait for response
  await page.waitForSelector('[data-testid="bot-message"]', { timeout: 5000 });

  // Verify response contains "ROS 2"
  const response = await page.textContent('[data-testid="bot-message"]');
  expect(response).toContain('ROS 2');
});
```

## Risk Management

### Technical Risks

**Risk 1: API Rate Limits Exceeded**
- **Probability**: Medium (OpenAI free tier: 3 RPM)
- **Impact**: High (chatbot unusable)
- **Mitigation**:
  1. Implement request queue with exponential backoff
  2. Cache common queries (in-memory dict or Redis)
  3. Show user-friendly message: "AI is processing... may take longer due to high demand"
  4. Fallback to simpler responses (no LLM) if quota exhausted

**Risk 2: Free Tier Storage Limits**
- **Probability**: Low (Neon 512 MB, Qdrant 1 GB generous for hackathon)
- **Impact**: Medium (can't store more users/chat history)
- **Mitigation**:
  1. Limit chat history to last 50 messages per user
  2. Auto-delete chat history >30 days old
  3. Efficient embeddings (quantization, smaller chunks)
  4. Monitor usage dashboard weekly

**Risk 3: Deployment Failures**
- **Probability**: Medium (first-time setup complexity)
- **Impact**: High (project not accessible for evaluation)
- **Mitigation**:
  1. Test deployment early (Week 1 of hackathon)
  2. Document rollback procedure (Git revert + redeploy)
  3. Have backup deployment option (Vercel for frontend, Render for backend)
  4. Screenshot working deployment as proof

**Risk 4: Content Quality Issues**
- **Probability**: Low (AI-generated content may have inaccuracies)
- **Impact**: Medium (affects educational value)
- **Mitigation**:
  1. Cite official documentation for all technical claims
  2. Manual review of each chapter after generation
  3. Plagiarism check with Turnitin or Copyscape
  4. Peer review by teammate with robotics background

### Project Risks

**Risk 5: Scope Creep**
- **Probability**: High (temptation to add features)
- **Impact**: High (miss hackathon deadline)
- **Mitigation**:
  1. Strictly adhere to 8 chapters (no additional content)
  2. Defer nice-to-haves: video tutorials, progress tracking, certificates
  3. Use TodoWrite tool to track planned tasks
  4. Weekly checkpoint: are we on track for all 250 points?

**Risk 6: Claude Code/Spec-Kit Plus Learning Curve**
- **Probability**: Medium (new workflow for many)
- **Impact**: Medium (slower initial progress)
- **Mitigation**:
  1. Dedicate Day 1 to workflow learning
  2. Follow official Spec-Kit Plus examples
  3. Create PHRs for all work (builds muscle memory)
  4. Ask for help in hackathon Discord/forums

## Success Criteria

### Minimum Viable Product (MVP) - 100 Points

**Must Have**:
- ✅ 8 chapters published (2 per module)
- ✅ Docusaurus builds without errors
- ✅ Deployed to GitHub Pages (public URL)
- ✅ RAG chatbot responds to queries
- ✅ Basic UI (readable, mobile-responsive)

**Acceptance Test**:
```bash
# Build succeeds
cd frontend && npm run build  # Exit code 0

# Deployment live
curl https://username.github.io/physical-ai-textbook  # HTTP 200

# Chatbot functional
curl -X POST https://backend.railway.app/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?"}' \
  # Returns JSON with "response" and "sources"
```

### Full Feature Set - 250 Points

**Must Have**:
- ✅ All MVP features
- ✅ Authentication (signup/login works)
- ✅ Background questionnaire stored
- ✅ Personalization generates different content
- ✅ Claude Subagents documented in README
- ✅ Premium UI (polished design, animations)

**Acceptance Test**:
```bash
# Auth works
curl -X POST https://backend.railway.app/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "Test123!", "profile": {...}}' \
  # Returns user and access_token

# Personalization works
curl -X POST https://backend.railway.app/api/personalize-chapter \
  -H "Authorization: Bearer <token>" \
  -d '{"chapter_id": "1-1", "complexity_level": "beginner"}' \
  # Returns personalized content (different from intermediate)

# Subagents exist
ls .claude/subagents/  # Shows content_generator.md, diagram_describer.md
```

### Quality Benchmarks

**Performance** (All Must Pass):
- Lighthouse score >90 (run in Chrome DevTools)
- Page load <2s on 3G throttling
- Chatbot p95 latency <3s (test 20 queries)

**Security** (All Must Pass):
- No API keys in git history
- HTTPS enabled in production
- CORS restricted to frontend domain
- Input validation on all endpoints

**Accessibility** (All Must Pass):
- WCAG 2.1 AA compliance (axe DevTools scan)
- Keyboard navigation works (tab through site)
- Color contrast 4.5:1 minimum
- Screen reader friendly (test with NVDA)

## Out of Scope

**Explicitly Excluded** (to prevent scope creep):
- ❌ Video tutorials or screencasts
- ❌ Physical robot hardware integration
- ❌ Live instructor/tutor chat
- ❌ Payment or subscription system
- ❌ Mobile native apps (iOS/Android)
- ❌ Full 13-week curriculum (only 4-5 weeks)
- ❌ Progress tracking and certificates
- ❌ Community features (forums, comments, user-generated content)
- ❌ Advanced Isaac features (sim-to-real, RL training)
- ❌ Multi-language support (English only)
- ❌ Advanced analytics (user behavior tracking)
- ❌ Email notifications
- ❌ Social login (GitHub/Google OAuth) - only email/password
- ❌ Real-time collaboration (shared notes, annotations)

## Appendix

### Glossary

- **RAG (Retrieval-Augmented Generation)**: AI technique combining document retrieval with LLM generation
- **ROS 2 (Robot Operating System 2)**: Middleware framework for robot software development
- **URDF (Unified Robot Description Format)**: XML format for robot kinematic models
- **VSLAM (Visual SLAM)**: Simultaneous Localization and Mapping using camera input
- **VLA (Vision-Language-Action)**: AI models mapping vision+language inputs to robot actions
- **Qdrant**: Open-source vector database for semantic search
- **Neon**: Serverless Postgres database with generous free tier
- **Docusaurus**: Static site generator for technical documentation (React-based)
- **better-auth**: Open-source authentication library for Node.js/Python
- **JWT (JSON Web Token)**: Compact, URL-safe token for API authentication

### References

**Technical Documentation**:
- ROS 2 Documentation: https://docs.ros.org/en/humble/
- NVIDIA Isaac Sim: https://developer.nvidia.com/isaac-sim
- Gazebo: https://gazebosim.org/docs
- Docusaurus: https://docusaurus.io/docs
- FastAPI: https://fastapi.tiangolo.com/
- OpenAI API: https://platform.openai.com/docs
- Qdrant: https://qdrant.tech/documentation/
- Neon: https://neon.tech/docs/introduction
- better-auth: https://www.better-auth.com/docs

**Academic Papers**:
- "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control" (Google, 2023)
- "SayCan: Grounding Language in Robotic Affordances" (Google, 2022)
- "Whisper: Robust Speech Recognition via Large-Scale Weak Supervision" (OpenAI, 2022)
- "Retrieval-Augmented Generation for Knowledge-Intensive NLP Tasks" (Lewis et al., 2020)

**Spec-Kit Plus**:
- GitHub: https://github.com/panaversity/spec-kit-plus/
- Documentation: `.specify/templates/` in this repo

### Constitution Alignment

This feature specification adheres to all principles defined in `.specify/memory/constitution.md`:

**I. Spec-Driven Development**: ✅ This document defines scope, success criteria, acceptance tests
**II. Academic Rigor**: ✅ Content will cite official docs + academic papers (minimum 5 per chapter)
**III. Zero-Plagiarism**: ✅ AI-generated content will be fact-checked and properly attributed
**IV. AI-Driven Workflow**: ✅ Using Claude Code + Spec-Kit Plus (`/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement`)
**V. Docusaurus-First**: ✅ All content in Markdown/MDX, sidebars.js navigation, GitHub Pages deployment
**VI. Technical Precision**: ✅ No vague claims, quantitative benchmarks (latency, Lighthouse scores), trade-off analysis

### Next Steps

**Immediate Actions**:
1. ✅ **Approve Spec**: Review this document, confirm scope with team
2. 🔄 **Run /sp.plan**: Generate implementation plan (architecture, tech decisions, ADRs)
3. ⏳ **Run /sp.tasks**: Break down into actionable tasks with acceptance tests
4. ⏳ **Run /sp.implement**: Begin AI-assisted development

**Timeline Estimate**:
- Planning: 2-4 hours
- Implementation: 30-40 hours (distributed across team)
- Testing & Polish: 6-8 hours
- **Total**: ~40-50 hours for full 250-point solution

---

**Specification Version**: 1.0.0
**Author**: Claude Code Agent with Spec-Kit Plus
**Date**: 2025-12-10
**Status**: ✅ Ready for Planning Phase
**Hackathon Target**: 250/250 Points (Base 100 + All Bonuses 150)
