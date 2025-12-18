# Physical AI & Humanoid Robotics Interactive Textbook

An interactive educational platform teaching Physical AI & Humanoid Robotics fundamentals through an 8-chapter textbook with embedded RAG chatbot, user authentication, and adaptive content personalization.

## Features

- **8-Chapter Textbook**: Comprehensive coverage of ROS 2, Simulation (Gazebo), NVIDIA Isaac Sim, and Vision-Language-Action systems
- **RAG Chatbot**: Intelligent Q&A system with <3s response time powered by OpenAI and Qdrant vector search
- **User Authentication**: Secure signup/login with JWT tokens and background profiling
- **Content Personalization**: Adaptive difficulty levels (Beginner, Intermediate, Advanced)
- **Premium UI**: Dark mode support, mobile-responsive design, and Lighthouse score >90

## Architecture

```
┌─────────────────┐         ┌──────────────────┐         ┌──────────────────┐
│                 │         │                  │         │                  │
│   Docusaurus    │◄────────┤   FastAPI        │◄────────┤   PostgreSQL     │
│   Frontend      │  REST   │   Backend        │         │   (Neon)         │
│   (GitHub Pages)│  API    │   (Railway)      │         │                  │
│                 │         │                  │         └──────────────────┘
└─────────────────┘         └──────────────────┘
                                    │
                                    │
                            ┌───────┴────────┐
                            │                │
                    ┌───────▼──────┐  ┌─────▼─────┐
                    │              │  │           │
                    │   OpenAI     │  │  Qdrant   │
                    │   GPT-3.5    │  │  Vector   │
                    │              │  │  Database │
                    └──────────────┘  └───────────┘
```

## Quick Start

### Prerequisites

- **Node.js** 18+ (for frontend)
- **Python** 3.11+ (for backend)
- **PostgreSQL** (or Neon serverless account)
- **API Keys**: OpenAI, Qdrant (free tier)

### Frontend Setup

```bash
cd frontend
npm install
npm run start  # Development server at http://localhost:3000
```

### Backend Setup

```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with your API keys

# Run database migrations
alembic upgrade head

# Start server
uvicorn app.main:app --reload  # http://localhost:8000
```

### Environment Variables

**Frontend** (`.env`):
```
REACT_APP_API_URL=http://localhost:8000
REACT_APP_ENABLE_AUTH=true
REACT_APP_ENABLE_CHAT=true
```

**Backend** (`.env`):
```
DATABASE_URL=postgresql+asyncpg://user:pass@localhost:5432/dbname
OPENAI_API_KEY=sk-...
QDRANT_URL=https://...
QDRANT_API_KEY=...
JWT_SECRET_KEY=...
```

## Development

### Running Tests

```bash
# Backend tests
cd backend
pytest tests/ -v

# Frontend tests
cd frontend
npm test
```

### Building for Production

```bash
# Frontend
cd frontend
npm run build  # Output in build/

# Backend
cd backend
docker build -t physical-ai-backend .
```

## Development with Claude Code

This project leverages **Claude Code** subagents and skills for accelerated development. The `.claude/` directory contains reusable AI workflows for common tasks.

### Available Subagents

#### 1. Content Generator (`/.claude/subagents/content_generator.md`)

Automatically generates educational content for robotics chapters.

**Usage:**
```bash
# Generate a new chapter
/generate-chapter --module 2 --chapter 1 --topic "Sensor Integration" --level intermediate

# Expand existing outline
/expand-chapter --file frontend/docs/module-3/chapter-3-2-outline.md --target-words 1000
```

**Capabilities:**
- Create complete chapter markdown files (900-1200 words)
- Generate working Python/ROS 2 code examples
- Produce content at different complexity levels
- Follow textbook structure and style guide

#### 2. Diagram Describer (`/.claude/subagents/diagram_describer.md`)

Creates visual representations of robotics concepts using Mermaid diagrams.

**Usage:**
```bash
# Generate architecture diagram
/create-diagram --type architecture --chapter 2-1 --concept "ROS 2 Sensor Pipeline"

# Create flowchart
/diagram-flow --algorithm "SLAM localization" --detail comprehensive
```

**Capabilities:**
- Architecture diagrams (node graphs, system components)
- Sequence diagrams (API flows, authentication)
- State machines (robot behavior, control loops)
- Flowcharts (algorithms, processing pipelines)

### Available Skills

#### 1. RAG Setup (`/.claude/skills/rag_setup.md`)

Automates document ingestion into Qdrant vector database for RAG chatbot.

**Usage:**
```bash
# Initial setup
python scripts/setup_rag.py --docs-dir frontend/docs --chunk-size 800 --overlap 100

# Reset and rebuild
python scripts/setup_rag.py --docs-dir frontend/docs --reset
```

**Workflow:**
1. Document discovery (find all markdown files)
2. Intelligent chunking (800 char chunks, 100 char overlap)
3. Embedding generation (OpenAI text-embedding-3-small)
4. Vector storage (Qdrant with metadata)
5. Validation (test search quality)

#### 2. Auth Setup (`/.claude/skills/auth_setup.md`)

Generates complete authentication system boilerplate.

**Usage:**
```bash
# Generate full auth system
./scripts/setup_auth.sh --project-name "physical-ai-textbook"

# Backend only
python scripts/generate_auth.py --backend-only --output backend/app
```

**Components:**
- Backend: User models, JWT utilities, API endpoints, middleware
- Frontend: Auth context, login/signup modals, protected routes
- Security: Password hashing, token refresh, httpOnly cookies

### Prompt History Records (PHRs)

All major development decisions are documented in **Prompt History Records** located in `history/prompts/`.

**Structure:**
```
history/prompts/
├── physical-ai-textbook/         # Feature-specific PHRs
│   ├── 0001-phase-1-setup.red.prompt.md
│   ├── 0002-phase-2-foundational.green.prompt.md
│   ├── 0004-phase-5-authentication.green.prompt.md
│   └── 0005-phase-6-personalization.green.prompt.md
├── constitution/                  # Project constitution PHRs
└── general/                       # General development PHRs
```

**Creating a PHR:**

1. **Automatic (via Claude Code):**
   - PHRs are automatically created after completing major tasks
   - Includes prompt, response, files changed, and evaluation notes

2. **Manual (via script):**
```bash
.specify/scripts/bash/create-phr.sh \
    --title "Your Feature Title" \
    --stage green \
    --feature physical-ai-textbook \
    --json
```

**PHR Stages:**
- `constitution`: Project principles and guidelines
- `spec`: Feature specifications
- `plan`: Architecture and design decisions
- `tasks`: Task breakdown and acceptance criteria
- `red`: Test-first implementation (TDD)
- `green`: Feature implementation
- `refactor`: Code refactoring and optimization
- `misc`: General development activities

**PHR Benefits:**
- **Traceability**: Track all implementation decisions
- **Knowledge Sharing**: Document reasoning for future developers
- **Reusability**: Learn from past implementations
- **Quality**: Record test results and failure modes

### Example PHR Workflow

1. **Planning Phase**: `/sp.plan` creates plan.md and generates planning PHR
2. **Task Breakdown**: `/sp.tasks` creates tasks.md and generates tasks PHR
3. **Implementation**: `/sp.implement` completes tasks and generates green PHR
4. **Documentation**: `/sp.phr` manually creates PHR for miscellaneous work

View example PHRs:
- `history/prompts/physical-ai-textbook/0004-phase-5-authentication.green.prompt.md`
- `history/prompts/physical-ai-textbook/0005-phase-6-personalization.green.prompt.md`

### Best Practices

1. **Use Subagents**: Delegate specialized tasks (content generation, diagrams) to subagents
2. **Leverage Skills**: Automate repetitive workflows (RAG setup, auth boilerplate)
3. **Document with PHRs**: Record all major decisions for team visibility
4. **Iterate Rapidly**: Use Claude Code for quick prototyping and experimentation

## Deployment

- **Frontend**: Automatically deployed to GitHub Pages via GitHub Actions on push to `main`
- **Backend**: Deploy to Railway with one-click deployment button or manual setup

### Railway Deployment

1. Create Railway project
2. Connect GitHub repository
3. Add environment variables in Railway dashboard
4. Deploy backend service

## Project Structure

```
robotic-hackathon/
├── frontend/                # Docusaurus frontend
│   ├── docs/               # Textbook chapters (Markdown/MDX)
│   ├── src/                # React components (ChatWidget, Auth, etc.)
│   ├── static/             # Images, diagrams, assets
│   └── docusaurus.config.ts
├── backend/                # FastAPI backend
│   ├── app/
│   │   ├── api/           # API endpoints
│   │   ├── models/        # SQLAlchemy models
│   │   ├── schemas/       # Pydantic schemas
│   │   ├── services/      # Business logic (RAG, Auth, etc.)
│   │   └── main.py
│   ├── alembic/           # Database migrations
│   ├── tests/             # Pytest tests
│   └── requirements.txt
├── specs/                 # Feature specifications and plans
├── history/               # Prompt History Records (PHRs)
└── .github/workflows/     # CI/CD workflows
```

## Tech Stack

### Frontend
- **Framework**: Docusaurus 3.x (React-based)
- **Styling**: Infima CSS + custom CSS
- **State Management**: React Context API
- **Deployment**: GitHub Pages

### Backend
- **Framework**: FastAPI 0.104+
- **Database**: PostgreSQL (Neon serverless)
- **Vector DB**: Qdrant (1GB free tier)
- **AI**: OpenAI GPT-3.5-turbo & text-embedding-3-small
- **Auth**: JWT tokens with bcrypt password hashing
- **Deployment**: Railway

## Contributing

This project follows Spec-Driven Development (SDD) workflow:

1. Specifications in `specs/<feature>/spec.md`
2. Implementation plans in `specs/<feature>/plan.md`
3. Task breakdowns in `specs/<feature>/tasks.md`
4. Prompt History Records in `history/prompts/<feature>/`

See `.specify/memory/constitution.md` for development principles.

## License

This project is created for educational purposes as part of a hackathon.

## Acknowledgments

- Built with Claude Code and Spec-Kit Plus
- Powered by Docusaurus, FastAPI, and free-tier AI services
- Content sources: ROS 2 docs, NVIDIA Isaac documentation, Gazebo tutorials

---

**Hackathon Target**: 250/250 points
- Base Features (100 pts): Docusaurus textbook + RAG chatbot
- Bonus Features (150 pts): Claude Code integration (50) + Authentication (50) + Personalization (50)
