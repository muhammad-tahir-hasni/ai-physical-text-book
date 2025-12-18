# Development Setup Guide

This guide walks through setting up the complete development environment for the Physical AI Interactive Textbook project.

## Prerequisites Installation

### 1. Install Node.js (Frontend)
```bash
# macOS (using Homebrew)
brew install node@18

# Ubuntu/Debian
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs

# Verify installation
node --version  # Should be 18+
npm --version
```

### 2. Install Python (Backend)
```bash
# macOS (using Homebrew)
brew install python@3.11

# Ubuntu/Debian
sudo apt-get update
sudo apt-get install python3.11 python3.11-venv python3-pip

# Verify installation
python3 --version  # Should be 3.11+
```

### 3. Install PostgreSQL (Optional - use Neon for cloud)
```bash
# macOS
brew install postgresql@15
brew services start postgresql@15

# Ubuntu/Debian
sudo apt-get install postgresql-15

# Or sign up for Neon (recommended for free tier)
# Visit: https://neon.tech/
```

## Project Setup

### 1. Clone Repository
```bash
git clone https://github.com/yourusername/robotic-hackathon.git
cd robotic-hackathon
```

### 2. Frontend Setup

```bash
cd frontend

# Install dependencies
npm install

# Create environment file
cp .env.example .env

# Start development server
npm run start  # http://localhost:3000
```

### 3. Backend Setup

```bash
cd backend

# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate  # macOS/Linux
# OR
venv\Scripts\activate  # Windows

# Install dependencies
pip install -r requirements.txt
pip install -r requirements-dev.txt  # For development tools

# Create environment file
cp .env.example .env

# Edit .env with your API keys
nano .env  # or use your preferred editor
```

### 4. Configure Environment Variables

#### Backend `.env` file:
```env
# Database
DATABASE_URL=postgresql+asyncpg://user:password@localhost:5432/physical_ai

# OpenAI
OPENAI_API_KEY=sk-...  # Get from https://platform.openai.com/api-keys

# Qdrant
QDRANT_URL=https://...  # Get from https://cloud.qdrant.io/
QDRANT_API_KEY=...
QDRANT_COLLECTION_NAME=physical_ai_textbook

# JWT Secret (generate with: openssl rand -hex 32)
JWT_SECRET_KEY=your-secret-key-here
REFRESH_SECRET_KEY=another-secret-key-here
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=60
REFRESH_TOKEN_EXPIRE_DAYS=7

# CORS
CORS_ORIGINS=http://localhost:3000,http://localhost:8000

# Application
APP_NAME=Physical AI Hackathon Platform
DEBUG=True
```

#### Frontend `.env` file:
```env
REACT_APP_API_URL=http://localhost:8000
REACT_APP_ENABLE_AUTH=true
REACT_APP_ENABLE_CHAT=true
REACT_APP_ENABLE_PERSONALIZATION=true
```

### 5. Database Setup

```bash
cd backend

# Activate virtual environment if not already activated
source venv/bin/activate

# Run database migrations
alembic upgrade head

# Verify tables were created
# Connect to your database and check tables
```

### 6. Set Up Pre-commit Hooks (Optional but Recommended)

```bash
# Install pre-commit (from project root)
pip install pre-commit

# Install git hooks
pre-commit install

# Test hooks (optional)
pre-commit run --all-files
```

This will automatically format code (Black for Python, Prettier for TypeScript) before each commit.

## Running the Application

### Development Mode

**Terminal 1 - Backend**:
```bash
cd backend
source venv/bin/activate
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

**Terminal 2 - Frontend**:
```bash
cd frontend
npm run start
```

Access the application at `http://localhost:3000`

### API Documentation

Once the backend is running, visit:
- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`

## Testing

### Backend Tests
```bash
cd backend
source venv/bin/activate
pytest tests/ -v

# With coverage
pytest tests/ --cov=app --cov-report=html
```

### Frontend Tests
```bash
cd frontend
npm test

# With coverage
npm test -- --coverage
```

## Common Issues

### Issue: Port already in use
```bash
# Kill process on port 8000 (backend)
lsof -ti:8000 | xargs kill -9

# Kill process on port 3000 (frontend)
lsof -ti:3000 | xargs kill -9
```

### Issue: Python module not found
```bash
# Ensure virtual environment is activated
source venv/bin/activate

# Reinstall dependencies
pip install -r requirements.txt
```

### Issue: npm install fails
```bash
# Clear npm cache
npm cache clean --force

# Delete node_modules and package-lock.json
rm -rf node_modules package-lock.json

# Reinstall
npm install
```

### Issue: Database connection error
- Verify DATABASE_URL in `.env`
- Ensure PostgreSQL is running: `pg_isready`
- Check firewall settings
- For Neon: Verify connection string from dashboard

## Next Steps

After setup is complete:

1. **Populate RAG Database**: Run the document ingestion script
   ```bash
   cd backend
   python scripts/ingest_chapters.py
   ```

2. **Create Test User**: Use the API or admin script
   ```bash
   python scripts/create_test_user.py
   ```

3. **Start Development**: Follow the Spec-Driven Development workflow
   - Review specs in `specs/physical-ai-textbook/`
   - Check tasks in `specs/physical-ai-textbook/tasks.md`
   - Implement features incrementally

## Useful Commands

```bash
# Frontend
npm run build          # Production build
npm run serve          # Serve production build locally
npm run clear          # Clear Docusaurus cache

# Backend
alembic revision -m "description"  # Create new migration
alembic upgrade head               # Apply migrations
alembic downgrade -1               # Rollback one migration
black app/                         # Format Python code
pytest tests/ -k test_name         # Run specific test
```

## Resources

- [Docusaurus Docs](https://docusaurus.io/docs)
- [FastAPI Docs](https://fastapi.tiangolo.com/)
- [Alembic Docs](https://alembic.sqlalchemy.org/)
- [Qdrant Docs](https://qdrant.tech/documentation/)
- [OpenAI API Docs](https://platform.openai.com/docs)

---

For questions or issues, check the project's GitHub Issues or consult the development team.
