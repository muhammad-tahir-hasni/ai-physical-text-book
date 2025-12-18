# Configuration Guide - Physical AI & Humanoid Robotics Platform

This guide explains all the configuration settings you need to manually set up for the RAG chatbot, APIs, and other services.

## Table of Contents
1. [Backend Configuration (.env)](#backend-configuration)
2. [Frontend Configuration (.env)](#frontend-configuration)
3. [RAG Chatbot Setup](#rag-chatbot-setup)
4. [API Keys and Services](#api-keys-and-services)
5. [Database Setup](#database-setup)
6. [Deployment Configuration](#deployment-configuration)

---

## Backend Configuration

### Location
Create/Edit: `/backend/.env` (copy from `/backend/.env.example`)

### Required Settings

#### 1. Database Configuration
```bash
# PostgreSQL connection string
# Local development example:
DATABASE_URL=postgresql+asyncpg://user:password@localhost:5432/physical_ai

# Production (Railway/Neon):
# Use the DATABASE_URL provided by your service
DATABASE_URL=postgresql+asyncpg://user:pass@host:5432/dbname
```

**You need to manually:**
- Install PostgreSQL locally OR sign up for a cloud service (Railway/Neon)
- Create a database named `physical_ai`
- Replace `user`, `password`, `localhost`, `5432` with your actual values

#### 2. Redis Configuration (Optional)
```bash
# For caching - improves performance
REDIS_URL=redis://localhost:6379/0
```

**You need to manually:**
- Install Redis locally: `brew install redis` (macOS) or use cloud service
- Start Redis: `redis-server`
- Or use a cloud Redis service (Railway/Upstash)

#### 3. JWT Secret Keys
```bash
# Generate secure keys with: openssl rand -hex 32
JWT_SECRET_KEY=your-generated-secret-key-here
REFRESH_SECRET_KEY=your-generated-refresh-secret-key-here
```

**You need to manually:**
1. Open terminal and run: `openssl rand -hex 32`
2. Copy the output and paste as `JWT_SECRET_KEY`
3. Run the command again for a different key
4. Paste the second output as `REFRESH_SECRET_KEY`

#### 4. Qdrant Vector Database (Required for RAG)
```bash
QDRANT_URL=https://your-cluster-id.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION_NAME=Humanoid
```

**You need to manually:**
1. Sign up at [https://qdrant.tech/](https://qdrant.tech/)
2. Create a free cluster (1GB free tier)
3. Copy the cluster URL (looks like: `https://abc123.cloud.qdrant.io`)
4. Create an API key from the Qdrant dashboard
5. Paste both values in your `.env` file

**Note:** The collection will be created automatically when you run the embedding script.

#### 5. Cohere API (Optional - for embeddings)
```bash
COHERE_API_KEY=your-cohere-api-key-here
COHERE_MODEL=embed-english-v3.0
```

**You need to manually:**
1. Sign up at [https://cohere.com/](https://cohere.com/)
2. Get a free API key from your dashboard
3. Paste it in your `.env` file

#### 6. OpenAI API (Primary for RAG and Chat)
```bash
OPENAI_API_KEY=sk-your-openai-api-key-here
```

**You need to manually:**
1. Sign up at [https://platform.openai.com/](https://platform.openai.com/)
2. Add billing information (paid service - ~$5-10/month for moderate use)
3. Create an API key from API Keys section
4. Paste it in your `.env` file

**Important:** This is required for the RAG chatbot to work!

#### 7. Claude API (Optional - Alternative to OpenAI)
```bash
CLAUDE_API_KEY=sk-ant-your-claude-api-key-here
```

**You need to manually:**
1. Sign up at [https://console.anthropic.com/](https://console.anthropic.com/)
2. Create an API key
3. Paste it in your `.env` file

#### 8. CORS Configuration
```bash
# Add your frontend URLs (comma-separated)
CORS_ORIGINS=http://localhost:3000,http://localhost:8000

# For production, add your deployed frontend URL:
# CORS_ORIGINS=http://localhost:3000,https://yourdomain.com
```

**You need to manually:**
- Add your production frontend URL when deploying
- Keep localhost URLs for development

#### 9. Application Settings
```bash
APP_NAME=Physical AI Interactive Textbook
APP_VERSION=1.0.0
DEBUG=False  # Set to True for development

FRONTEND_URL=http://localhost:3000  # Update for production
```

---

## Frontend Configuration

### Location
Create/Edit: `/frontend/.env` (copy from `/frontend/.env.example`)

### Required Settings

#### 1. Backend API URL
```bash
# Local development:
REACT_APP_API_URL=http://localhost:8000

# Production:
# REACT_APP_API_URL=https://your-backend.railway.app
```

**You need to manually:**
- Keep as `http://localhost:8000` for local development
- Update to your deployed backend URL for production

#### 2. Application Configuration
```bash
REACT_APP_NAME=Physical AI Interactive Textbook
REACT_APP_ENVIRONMENT=development  # Change to 'production' when deploying
```

#### 3. Feature Flags
```bash
REACT_APP_ENABLE_AUTH=true
REACT_APP_ENABLE_CHAT=true
REACT_APP_ENABLE_PERSONALIZATION=true
```

**You need to manually:**
- Set to `false` to disable any feature
- Set to `true` to enable (default)

#### 4. Deployment Configuration
```bash
# For GitHub Pages deployment:
BASE_URL=/robotic-hackathon

# For custom domain or Vercel:
# BASE_URL=/
```

**You need to manually:**
- Update `BASE_URL` based on your deployment method
- For GitHub Pages: use `/<repository-name>`
- For custom domain: use `/`

---

## RAG Chatbot Setup

### Overview
The RAG (Retrieval-Augmented Generation) chatbot requires:
1. Vector embeddings of your course content
2. Qdrant vector database
3. OpenAI/Claude API for generation

### Step-by-Step Setup

#### Step 1: Configure Vector Database (Qdrant)
Already covered in Backend Configuration - Qdrant section above.

#### Step 2: Generate Embeddings
You need to run the embedding script to populate Qdrant with course content.

**Location:** `/backend/app/utils/chunking.py`

**You need to manually:**
1. Ensure all API keys are configured in `/backend/.env`
2. Run the embedding script:
   ```bash
   cd backend
   python -m app.utils.chunking
   ```
3. This will:
   - Read all markdown files from `/frontend/docs/`
   - Generate embeddings using Cohere or OpenAI
   - Store them in Qdrant vector database

**Note:** This needs to be run once initially, and again whenever you update course content.

#### Step 3: Configure Chat API Endpoints
The chat API is automatically configured when you set up the backend.

**Endpoint:** `POST /api/v1/chat/message`

**Required in .env:**
- `OPENAI_API_KEY` or `CLAUDE_API_KEY`
- `QDRANT_URL` and `QDRANT_API_KEY`

#### Step 4: Test the Chatbot
1. Start the backend: `cd backend && uvicorn app.main:app --reload`
2. Start the frontend: `cd frontend && npm start`
3. Login to the application
4. Click the chat widget (bottom right)
5. Ask a question about the course content

---

## API Keys and Services

### Summary of Required Services

| Service | Purpose | Cost | Required? | Sign Up Link |
|---------|---------|------|-----------|--------------|
| PostgreSQL | User data, profiles | Free tier available | Yes | [Railway](https://railway.app/) or [Neon](https://neon.tech/) |
| Redis | Caching | Free tier available | Optional | [Upstash](https://upstash.com/) |
| Qdrant | Vector database for RAG | Free 1GB | Yes | [qdrant.tech](https://qdrant.tech/) |
| Cohere | Text embeddings | Free tier available | Optional | [cohere.com](https://cohere.com/) |
| OpenAI | Chat completion, embeddings | Paid (~$5-10/mo) | Yes | [platform.openai.com](https://platform.openai.com/) |
| Claude (Anthropic) | Alternative to OpenAI | Paid | Optional | [console.anthropic.com](https://console.anthropic.com/) |

### Cost Estimation
- **Development (Local):** ~$0-5/month (using free tiers)
- **Production (Small Scale):** ~$20-30/month
  - Database: $5-10/month
  - OpenAI API: $5-15/month (depends on usage)
  - Hosting: Free tier (Vercel/Railway)

---

## Database Setup

### PostgreSQL Schema
The schema is automatically created when you run migrations.

**You need to manually:**

1. Install Alembic (already in requirements.txt):
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

2. Run migrations:
   ```bash
   cd backend
   alembic upgrade head
   ```

3. Verify tables were created:
   ```bash
   # Connect to your database and check tables
   psql -d physical_ai -c "\dt"
   ```

### Tables Created
- `users` - User authentication
- `user_profiles` - User background/preferences
- `chat_messages` - Chat history

---

## Deployment Configuration

### Backend Deployment (Railway)

**You need to manually:**

1. Sign up at [Railway](https://railway.app/)
2. Create a new project
3. Add a PostgreSQL database
4. Deploy from GitHub:
   - Connect your repository
   - Set root directory to `/backend`
   - Railway will auto-detect FastAPI

5. Add environment variables in Railway dashboard:
   - Copy all variables from `/backend/.env.example`
   - Paste and fill with your actual values
   - Railway provides `DATABASE_URL` automatically

6. Note your backend URL (e.g., `https://your-app.up.railway.app`)

### Frontend Deployment (Vercel)

**You need to manually:**

1. Sign up at [Vercel](https://vercel.com/)
2. Import your GitHub repository
3. Configure build settings:
   - Framework: Other
   - Root Directory: `frontend`
   - Build Command: `npm run build`
   - Output Directory: `build`

4. Add environment variables:
   - `REACT_APP_API_URL`: Your Railway backend URL
   - Copy other variables from `/frontend/.env.example`

5. Deploy!

---

## Troubleshooting

### Common Issues

#### 1. "Connection refused" when accessing backend
- Check if backend is running: `cd backend && uvicorn app.main:app --reload`
- Verify `REACT_APP_API_URL` in frontend `.env` matches backend URL

#### 2. "Authentication failed" on login
- Check `JWT_SECRET_KEY` and `REFRESH_SECRET_KEY` are set
- Verify database connection
- Check if user exists in database

#### 3. RAG Chatbot not responding
- Verify `OPENAI_API_KEY` is set and valid
- Check if Qdrant is accessible (test with API key)
- Ensure embeddings were generated (run chunking script)
- Check backend logs for errors

#### 4. "No embeddings found" error
- Run the embedding script: `python -m app.utils.chunking`
- Check if Qdrant collection exists in dashboard
- Verify `QDRANT_COLLECTION_NAME` matches in `.env`

#### 5. CORS errors in browser
- Add frontend URL to `CORS_ORIGINS` in backend `.env`
- Restart backend after changing `.env`

---

## Quick Start Checklist

### Backend Setup
- [ ] Copy `/backend/.env.example` to `/backend/.env`
- [ ] Generate JWT secret keys with `openssl rand -hex 32`
- [ ] Sign up for Qdrant and get API key
- [ ] Sign up for OpenAI and get API key
- [ ] Set up PostgreSQL database (local or cloud)
- [ ] Update `DATABASE_URL` in `.env`
- [ ] Install dependencies: `pip install -r requirements.txt`
- [ ] Run migrations: `alembic upgrade head`
- [ ] Generate embeddings: `python -m app.utils.chunking`
- [ ] Start backend: `uvicorn app.main:app --reload`

### Frontend Setup
- [ ] Copy `/frontend/.env.example` to `/frontend/.env`
- [ ] Update `REACT_APP_API_URL` to backend URL
- [ ] Install dependencies: `npm install`
- [ ] Start frontend: `npm start`
- [ ] Test login and signup
- [ ] Test RAG chatbot

---

## Support and Additional Resources

### Documentation
- Backend API docs: `http://localhost:8000/docs` (when running)
- Qdrant docs: [https://qdrant.tech/documentation/](https://qdrant.tech/documentation/)
- OpenAI API docs: [https://platform.openai.com/docs](https://platform.openai.com/docs)

### File Locations
- Backend config: `/backend/.env`
- Frontend config: `/frontend/.env`
- Embedding script: `/backend/app/utils/chunking.py`
- RAG chat API: `/backend/app/api/chat.py`
- Chat widget: `/frontend/src/components/ChatWidget.tsx`

---

**Last Updated:** December 2024

**Note:** Keep your API keys secret! Never commit `.env` files to Git. The `.env.example` files are templates only.
