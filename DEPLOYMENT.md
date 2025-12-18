# Deployment Guide

Complete guide to deploying the Physical AI Interactive Textbook to production.

## Architecture Overview

- **Frontend**: Docusaurus static site deployed to GitHub Pages
- **Backend**: FastAPI application deployed to Railway
- **Database**: PostgreSQL (Neon serverless)
- **Vector DB**: Qdrant Cloud (free tier)
- **AI Services**: OpenAI API

## Prerequisites

Before deploying, ensure you have:

1. **GitHub Account** (for GitHub Pages hosting)
2. **Railway Account** (sign up at [railway.app](https://railway.app))
3. **Neon Account** (serverless PostgreSQL at [neon.tech](https://neon.tech))
4. **Qdrant Account** (vector database at [qdrant.tech](https://qdrant.tech))
5. **OpenAI API Key** (from [platform.openai.com](https://platform.openai.com))

---

## Part 1: Database Setup (Neon PostgreSQL)

### Step 1: Create Neon Project

1. Go to [neon.tech](https://neon.tech) and sign in
2. Click "Create Project"
3. Name: `physical-ai-textbook`
4. Region: Choose closest to your users
5. PostgreSQL version: 15 or 16
6. Click "Create Project"

### Step 2: Get Connection String

1. In your Neon dashboard, click "Connection Details"
2. Copy the connection string (starts with `postgresql://`)
3. **Important**: Add `+asyncpg` after `postgresql` for async support
   - Example: `postgresql+asyncpg://user:pass@host/db`
4. Save this for later (Railway environment variables)

---

## Part 2: Vector Database Setup (Qdrant)

### Step 1: Create Qdrant Cluster

1. Go to [qdrant.tech](https://qdrant.tech) and sign in
2. Click "Create Cluster"
3. Choose "Free" tier (1GB storage)
4. Region: Choose closest to your users
5. Cluster name: `physical-ai-docs`
6. Click "Create"

### Step 2: Get API Credentials

1. Once cluster is created, click "API Keys"
2. Copy the cluster URL (e.g., `https://xxx.region.cloud.qdrant.io`)
3. Copy the API key
4. Save both for later

### Step 3: Create Collection

1. In Qdrant dashboard, click "Collections"
2. Click "Create Collection"
3. Name: `physical_ai_docs`
4. Vector size: `1536` (for OpenAI text-embedding-3-small)
5. Distance: `Cosine`
6. Click "Create"

---

## Part 3: Backend Deployment (Railway)

### Step 1: Create Railway Project

1. Go to [railway.app](https://railway.app) and sign in
2. Click "New Project"
3. Select "Deploy from GitHub repo"
4. Authorize Railway to access your GitHub
5. Select your `robotic-hackathon` repository
6. Railway will detect the `railway.json` and configure automatically

### Step 2: Configure Environment Variables

In Railway dashboard, go to "Variables" tab and add:

```bash
# Database (from Neon)
DATABASE_URL=postgresql+asyncpg://user:pass@host/db?sslmode=require

# OpenAI (Required)
OPENAI_API_KEY=sk-your-openai-api-key

# Qdrant Vector Database
QDRANT_URL=https://your-cluster.region.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=physical_ai_docs

# JWT Secrets (Generate with: openssl rand -hex 32)
JWT_SECRET_KEY=your-generated-secret-here
REFRESH_SECRET_KEY=your-generated-refresh-secret-here
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=60
REFRESH_TOKEN_EXPIRE_DAYS=7

# CORS Configuration
# Update after frontend is deployed
CORS_ORIGINS=https://your-username.github.io,http://localhost:3000

# Application Config
APP_NAME=Physical AI Interactive Textbook
APP_VERSION=1.0.0
DEBUG=False

# Frontend URL (update after frontend deployment)
FRONTEND_URL=https://your-username.github.io/physical-ai-textbook
```

### Step 3: Generate Secure Secrets

On your local machine, run:

```bash
# Generate JWT secret
openssl rand -hex 32

# Generate refresh secret
openssl rand -hex 32
```

Copy these values to Railway environment variables.

### Step 4: Deploy

1. Click "Deploy" in Railway
2. Wait for build to complete (~3-5 minutes)
3. Once deployed, Railway provides a URL: `https://your-app.up.railway.app`
4. Copy this URL for frontend configuration

### Step 5: Verify Deployment

Test your backend:

```bash
# Check health endpoint
curl https://your-app.up.railway.app/

# Should return:
# {
#   "name": "Physical AI Interactive Textbook",
#   "version": "1.0.0",
#   "status": "running",
#   "docs": "/docs"
# }
```

Visit `https://your-app.up.railway.app/docs` to see API documentation.

### Step 6: Run Database Migrations

Railway automatically runs `alembic upgrade head` on deployment (configured in `railway.json`).

To manually trigger migration:

1. In Railway dashboard, go to "Deployments"
2. Click on latest deployment
3. Click "View Logs"
4. Verify you see: `Running migrations: alembic upgrade head`

---

## Part 4: Ingest Documents into Qdrant

Before deploying frontend, populate the vector database with chapters:

### Option A: Use RAG Setup Script (Recommended)

```bash
# From project root
cd backend

# Activate virtual environment
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Set environment variables
export OPENAI_API_KEY=your-key
export QDRANT_URL=your-url
export QDRANT_API_KEY=your-key
export QDRANT_COLLECTION_NAME=physical_ai_docs

# Run ingestion script
python scripts/setup_rag.py --docs-dir ../frontend/docs --chunk-size 800
```

### Option B: Use RAG Skill (Claude Code)

If using Claude Code:

```bash
# Use the RAG Setup skill
python scripts/setup_rag.py --docs-dir frontend/docs --reset
```

**Expected Output**:
```
🚀 Starting RAG Setup...
📁 Phase 1: Document Discovery
✓ Found 24 markdown files

✂️  Phase 2: Intelligent Chunking
✓ Created 180 chunks

🔢 Phase 3: Embedding Generation
✓ Generated embeddings for batch 1
✓ Generated embeddings for batch 2

💾 Phase 4: Vector Storage
✓ Created collection: physical_ai_docs
✓ Uploaded 180 points to Qdrant

✅ Phase 5: Validation
Query: What is ROS 2?
✓ Found 3 results

🎉 RAG Setup Complete!
```

---

## Part 5: Frontend Deployment (GitHub Pages)

### Step 1: Update Frontend Configuration

1. Open `frontend/docusaurus.config.ts`
2. Update the `url` and `baseUrl`:

```typescript
const config: Config = {
  // ...
  url: 'https://your-username.github.io',
  baseUrl: '/physical-ai-textbook/', // Your repo name
  // ...
```

3. Update API base URL in frontend code:
   - Option A: Environment variable (create `frontend/.env.production`):

```bash
REACT_APP_API_URL=https://your-app.up.railway.app
REACT_APP_API_VERSION=v1
REACT_APP_ENVIRONMENT=production
```

   - Option B: Update `frontend/src/utils/apiClient.ts` directly:

```typescript
const API_BASE_URL = process.env.REACT_APP_API_URL || 'https://your-app.up.railway.app';
```

### Step 2: Enable GitHub Pages

1. Go to your GitHub repository
2. Click "Settings"
3. Scroll to "Pages" section
4. Source: "GitHub Actions"
5. Save

### Step 3: Deploy

Deployment happens automatically on push to `main` branch:

```bash
# Commit your changes
git add .
git commit -m "Configure production deployment"
git push origin main
```

GitHub Actions will:
1. Build the Docusaurus site
2. Deploy to GitHub Pages
3. Complete in ~3-5 minutes

### Step 4: Verify Deployment

1. Check Actions tab in GitHub for workflow status
2. Once complete, visit: `https://your-username.github.io/physical-ai-textbook`
3. Test all features:
   - Browse chapters
   - Test chatbot (should connect to Railway backend)
   - Sign up / Log in
   - Try personalization

### Step 5: Update CORS in Railway

Now that frontend is deployed, update Railway CORS settings:

1. Go to Railway dashboard
2. Navigate to "Variables"
3. Update `CORS_ORIGINS`:

```bash
CORS_ORIGINS=https://your-username.github.io,http://localhost:3000
```

4. Update `FRONTEND_URL`:

```bash
FRONTEND_URL=https://your-username.github.io/physical-ai-textbook
```

5. Click "Save" - Railway will auto-redeploy

---

## Part 6: Testing Checklist

### ✅ Backend Health Check

- [ ] `GET /` returns status JSON
- [ ] `GET /docs` shows API documentation
- [ ] `GET /api/v1/health` returns healthy status

### ✅ Database Connection

- [ ] Backend logs show successful database connection
- [ ] No migration errors in Railway logs

### ✅ RAG Chatbot

- [ ] Chat widget appears in frontend
- [ ] Test query: "What is ROS 2?" returns relevant answer
- [ ] Response time < 3 seconds
- [ ] Sources are displayed with chapter links

### ✅ Authentication

- [ ] Sign up form accepts new users
- [ ] Login works with created credentials
- [ ] JWT tokens are set in localStorage
- [ ] Protected routes redirect to login when not authenticated
- [ ] Logout clears tokens and redirects

### ✅ Content Personalization

- [ ] Personalize button appears on chapter pages
- [ ] Selecting "Beginner" changes content
- [ ] Selecting "Advanced" changes content
- [ ] Reset button restores original content
- [ ] Preference persists across page reloads

### ✅ Performance

- [ ] Lighthouse score > 90
- [ ] Mobile responsive on iPhone/Android
- [ ] Dark mode toggle works
- [ ] Search results appear < 500ms

---

## Part 7: Post-Deployment Tasks

### Monitor Application

**Railway**:
- Check "Metrics" tab for CPU/memory usage
- Review "Logs" for errors
- Set up alerts for downtime

**Qdrant**:
- Monitor storage usage (free tier: 1GB)
- Check query performance in dashboard

**OpenAI**:
- Monitor API usage at [platform.openai.com/usage](https://platform.openai.com/usage)
- Set up usage limits to avoid unexpected charges

### Set Up Custom Domain (Optional)

**For Frontend (GitHub Pages)**:
1. Purchase domain (e.g., `physical-ai-textbook.com`)
2. In GitHub repo settings → Pages → Custom domain
3. Add CNAME record in DNS:
   - Type: CNAME
   - Name: www
   - Value: your-username.github.io
4. Update `docusaurus.config.ts` with new domain

**For Backend (Railway)**:
1. In Railway project settings → Domains
2. Click "Add Custom Domain"
3. Follow Railway's DNS instructions
4. Update `CORS_ORIGINS` to include custom domain

### Enable Analytics (Optional)

**Google Analytics**:
1. Create GA4 property at [analytics.google.com](https://analytics.google.com)
2. Get tracking ID (G-XXXXXXXXXX)
3. Add to `frontend/.env.production`:
   ```bash
   REACT_APP_GA_TRACKING_ID=G-XXXXXXXXXX
   ```
4. Redeploy frontend

---

## Troubleshooting

### Issue: Backend Returns 500 Error

**Solution**:
- Check Railway logs for Python errors
- Verify all environment variables are set
- Ensure DATABASE_URL includes `+asyncpg`
- Check database connection string

### Issue: CORS Error in Frontend

**Solution**:
- Verify `CORS_ORIGINS` in Railway includes frontend URL
- Ensure frontend URL matches exactly (no trailing slash)
- Check browser console for exact CORS error
- Redeploy backend after CORS changes

### Issue: Chatbot Returns "No results found"

**Solution**:
- Verify Qdrant collection has documents:
  ```bash
  curl -X GET "https://your-cluster.qdrant.io/collections/physical_ai_docs" \
    -H "api-key: your-key"
  ```
- Re-run RAG ingestion script
- Check QDRANT_COLLECTION_NAME matches

### Issue: Authentication Not Working

**Solution**:
- Check browser localStorage for tokens
- Verify JWT_SECRET_KEY is set in Railway
- Check backend logs for JWT errors
- Ensure passwords meet strength requirements (8+ chars, 1 uppercase, 1 number)

### Issue: GitHub Pages 404 Error

**Solution**:
- Verify `baseUrl` in `docusaurus.config.ts` matches repo name
- Check GitHub Actions workflow completed successfully
- Wait 5-10 minutes for DNS propagation
- Try hard refresh (Ctrl+Shift+R)

---

## Cost Estimate

**Free Tier Services**:
- GitHub Pages: Free for public repos
- Railway: $5/month credit (enough for small projects)
- Neon PostgreSQL: Free tier (3GB storage)
- Qdrant: Free tier (1GB storage)
- OpenAI: Pay-as-you-go (~$1-5/month for light usage)

**Total Monthly Cost**: ~$5-10 for moderate usage

---

## Rollback Strategy

### Roll Back Backend

1. Go to Railway → Deployments
2. Click on previous deployment
3. Click "Redeploy"

### Roll Back Frontend

1. Go to GitHub repository
2. Find last working commit: `git log`
3. Revert: `git revert <commit-hash>`
4. Push: `git push origin main`

---

## Security Checklist

- [ ] All API keys stored in environment variables (not in code)
- [ ] `.env` file added to `.gitignore`
- [ ] `.env.example` has placeholder values (no real credentials)
- [ ] CORS configured to allow only frontend domain
- [ ] HTTPS enabled (Railway provides by default)
- [ ] JWT secrets generated with strong random values
- [ ] Database uses SSL connection (`sslmode=require`)
- [ ] Rate limiting enabled on backend endpoints
- [ ] Input validation on all API endpoints

---

## Maintenance

### Weekly Tasks
- [ ] Check Railway logs for errors
- [ ] Monitor OpenAI API usage
- [ ] Review Qdrant storage usage

### Monthly Tasks
- [ ] Update dependencies (`npm audit fix`, `pip-audit`)
- [ ] Review and rotate JWT secrets
- [ ] Backup database (Neon provides automatic backups)
- [ ] Test all features end-to-end

### As Needed
- [ ] Re-ingest documents when chapters are updated
- [ ] Scale Railway plan if usage increases
- [ ] Optimize Qdrant queries for performance

---

## Support

- **Project Issues**: [GitHub Issues](https://github.com/yourusername/robotic-hackathon/issues)
- **Railway Docs**: [docs.railway.app](https://docs.railway.app)
- **Neon Docs**: [neon.tech/docs](https://neon.tech/docs)
- **Qdrant Docs**: [qdrant.tech/documentation](https://qdrant.tech/documentation)
- **Docusaurus Docs**: [docusaurus.io](https://docusaurus.io)

---

**Deployed Successfully?** 🎉

Your Physical AI Interactive Textbook is now live! Share the URL and gather feedback from users.
