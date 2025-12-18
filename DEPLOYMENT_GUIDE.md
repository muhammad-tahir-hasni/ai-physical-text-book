# Complete Deployment Guide
## Physical AI & Humanoid Robotics Interactive Textbook

---

## 📋 Table of Contents
1. [GitHub Setup](#part-1-github-setup)
2. [Backend Deployment (Railway)](#part-2-backend-deployment-railway)
3. [Frontend Deployment (Vercel)](#part-3-frontend-deployment-vercel)
4. [Environment Variables Setup](#part-4-environment-variables)
5. [Testing & Verification](#part-5-testing)

---

## Part 1: GitHub Setup

### Step 1: Create GitHub Repository

1. Go to https://github.com and login
2. Click **"New"** button (top right, next to your profile)
3. Fill in details:
   - **Repository name**: `robotic-hackathon`
   - **Description**: "Physical AI & Humanoid Robotics Interactive Textbook"
   - **Visibility**: Public or Private (your choice)
   - ✅ **Do NOT** check "Initialize with README"
4. Click **"Create Repository"**

### Step 2: Push Your Code to GitHub

Open terminal and run these commands:

```bash
# Navigate to your project directory
cd /Users/apple/Desktop/robotic-hackathon

# Initialize git (if not already initialized)
git init

# Add all files
git add .

# Create first commit
git commit -m "Initial commit: Physical AI Interactive Textbook"

# Add your GitHub repository as remote
# Replace YOUR_USERNAME and YOUR_REPO_NAME with your actual GitHub username and repo name
git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git

# Push to GitHub
git branch -M main
git push -u origin main
```

**Important**: When prompted, enter your GitHub username and Personal Access Token (not password).

#### How to Create GitHub Personal Access Token:
1. Go to GitHub Settings → Developer Settings → Personal Access Tokens → Tokens (classic)
2. Click "Generate new token (classic)"
3. Give it a name: "Deployment Token"
4. Select scopes: `repo` (all checkboxes under it)
5. Click "Generate token"
6. **Copy the token** (you won't see it again!)
7. Use this token as password when pushing to GitHub

---

## Part 2: Backend Deployment (Railway)

**Why Railway?**
- Free $5 credit per month (enough for small projects)
- Easy PostgreSQL setup
- Environment variables management
- Auto-deploy from GitHub

### Step 1: Create Railway Account

1. Go to https://railway.app
2. Sign up with GitHub account
3. Verify your email

### Step 2: Create New Project

1. Click **"New Project"**
2. Select **"Deploy from GitHub repo"**
3. Select your `robotic-hackathon` repository
4. Railway will detect it's a Python project

### Step 3: Configure Backend Service

1. **Select Root Directory**:
   - Click on the deployed service
   - Go to **Settings** tab
   - Under **Build**, set **Root Directory** to `backend`
   - Under **Build**, set **Build Command** to:
     ```
     pip install -r requirements.txt
     ```
   - Under **Deploy**, set **Start Command** to:
     ```
     uvicorn app.main:app --host 0.0.0.0 --port $PORT
     ```

2. **Add PostgreSQL Database**:
   - Click **"+ New"** → **Database** → **PostgreSQL**
   - Railway will automatically create a database
   - Go to the PostgreSQL service → **Variables** tab
   - Copy the `DATABASE_URL` value

3. **Add Qdrant Vector Database**:
   - You need to use Qdrant Cloud (separate service)
   - Go to https://cloud.qdrant.io
   - Sign up for free account
   - Create a new cluster (free tier)
   - Copy the **API URL** and **API Key**

### Step 4: Set Environment Variables

In your Railway backend service, go to **Variables** tab and add:

```bash
# Database
DATABASE_URL=<automatically provided by Railway PostgreSQL>

# Security
SECRET_KEY=<generate a random 32-character string>
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=60
REFRESH_TOKEN_EXPIRE_DAYS=7

# OpenAI (for RAG)
OPENAI_API_KEY=<your OpenAI API key>

# Qdrant Vector Database
QDRANT_URL=<your Qdrant cloud URL>
QDRANT_API_KEY=<your Qdrant API key>
QDRANT_COLLECTION_NAME=Humanoid

# CORS (your frontend URL - will add after frontend deployment)
ALLOWED_ORIGINS=https://your-frontend-url.vercel.app,http://localhost:3000

# Environment
ENVIRONMENT=production
```

### Step 5: Deploy Backend

1. Click **"Deploy"** button
2. Wait for build to complete (5-10 minutes first time)
3. Once deployed, you'll get a URL like: `https://robotic-hackathon-production.up.railway.app`
4. **Copy this URL** - you'll need it for frontend

### Step 6: Initialize Database

After deployment, you need to create database tables:

1. Go to your Railway backend service
2. Click on **"Deployments"** tab
3. Find the latest deployment, click **"View Logs"**
4. In a separate terminal, connect to your Railway PostgreSQL:

```bash
# Install Railway CLI
npm install -g @railway/cli

# Login to Railway
railway login

# Link to your project
railway link

# Run migrations (in backend directory)
cd backend
railway run alembic upgrade head

# Run RAG setup
railway run python setup_rag.py
```

---

## Part 3: Frontend Deployment (Vercel)

**Why Vercel?**
- Built specifically for React/Next.js/Docusaurus
- Free tier with unlimited bandwidth
- Auto-deploy on git push
- Great CDN performance

### Step 1: Create Vercel Account

1. Go to https://vercel.com
2. Sign up with GitHub account
3. Authorize Vercel to access your repositories

### Step 2: Import Project

1. Click **"Add New..."** → **Project**
2. Select your `robotic-hackathon` repository
3. Click **"Import"**

### Step 3: Configure Build Settings

Vercel will auto-detect Docusaurus, but verify:

1. **Framework Preset**: Docusaurus 2
2. **Root Directory**: `frontend` (click "Edit" and select frontend folder)
3. **Build Command**: `npm run build` (should be auto-filled)
4. **Output Directory**: `build` (should be auto-filled)
5. **Install Command**: `npm install` (should be auto-filled)

### Step 4: Set Environment Variables

In Vercel project settings → **Environment Variables**, add:

```bash
REACT_APP_API_URL=https://your-backend-url.railway.app
```

Replace `your-backend-url.railway.app` with your Railway backend URL from Part 2.

### Step 5: Update Base URL in Code

Before deploying, update `frontend/docusaurus.config.ts`:

```typescript
// Change this:
url: 'https://yourusername.github.io',
baseUrl: '/robotic-hackathon/',

// To this (for Vercel):
url: 'https://your-vercel-project.vercel.app',
baseUrl: '/',
```

Commit and push this change:

```bash
cd /Users/apple/Desktop/robotic-hackathon
git add frontend/docusaurus.config.ts
git commit -m "Update config for Vercel deployment"
git push
```

### Step 6: Deploy Frontend

1. Click **"Deploy"** button
2. Wait for build to complete (3-5 minutes)
3. Once deployed, you'll get a URL like: `https://robotic-hackathon-xyz.vercel.app`

---

## Part 4: Update CORS Settings

After both deployments, you need to update backend CORS settings:

### On Railway:

1. Go to your backend service on Railway
2. Go to **Variables** tab
3. Update `ALLOWED_ORIGINS`:
   ```
   https://your-frontend.vercel.app,http://localhost:3000
   ```
4. Click **"Deploy"** to restart with new settings

---

## Part 5: Testing & Verification

### Test Backend:

Visit: `https://your-backend.railway.app/docs`
- You should see FastAPI Swagger documentation
- Try the `/api/v1/health` endpoint - should return `{"status": "healthy"}`

### Test Frontend:

Visit: `https://your-frontend.vercel.app`
- Homepage should load
- Try signing up with a real email
- Login and verify redirection to `/docs/intro`
- Test RAG chatbot
- Logout and verify redirect to homepage

---

## 🎯 Quick Reference: What Goes Where?

| Component | Platform | Why |
|-----------|----------|-----|
| Backend (FastAPI) | Railway | Needs database, easy setup |
| PostgreSQL | Railway | Built-in with backend |
| Qdrant | Qdrant Cloud | Specialized vector DB |
| Frontend (Docusaurus) | Vercel | Best for static sites |

---

## 💰 Cost Breakdown

### Free Tier Limits:

**Railway** ($5/month credit):
- Backend API: ~$3-4/month
- PostgreSQL: ~$1-2/month
- **Total**: Within free credit

**Vercel** (Free tier):
- Frontend: Unlimited bandwidth
- 100GB bandwidth per month
- **Cost**: $0

**Qdrant Cloud** (Free tier):
- 1GB storage
- 100k vectors
- **Cost**: $0

**Total Monthly Cost**: $0 (if within free tiers)

---

## 🔧 Troubleshooting

### Issue: Frontend can't connect to backend
**Solution**: Check CORS settings in backend environment variables

### Issue: Database connection failed
**Solution**: Verify DATABASE_URL is set correctly in Railway

### Issue: RAG chatbot not working
**Solution**:
1. Check OPENAI_API_KEY is valid
2. Verify Qdrant connection
3. Run `railway run python setup_rag.py` to reinitialize

### Issue: Build fails on Vercel
**Solution**: Check that `frontend/package.json` has all dependencies listed

---

## 📝 Post-Deployment Checklist

- [ ] Backend deployed on Railway
- [ ] PostgreSQL database created and tables initialized
- [ ] Frontend deployed on Vercel
- [ ] CORS configured correctly
- [ ] All environment variables set
- [ ] RAG system initialized with embeddings
- [ ] Test signup/login flow
- [ ] Test chatbot functionality
- [ ] Custom domain configured (optional)

---

## 🚀 Alternative Deployment Options

### Option 2: Both on Render.com
- Free tier available
- Both frontend and backend on one platform
- Easier management but slower cold starts

### Option 3: AWS (More Complex)
- EC2 for backend
- S3 + CloudFront for frontend
- RDS for PostgreSQL
- More control but higher cost and complexity

---

## 📞 Need Help?

Common issues and solutions:
1. **"Module not found" errors**: Run `pip install -r requirements.txt` (backend) or `npm install` (frontend)
2. **CORS errors**: Double-check ALLOWED_ORIGINS includes your frontend URL
3. **Database errors**: Ensure migrations ran successfully with `alembic upgrade head`
4. **API key errors**: Verify all API keys are valid and have necessary permissions

---

## 🎓 Best Practices

1. **Never commit .env files** (already in .gitignore)
2. **Use different API keys** for production vs development
3. **Set up monitoring** (Railway and Vercel provide built-in logs)
4. **Enable HTTPS only** (automatic on Railway and Vercel)
5. **Regular backups** of PostgreSQL database (Railway automatic backups)

---

Good luck with your deployment! 🚀
