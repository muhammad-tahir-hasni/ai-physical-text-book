# 🚀 SETUP COMMANDS - Ye Commands Run Karein

**Date**: 2025-12-16
**Status**: ✅ Project Ready for Development

---

## ✅ Jo Kuch Ho Chuka Hai

1. ✅ **robotic/** folder remove kar diya - ab sirf **frontend/** hai
2. ✅ **robotic/** ka content **frontend/** mein merge ho gaya
3. ✅ API keys sanitize ho gayi hain
4. ✅ `.gitignore` updated
5. ✅ OpenAI dependency add ho gayi
6. ✅ Free deployment configs ready (Render.com + Vercel)

---

## 📋 STEP 1: Backend Setup (5 minutes)

```bash
# Terminal mein ye commands run karein:

# 1. Backend folder mein jao
cd backend

# 2. Virtual environment banao
python3 -m venv venv

# 3. Virtual environment activate karo
source venv/bin/activate

# 4. Dependencies install karo (2-3 minutes lagenge)
pip install -r requirements.txt

# 5. Database migrations run karo
alembic upgrade head

# 6. Backend start karo
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

**Backend ab chalega**: http://localhost:8000
**API Docs**: http://localhost:8000/docs

Backend ko running rehne do, **naya terminal** kholo next step ke liye.

---

## 📋 STEP 2: Frontend Setup (5 minutes)

```bash
# Naye terminal mein ye commands run karein:

# 1. Frontend folder mein jao
cd frontend

# 2. Dependencies install karo (2-3 minutes lagenge)
npm install

# 3. Frontend start karo
npm start
```

**Frontend ab chalega**: http://localhost:3000

---

## 🧪 STEP 3: Test Karo (2 minutes)

Browser mein jao:
- **Frontend**: http://localhost:3000 - Textbook dekho
- **Backend API**: http://localhost:8000/docs - API test karo
- **Health Check**: http://localhost:8000/health - Status check karo

---

## 🌐 STEP 4: GitHub Pe Push Karo

```bash
# 1. Git status dekho
git status

# 2. Sabhi changes add karo
git add .

# 3. Commit karo
git commit -m "Complete project setup: merged robotic into frontend, added deployment configs"

# 4. GitHub pe naya repository banao (github.com pe jao)
# Repository name: robotic-hackathon

# 5. Remote add karo (apna username dalein)
git remote add origin https://github.com/YOUR-USERNAME/robotic-hackathon.git

# 6. Push karo
git push -u origin main

# Note: Agar branch name master hai to:
git branch -M main  # Pehle main mein convert karo
git push -u origin main
```

---

## 🚀 STEP 5: Deployment (FREE Platforms)

### Option 1: Render.com (Backend) + GitHub Pages (Frontend) ✅ RECOMMENDED

#### Backend pe Render.com:

1. **Render.com** pe jao: https://render.com
2. Sign up karo (GitHub se connect karo)
3. "New +" → "Web Service" click karo
4. GitHub repository select karo: `robotic-hackathon`
5. Settings:
   - **Name**: `physical-ai-backend`
   - **Region**: `Oregon (US West)`
   - **Branch**: `main`
   - **Root Directory**: `backend`
   - **Environment**: `Python 3`
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
   - **Plan**: **Free** (select karo)

6. **Environment Variables** add karo (ek ek kar ke):
   ```
   DATABASE_URL=postgresql://...  (Render ka PostgreSQL use karo)
   OPENAI_API_KEY=your-key-here
   QDRANT_URL=your-qdrant-url
   QDRANT_API_KEY=your-qdrant-key
   COHERE_API_KEY=your-cohere-key
   CLAUDE_API_KEY=your-claude-key
   JWT_SECRET_KEY=your-jwt-secret
   REFRESH_SECRET_KEY=your-refresh-secret
   CORS_ORIGINS=https://YOUR-USERNAME.github.io
   FRONTEND_URL=https://YOUR-USERNAME.github.io
   ```

7. "Create Web Service" click karo
8. Wait karo (5-10 minutes)
9. URL milega: `https://physical-ai-backend.onrender.com`

#### Frontend pe GitHub Pages:

1. GitHub repository settings mein jao
2. **Settings** → **Pages**
3. **Source**: `GitHub Actions` select karo
4. GitHub Actions automatically deploy karega (workflow already hai)
5. Frontend URL: `https://YOUR-USERNAME.github.io/robotic-hackathon`

6. **Frontend .env update karo**:
   ```bash
   cd frontend
   nano .env  # ya koi editor use karo

   # Ye line change karo:
   REACT_APP_API_URL=https://physical-ai-backend.onrender.com
   ```

7. Commit aur push karo:
   ```bash
   git add frontend/.env
   git commit -m "Update API URL for production"
   git push
   ```

---

### Option 2: Fly.io (Backend + Frontend dono) - Completely FREE

```bash
# 1. Fly.io CLI install karo
curl -L https://fly.io/install.sh | sh

# 2. Login karo
fly auth login

# 3. Backend deploy karo
cd backend
fly launch --name physical-ai-backend --region sjc
# Postgres chahiye? Yes select karo (free tier)

# 4. Environment variables set karo
fly secrets set OPENAI_API_KEY=your-key
fly secrets set QDRANT_URL=your-url
fly secrets set QDRANT_API_KEY=your-key
# ... baaki sab bhi

# 5. Deploy karo
fly deploy

# 6. Frontend deploy karo
cd ../frontend
fly launch --name physical-ai-frontend --region sjc
fly deploy
```

---

### Option 3: Vercel (Frontend only - FREE)

```bash
# 1. Vercel CLI install karo
npm install -g vercel

# 2. Login karo
vercel login

# 3. Frontend deploy karo
cd frontend
vercel --prod

# Backend ke liye Render use karo (upar dekho)
```

---

## 🔧 Free Services Setup

### 1. PostgreSQL Database (FREE):
- **Neon.tech**: https://neon.tech (FREE tier - best)
- **Render PostgreSQL**: Automatic with Render.com
- **Supabase**: https://supabase.com (FREE tier)

### 2. Redis (FREE):
- **Upstash**: https://upstash.com (FREE tier - best)
- **Render Redis**: Automatic with Render.com

### 3. Vector Database (Already hai):
- ✅ Qdrant Cloud: 1GB free tier

---

## 📊 Deployment Summary

| Service | Platform | Cost | URL After Deploy |
|---------|----------|------|------------------|
| **Frontend** | GitHub Pages | FREE | `https://username.github.io/robotic-hackathon` |
| **Backend** | Render.com | FREE | `https://physical-ai-backend.onrender.com` |
| **Database** | Neon.tech | FREE | Auto (connection string milega) |
| **Redis** | Upstash | FREE | Auto (connection string milega) |
| **Vector DB** | Qdrant Cloud | FREE | Already configured |

**Total Cost**: 💰 **₹0 (Completely FREE)**

---

## 🐛 Agar Problem Aaye

### Backend start nahi ho raha:
```bash
# Virtual environment active hai?
source backend/venv/bin/activate

# Dependencies install hain?
pip install -r backend/requirements.txt

# Database migrate hua?
cd backend
alembic upgrade head

# Port busy hai?
lsof -ti:8000 | xargs kill -9
```

### Frontend start nahi ho raha:
```bash
# Node modules install hain?
cd frontend
rm -rf node_modules package-lock.json
npm install

# Cache clear karo
npm run clear

# Port busy hai?
lsof -ti:3000 | xargs kill -9
```

### Database connection error:
```bash
# .env file check karo
cat backend/.env

# DATABASE_URL correct hai?
# Format: postgresql+asyncpg://user:pass@host:5432/dbname
```

---

## 📚 Documentation

- **Complete Analysis**: `PROJECT_ANALYSIS.md`
- **Setup Guide**: `SETUP.md`
- **Deployment Guide**: `DEPLOYMENT.md`
- **API Documentation**: http://localhost:8000/docs (backend running hone ke baad)

---

## ✅ Final Checklist

Before GitHub push:
- [ ] Backend locally chal raha hai? (http://localhost:8000)
- [ ] Frontend locally chal raha hai? (http://localhost:3000)
- [ ] API keys `.env` files mein hain? (git mein nahi)
- [ ] Tests pass ho rahe hain? (`cd backend && pytest`)
- [ ] `.gitignore` updated hai?
- [ ] `robotic/` folder remove ho gaya?

After GitHub push:
- [ ] Repository public/private set kiya?
- [ ] GitHub Pages enable kiya?
- [ ] Render.com pe backend deploy kiya?
- [ ] Environment variables Render pe set kiye?
- [ ] Frontend URL backend CORS mein add kiya?

---

## 🎯 Quick Start Commands (Copy-Paste)

```bash
# Terminal 1: Backend
cd backend
source venv/bin/activate
uvicorn app.main:app --reload --port 8000

# Terminal 2: Frontend
cd frontend
npm start

# Terminal 3: Tests
cd backend
source venv/bin/activate
pytest tests/ -v
```

---

## 🚀 Deployment Commands (Copy-Paste)

```bash
# GitHub push
git add .
git commit -m "Production ready: Complete setup"
git push -u origin main

# Render.com setup (already configured in render.yaml)
# Just connect GitHub repository

# Fly.io deployment
fly launch --name physical-ai-backend
fly deploy
```

---

**Sab kuch ready hai! Ab bas ye commands run karein aur apka project live ho jayega! 🎉**

Koi problem ho to mujhe batao, main help karunga.
