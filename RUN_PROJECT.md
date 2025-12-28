# Complete Project Setup and Run Guide

This guide provides all commands needed to run the entire project: Frontend, Backend, and Chatbot.

---

## 🚀 Quick Start (All Services)

### Terminal 1: Backend Server
```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend
source venv/bin/activate
export PYTHONPATH=/media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend:$PYTHONPATH
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

### Terminal 2: Frontend/Docusaurus
```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/docs
npm install
npm start
```

---

## 📋 Detailed Setup Instructions

## 1️⃣ Backend Setup

### First-Time Setup

```bash
# Navigate to backend directory
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend

# Create virtual environment (if not exists)
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Configure environment variables
cp .env.example .env
# Edit .env with your actual API keys
nano .env
```

### Required Environment Variables in `.env`

```env
# AI Provider Configuration
AI_PROVIDER=gemini
GEMINI_API_KEY=your_actual_gemini_api_key
GEMINI_MODEL=gemini-2.5-flash

# Qdrant Configuration
QDRANT_API_KEY=your_actual_qdrant_api_key
QDRANT_HOST=your_qdrant_cluster_url
QDRANT_PORT=6333
QDRANT_HTTPS=true
QDRANT_COLLECTION_NAME=rag_embeddings

# Database Configuration
DATABASE_URL=postgresql+asyncpg://user:password@host:5432/dbname

# Server Settings
SERVER_HOST=0.0.0.0
SERVER_PORT=8000

# Frontend URL (for CORS)
FRONTEND_URL=http://localhost:3000

# Cohere API
COHERE_API_KEY=your_cohere_api_key
```

### Run Backend Server

**Method 1: Using uvicorn with PYTHONPATH (RECOMMENDED)**
```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend
source venv/bin/activate
export PYTHONPATH=/media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend:$PYTHONPATH
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

**Method 2: Using Python directly**
```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend
source venv/bin/activate
export PYTHONPATH=/media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend:$PYTHONPATH
python src/main.py
```

**Method 3: From parent directory**
```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing
source backend/venv/bin/activate
uvicorn backend.src.main:app --reload --host 0.0.0.0 --port 8000
```

### Verify Backend is Running
```bash
# Test health endpoint
curl http://localhost:8000/health

# Open API documentation in browser
# http://localhost:8000/docs
```

---

## 2️⃣ Frontend/Docusaurus Setup

### First-Time Setup

```bash
# Navigate to docs directory
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/docs

# Install dependencies
npm install

# If you get errors, try:
npm install --legacy-peer-deps
```

### Run Frontend Development Server

```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/docs
npm start
```

The frontend will automatically open in your browser at: **http://localhost:3000**

### Build for Production

```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/docs
npm run build
npm run serve
```

---

## 3️⃣ Chatbot Integration

The chatbot is integrated into the frontend and runs automatically when you access:

**http://localhost:3000/chatbot**

### Prerequisites for Chatbot:
1. ✅ Backend must be running (port 8000)
2. ✅ Qdrant vector database configured with embeddings
3. ✅ AI provider (Gemini/OpenAI) API key configured

### Test Chatbot Separately

If you want to test the chatbot service directly:

```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/docs/src/components/Chatbot
# Open ChatbotService.ts to see the API endpoints
# The chatbot communicates with: http://localhost:8000/api/v1/query
```

---

## 🎯 Running Everything Together

### Open 2 Terminal Windows:

**Terminal 1 - Backend:**
```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend
source venv/bin/activate
export PYTHONPATH=$PWD:$PYTHONPATH
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

**Terminal 2 - Frontend:**
```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/docs
npm start
```

### Access Your Application:

- **Frontend**: http://localhost:3000
- **Backend API**: http://localhost:8000
- **API Docs**: http://localhost:8000/docs
- **Chatbot**: http://localhost:3000/chatbot
- **Health Check**: http://localhost:8000/health

---

## 🔧 Troubleshooting

### Backend Issues

**Issue: "No module named src.main"**
```bash
# Solution: Set PYTHONPATH
export PYTHONPATH=/media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend:$PYTHONPATH

# Or run from the correct directory
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend
uvicorn src.main:app --reload
```

**Issue: "ModuleNotFoundError" for dependencies**
```bash
# Make sure virtual environment is activated
source venv/bin/activate

# Reinstall dependencies
pip install -r requirements.txt
```

**Issue: Database connection errors**
```bash
# Check your .env file has correct DATABASE_URL
cat .env | grep DATABASE_URL

# Test database connection
python -c "from src.config.database import async_engine; print('Database configured')"
```

**Issue: Port 8000 already in use**
```bash
# Find and kill the process
lsof -i :8000
kill -9 <PID>

# Or use a different port
uvicorn src.main:app --reload --port 8001
```

### Frontend Issues

**Issue: "Module not found" errors**
```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/docs
rm -rf node_modules package-lock.json
npm install
```

**Issue: Port 3000 already in use**
```bash
# Kill process on port 3000
lsof -i :3000
kill -9 <PID>

# Or Docusaurus will ask to use port 3001
```

**Issue: Build fails**
```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/docs
npm run clear
npm run build
```

### Chatbot Issues

**Issue: Chatbot not responding**
1. Check backend is running: `curl http://localhost:8000/health`
2. Check Qdrant connection in backend logs
3. Verify API keys in `.env`
4. Check browser console for errors (F12)

**Issue: CORS errors**
```bash
# Make sure backend CORS is configured for frontend URL
# Check src/main.py allows origin: http://localhost:3000
```

---

## 📊 System Requirements

- **Python**: 3.11 or higher
- **Node.js**: 20.0 or higher
- **RAM**: Minimum 4GB (8GB recommended)
- **Disk Space**: ~2GB for all dependencies

---

## 🌐 Production Deployment

### Backend Production
```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend
source venv/bin/activate
gunicorn src.main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:8000
```

### Frontend Production
```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/docs
npm run build
# Deploy the 'build' folder to GitHub Pages or hosting service
```

---

## 📝 Useful Commands

### Backend Commands
```bash
# View logs
tail -f backend/logs/app.log

# Run tests
cd backend && pytest tests/

# Generate embeddings
cd backend && python generate_embeddings.py

# Check Python version
python --version

# List installed packages
pip list
```

### Frontend Commands
```bash
# Clear cache
npm run clear

# Type check
npm run typecheck

# Build for production
npm run build

# Serve production build
npm run serve
```

### Database Commands
```bash
# Run migrations
cd backend
python -c "from src.database.migrations import create_tables; create_tables()"

# Check database connection
psql $DATABASE_URL
```

---

## 🎓 Features Available

Once everything is running, you have access to:

✅ **User Authentication** - Login/Signup pages
✅ **Urdu Translation** - Translate content with RTL support
✅ **AI Chatbot** - RAG-powered Q&A system
✅ **Chapter Personalization** - Personalized content based on user profile
✅ **Physical AI Documentation** - Complete robotics guide
✅ **MCP Integration** - Model Context Protocol examples

---

## 📞 Support

- **GitHub**: https://github.com/asma-aslam30/hackathon
- **Issues**: https://github.com/asma-aslam30/hackathon/issues
- **API Docs**: http://localhost:8000/docs (when running)

---

## ✅ Quick Checklist

Before running, make sure you have:

- [ ] Python 3.11+ installed
- [ ] Node.js 20+ installed
- [ ] Virtual environment created for backend
- [ ] Dependencies installed (pip & npm)
- [ ] `.env` file configured with all API keys
- [ ] PostgreSQL/Neon database accessible
- [ ] Qdrant vector database configured
- [ ] Embeddings generated in Qdrant

---

**Happy Coding! 🚀**
