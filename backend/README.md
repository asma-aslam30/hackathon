# Backend Setup and Run Guide

This guide provides step-by-step instructions to set up and run the FastAPI backend server.

## Prerequisites

- Python 3.11 or higher
- pip (Python package manager)
- PostgreSQL database (Neon or local)
- Qdrant vector database (cloud or local)

## Quick Start

### 1. Navigate to Backend Directory

```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend
```

### 2. Create Virtual Environment (Recommended)

```bash
# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate  # On Linux/Mac
# OR
venv\Scripts\activate  # On Windows
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Configure Environment Variables

Copy the example environment file and update with your credentials:

```bash
cp .env.example .env
```

Edit the `.env` file with your actual values:

```env
# AI Provider Configuration
AI_PROVIDER=gemini  # or 'openai'
GEMINI_API_KEY=your_actual_gemini_api_key
GEMINI_MODEL=gemini-2.5-flash
OPENAI_API_KEY=your_actual_openai_api_key
OPENAI_MODEL=gpt-4-turbo-preview

# Qdrant Configuration
QDRANT_API_KEY=your_actual_qdrant_api_key
QDRANT_HOST=your_qdrant_cluster_url
QDRANT_PORT=6333
QDRANT_HTTPS=true
QDRANT_COLLECTION_NAME=rag_embeddings

# Database Configuration (for PostgreSQL/Neon)
DATABASE_URL=postgresql://user:password@host:5432/dbname
# OR for async
DATABASE_URL=postgresql+asyncpg://user:password@host:5432/dbname

# Application Settings
DEBUG_MODE=true
LOG_LEVEL=INFO
MAX_QUERY_LENGTH=1000
RESPONSE_TIMEOUT=30

# Server Settings
SERVER_HOST=0.0.0.0
SERVER_PORT=8000

# Frontend URL (for CORS)
FRONTEND_URL=http://localhost:3000

# Additional Settings
COHERE_API_KEY=your_cohere_api_key_here
```

### 5. Run Database Migrations (if needed)

```bash
# The migrations will run automatically on startup
# But you can run them manually if needed:
python -c "from src.database.migrations import create_tables; create_tables()"
```

### 6. Start the Backend Server

**Option A: Using uvicorn directly (Recommended for Development)**

```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

**Option B: Using Python module**

```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend
python -m src.main
```

**Option C: Using the alternative main.py**

```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend
python -m uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

### 7. Verify Backend is Running

Open your browser or use curl to check:

```bash
# Health check
curl http://localhost:8000/health

# Root endpoint
curl http://localhost:8000/

# API documentation (Swagger UI)
# Open in browser: http://localhost:8000/docs
```

## API Endpoints

Once running, the backend provides the following key endpoints:

### General Endpoints
- `GET /` - Root endpoint with API info
- `GET /health` - Health check endpoint
- `GET /docs` - Interactive API documentation (Swagger UI)
- `GET /redoc` - Alternative API documentation (ReDoc)

### Authentication Endpoints
- `POST /api/v1/auth/register` - User registration
- `POST /api/v1/auth/login` - User login
- `POST /api/v1/auth/logout` - User logout
- `GET /api/v1/auth/me` - Get current user

### Translation Endpoints
- `POST /api/v1/translation/translate` - Translate content to Urdu
- `GET /api/v1/translation/cache/{content_id}` - Get cached translation

### Personalization Endpoints
- `GET /api/v1/personalization/content/{chapter_id}` - Get personalized chapter content
- `POST /api/v1/personalization/preferences` - Update personalization preferences

### Query/RAG Endpoints
- `POST /api/v1/query` - Query the RAG system with a question

### User Profile Endpoints
- `GET /api/v1/profile` - Get user profile
- `PUT /api/v1/profile` - Update user profile
- `POST /api/v1/profile/background` - Update user background

## Troubleshooting

### Issue: Module not found errors

**Solution:**
```bash
# Make sure you're in the backend directory
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend

# Install dependencies again
pip install -r requirements.txt

# Run with Python module syntax
python -m uvicorn src.main:app --reload
```

### Issue: Database connection errors

**Solution:**
- Verify your `DATABASE_URL` in `.env` is correct
- Ensure PostgreSQL/Neon database is running and accessible
- Check network connectivity to database

### Issue: Qdrant connection errors

**Solution:**
- Verify `QDRANT_API_KEY` and `QDRANT_HOST` in `.env`
- Ensure Qdrant collection exists
- Test Qdrant connection separately

### Issue: Port already in use

**Solution:**
```bash
# Find process using port 8000
lsof -i :8000

# Kill the process
kill -9 <PID>

# Or use a different port
uvicorn src.main:app --reload --port 8001
```

## Development Tips

### Hot Reload
The `--reload` flag enables hot reload during development. Changes to your code will automatically restart the server.

### Debug Mode
Set `DEBUG_MODE=true` in `.env` for detailed error messages.

### View Logs
Logs are printed to console by default. Configure `LOG_LEVEL` in `.env` to control verbosity.

### Testing Endpoints
Use the interactive API docs at `http://localhost:8000/docs` to test endpoints without writing code.

## Production Deployment

For production deployment:

1. Set `DEBUG_MODE=false` in `.env`
2. Remove `--reload` flag
3. Use a production WSGI server like Gunicorn:

```bash
gunicorn src.main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:8000
```

4. Set up proper CORS origins in `src/main.py`
5. Use environment-specific `.env` files
6. Enable HTTPS/SSL certificates

## Additional Commands

### Generate Embeddings
```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend
python generate_embeddings.py
```

### Run Tests
```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend
pytest tests/
```

### Run Specific Test File
```bash
cd /media/xolva/1610d648-c91c-4442-9109-d3d99767152b/Speckitplus/testing/backend
python test_retrieval_pipeline.py
```

## Architecture

- **src/main.py** - Main application entry point (User Auth API)
- **src/api/main.py** - Alternative entry point (Agent Retrieval API)
- **src/api/** - API route handlers
- **src/services/** - Business logic services
- **src/models/** - Data models and schemas
- **src/config/** - Configuration and settings
- **src/utils/** - Utility functions
- **src/database/** - Database setup and migrations

## Support

For issues or questions, please check:
- API Documentation: http://localhost:8000/docs
- Project Repository: https://github.com/asma-aslam30/hackathon
- Logs: Check console output for detailed error messages
