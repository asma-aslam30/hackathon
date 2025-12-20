# Deployment Guide: Physical AI & Humanoid Robotics Platform

This guide provides instructions for deploying the complete system with both frontend (Docusaurus) and backend (FastAPI) services, including the integrated chatbot functionality.

## Architecture Overview

The system consists of:
- **Frontend**: Docusaurus-based documentation site with integrated chatbot
- **Backend**: FastAPI service with AI-powered query capabilities
- **Vector Database**: Qdrant for storing and retrieving embeddings
- **AI Provider**: OpenAI or Google Gemini for natural language processing

## Prerequisites

- Docker and Docker Compose
- Node.js and npm for frontend
- Valid API keys for your chosen AI provider (OpenAI or Google Gemini)
- Qdrant Cloud account or local Qdrant instance

## Backend Deployment

### 1. Configure Environment Variables

Copy the example environment file and update with your actual API keys:

```bash
cd backend
cp .env.example .env
# Edit .env with your actual API keys
```

Required environment variables:
- `GEMINI_API_KEY` or `OPENAI_API_KEY`
- `QDRANT_API_KEY` and `QDRANT_HOST`
- `COHERE_API_KEY` (for embedding generation)

### 2. Deploy Backend Services

```bash
# Build and deploy using Docker Compose
./deploy.sh
```

Or manually:

```bash
docker-compose up --build -d
```

The backend will be available at `http://localhost:8000`

## Frontend Deployment

### 1. Install Dependencies

```bash
cd docs
npm install
```

### 2. Build for Production

```bash
npm run build
```

### 3. Deploy Frontend

The frontend can be deployed in several ways:

#### Option A: Using Docusaurus Static Server
```bash
npm run serve
```

#### Option B: Deploy to GitHub Pages
```bash
GIT_USER=<your-github-username> npm run deploy
```

#### Option C: Deploy to Static Hosting (Netlify, Vercel, etc.)
Upload the contents of the `build/` directory to your static hosting provider.

## Configuration for Production

### Frontend API Connection

The chatbot component automatically detects the backend API URL:
- In development: connects to `http://localhost:8000`
- In production: connects to the same origin (protocol + hostname + port)

You can override this by setting the `REACT_APP_API_BASE_URL` environment variable during the build process:

```bash
REACT_APP_API_BASE_URL=https://your-backend-domain.com npm run build
```

### Environment Variables Reference

#### Backend (.env)
```bash
# AI Provider (gemini or openai)
AI_PROVIDER=gemini

# Gemini Configuration
GEMINI_API_KEY=your_api_key_here
GEMINI_MODEL=gemini-2.5-flash

# OpenAI Configuration (alternative to Gemini)
OPENAI_API_KEY=your_api_key_here
OPENAI_MODEL=gpt-4-turbo-preview

# Qdrant Configuration
QDRANT_API_KEY=your_api_key_here
QDRANT_HOST=your_cluster_url
QDRANT_PORT=6333
QDRANT_COLLECTION_NAME=rag_embeddings

# Application Settings
DEBUG_MODE=false
LOG_LEVEL=INFO
MAX_QUERY_LENGTH=1000
RESPONSE_TIMEOUT=30

# Server Settings
SERVER_HOST=0.0.0.0
SERVER_PORT=8000

# Additional Settings
COHERE_API_KEY=your_cohere_api_key_here
TARGET_URL=https://your-frontend-domain.com
```

## Health Checks and Monitoring

### Backend Health Check
- Health endpoint: `http://localhost:8000/health`
- API documentation: `http://localhost:8000/docs`

### Chatbot Service
- Query endpoint: `http://localhost:8000/api/v1/query`
- The chatbot in the frontend will automatically connect to this endpoint

## Troubleshooting

### Common Issues

1. **Connection refused between frontend and backend**
   - Ensure the backend service is running and accessible
   - Check CORS settings in the backend

2. **API keys not working**
   - Verify API keys are correctly set in the environment
   - Check that the AI provider is properly configured

3. **Qdrant connection issues**
   - Verify Qdrant credentials and URL
   - Ensure the Qdrant collection exists

### Useful Commands

```bash
# View backend logs
docker-compose logs -f agent-api

# Check service status
docker-compose ps

# Restart services
docker-compose restart

# Stop services
docker-compose down
```

## Scaling Considerations

- The backend uses multiple workers for handling concurrent requests
- Qdrant can be scaled independently for better performance
- Consider using a load balancer for production deployments
- Implement caching for frequently accessed content