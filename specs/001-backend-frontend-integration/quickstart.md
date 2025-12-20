# Quickstart Guide: Backend-Frontend Integration

## Prerequisites

- Node.js 16+ for Docusaurus frontend
- Python 3.11+ for FastAPI backend
- Access to Qdrant vector database with book embeddings
- API keys for AI provider (OpenAI or Gemini)

## Setting Up the Integration

### 1. Backend Setup

1. Ensure the FastAPI backend is running:
   ```bash
   cd backend
   pip install -r requirements.txt
   uvicorn src.api.main:app --host 0.0.0.0 --port 8000
   ```

2. Verify the backend is accessible at `http://localhost:8000`
3. Check that the query endpoint is available at `http://localhost:8000/api/v1/query`

### 2. Frontend Integration

1. Create the chatbot component in your Docusaurus project:
   ```bash
   # In your Docusaurus project directory
   mkdir src/components/Chatbot
   ```

2. Add the chatbot component files:
   - `Chatbot.jsx` - Main chatbot component
   - `Chatbot.css` - Styling for the chatbot
   - `ChatbotService.js` - API communication logic

3. Update your Docusaurus configuration to include the chatbot component

### 3. API Communication

The frontend will communicate with the backend using the following endpoints:

- **Query endpoint**: `POST /api/v1/query`
- **Health check**: `GET /api/v1/health`

Example API call from frontend:
```javascript
const response = await fetch('http://localhost:8000/api/v1/query', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
  },
  body: JSON.stringify({
    query: 'Your question here',
    userId: 'optional-user-id',
    sessionId: 'optional-session-id'
  })
});
```

### 4. Environment Configuration

Ensure the following environment variables are configured:

**Backend (.env)**:
- `AI_PROVIDER` - Set to 'openai' or 'gemini'
- `OPENAI_API_KEY` or `GEMINI_API_KEY` - API key for your chosen provider
- `QDRANT_API_KEY` - API key for Qdrant database
- `QDRANT_HOST` - Host URL for Qdrant database

### 5. Testing the Integration

1. Start both frontend and backend servers
2. Navigate to a book page in your Docusaurus site
3. Use the chatbot interface to submit a query about the book content
4. Verify that you receive a relevant response within 10 seconds

### 6. Troubleshooting

**Common Issues**:
- If the chatbot doesn't appear, check that the component is properly imported in your layout
- If API calls fail, verify that the backend is running and accessible
- If responses are inaccurate, check that the Qdrant database contains the correct embeddings

**CORS Issues**:
If you encounter CORS errors, ensure the backend allows requests from your frontend origin by updating the CORS middleware in `src/api/main.py`.

## Next Steps

1. Customize the chatbot UI to match your site's design
2. Add analytics to track user queries and engagement
3. Implement additional features like conversation history
4. Set up monitoring for the API endpoints