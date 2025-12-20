# Quickstart: Agent Development with Retrieval Integration

## Prerequisites

- Python 3.11 or higher
- Access to OpenAI API key
- Access to Qdrant vector database (with existing embeddings from Spec 1)
- Git

## Setup Instructions

### 1. Clone and Navigate to Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Navigate to Backend Directory
```bash
cd backend
```

### 3. Set Up Python Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 4. Install Dependencies
```bash
pip install -r requirements.txt
```

### 5. Set Up Environment Variables
Create a `.env` file in the backend directory:
```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_HOST=your_qdrant_cluster_url
QDRANT_COLLECTION_NAME=rag_embeddings  # Collection from Spec 1
DEBUG_MODE=true  # Set to false for production
LOG_LEVEL=INFO
```

### 6. Start the Application
```bash
# Run the FastAPI application
uvicorn src.api.main:app --host 0.0.0.0 --port 8000 --reload
```

Or use the docker-compose if available:
```bash
docker-compose up --build
```

## API Usage

### Submit a Query
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is the main concept of the book?",
    "user_id": "optional-user-id"
  }'
```

### Expected Response
```json
{
  "response": "The main concept of the book is...",
  "query_id": "unique-query-id",
  "retrieved_chunks": [
    {
      "id": "chunk-id",
      "content": "Relevant content from the book...",
      "source_url": "source-document-url",
      "similarity_score": 0.85
    }
  ],
  "confidence_score": 0.92,
  "timestamp": "2025-12-18T10:30:00Z",
  "sources": ["source1", "source2"]
}
```

## Testing the Agent

### Run Unit Tests
```bash
pytest tests/unit/
```

### Run Integration Tests
```bash
pytest tests/integration/
```

### Test Specific Endpoints
```bash
# Test the health endpoint
curl http://localhost:8000/health

# Test a sample query
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"query": "Explain the key concepts in Chapter 1"}'
```

## Configuration Options

### Environment Variables
- `OPENAI_API_KEY`: Your OpenAI API key for agent access
- `QDRANT_HOST`: URL to your Qdrant cluster
- `QDRANT_API_KEY`: Your Qdrant API key
- `QDRANT_COLLECTION_NAME`: Name of the collection with embeddings (default: rag_embeddings)
- `DEBUG_MODE`: Enable/disable debug mode (default: false)
- `LOG_LEVEL`: Logging level (default: INFO)
- `MAX_QUERY_LENGTH`: Maximum length of user queries (default: 1000)
- `RESPONSE_TIMEOUT`: Timeout for agent responses in seconds (default: 30)

## Troubleshooting

### Common Issues

1. **Connection to Qdrant fails**
   - Verify your Qdrant credentials in `.env`
   - Check that the collection name matches the one from Spec 1
   - Ensure the Qdrant cluster is accessible

2. **Agent returns generic responses**
   - Verify that embeddings exist in the Qdrant collection
   - Check that the query is semantically similar to content in the database

3. **Slow response times**
   - Consider adding caching for frequent queries
   - Verify that your OpenAI API key has sufficient rate limits
   - Check that the Qdrant cluster is properly indexed

### Health Checks

Monitor these endpoints to ensure proper operation:
- `/health`: Basic service health
- `/metrics`: Performance metrics
- `/status`: Detailed system status