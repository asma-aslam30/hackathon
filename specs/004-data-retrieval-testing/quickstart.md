# Quickstart: Data Retrieval and Pipeline Testing

## Prerequisites
- Python 3.11 or higher
- Access to Qdrant database with existing embeddings from Spec 1
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

### 4. Install Test Dependencies
```bash
pip install -r requirements-test.txt
```

### 5. Create requirements-test.txt
```txt
qdrant-client==1.9.2
python-dotenv==1.0.0
pytest==7.4.3
numpy==1.24.3
```

### 6. Set Up Environment Variables
Create a `.env` file in the backend directory:
```env
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_HOST=your_qdrant_cluster_url
QDRANT_COLLECTION_NAME=rag_embeddings  # Collection from Spec 1
SOURCE_CONTENT_PATH=./test_data/original_content.json  # Path to original content for comparison
```

### 7. Run the Retrieval Tests
```bash
python test_retrieval_pipeline.py
```

## Expected Output
- Connection to Qdrant database established successfully
- All stored embeddings verified (100% success rate)
- Sample queries executed with relevant results returned
- Retrieval accuracy measured and documented (>90% threshold)
- Pipeline issues identified and reported (if any)
- Comprehensive test results documented

## Testing Coverage
The script will:
- Verify all embeddings from Spec 1 are accessible
- Execute sample queries covering at least 80% of book content
- Compare retrieved results with original text for accuracy
- Generate detailed accuracy metrics
- Document all findings and results

## Troubleshooting
- If you get connection errors, verify your Qdrant credentials in `.env`
- If accuracy is below threshold, check the original content path
- If queries fail, ensure the collection name matches Spec 1
- Check logs for detailed error information