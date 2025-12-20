# Quickstart: Initial Project Setup

## Prerequisites
- Python 3.11 or higher
- Cohere API key
- Qdrant Cloud account and API key
- Git

## Setup Instructions

### 1. Clone and Navigate to Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create Backend Directory
```bash
mkdir backend
cd backend
```

### 3. Set Up Python Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 4. Install UV Package Manager (if not already installed)
```bash
pip install uv
```

### 5. Create requirements.txt
```txt
cohere==5.5.8
qdrant-client==1.9.2
requests==2.31.0
beautifulsoup4==4.12.2
python-dotenv==1.0.0
tiktoken==0.5.1
pytest==7.4.3
```

### 6. Install Dependencies
```bash
pip install -r requirements.txt
```

### 7. Set Up Environment Variables
Create a `.env` file in the backend directory:
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_HOST=your_qdrant_cluster_url
SOURCE_URL=https://asma-aslam30.github.io/hackathon/
```

### 8. Run the Application
```bash
python main.py
```

## Expected Output
- All URLs from the source site will be discovered
- Text will be extracted from each URL
- Text will be chunked into 500-1000 token segments
- Embeddings will be generated using Cohere
- Embeddings will be stored in Qdrant with metadata
- Validation queries will confirm retrieval accuracy

## Troubleshooting
- If you get API errors, verify your API keys in `.env`
- If URL extraction fails, check that the source URL is accessible
- If chunking seems incorrect, adjust the token size parameters
- If Qdrant storage fails, verify your cluster URL and credentials