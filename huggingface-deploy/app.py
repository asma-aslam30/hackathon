"""
Simplified FastAPI backend for Hugging Face Spaces.
"""
import os
import sys

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List
import google.generativeai as genai
from qdrant_client import QdrantClient

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI Chatbot API",
    version="1.0.0"
)

# Add CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://asma-aslam30.github.io",
        "http://localhost:3000",
        "*"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request/Response models
class QueryRequest(BaseModel):
    query: str
    userId: Optional[str] = None
    sessionId: Optional[str] = None

class QueryResponse(BaseModel):
    response: str
    query_id: Optional[str] = None
    sources: List[str] = []
    confidence_score: Optional[float] = None

# Initialize clients
gemini_model = None
qdrant_client = None

def get_gemini_client():
    global gemini_model
    if gemini_model is None:
        api_key = os.getenv("GEMINI_API_KEY")
        if api_key:
            genai.configure(api_key=api_key)
            gemini_model = genai.GenerativeModel("gemini-2.5-flash")
    return gemini_model

def get_qdrant_client():
    global qdrant_client
    if qdrant_client is None:
        host = os.getenv("QDRANT_HOST")
        api_key = os.getenv("QDRANT_API_KEY")
        if host and api_key:
            qdrant_client = QdrantClient(url=host, api_key=api_key)
    return qdrant_client

@app.get("/")
async def root():
    return {"message": "Physical AI Chatbot API", "status": "running"}

@app.get("/health")
async def health():
    return {"status": "healthy"}

@app.post("/api/v1/query", response_model=QueryResponse)
async def query(request: QueryRequest):
    try:
        model = get_gemini_client()
        if not model:
            raise HTTPException(status_code=500, detail="AI model not configured")

        # Generate response using Gemini
        prompt = f"""You are a helpful assistant for Physical AI and Humanoid Robotics.
        Answer the following question concisely and helpfully:

        Question: {request.query}
        """

        response = model.generate_content(prompt)

        return QueryResponse(
            response=response.text,
            sources=["Physical AI & Humanoid Robotics Documentation"],
            confidence_score=0.9
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("PORT", "7860"))
    uvicorn.run(app, host="0.0.0.0", port=port)
