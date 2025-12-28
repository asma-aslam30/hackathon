"""
Full FastAPI backend for Hugging Face Spaces.
Includes: Auth, Query, Profile, Translation endpoints.
"""
import os
import sys
import hashlib
import secrets
from datetime import datetime, timedelta
from typing import Optional, List, Dict, Any

from fastapi import FastAPI, HTTPException, Depends, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel, EmailStr
import google.generativeai as genai
from qdrant_client import QdrantClient
import jwt

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
        "http://localhost:3001",
        "*"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Security
security = HTTPBearer(auto_error=False)
SECRET_KEY = os.getenv("SECRET_KEY", "hackathon-secret-key-2024")
ALGORITHM = "HS256"

# In-memory storage (for demo - use database in production)
users_db: Dict[str, dict] = {}
sessions_db: Dict[str, dict] = {}

# ============== MODELS ==============

class UserRegister(BaseModel):
    name: str
    email: EmailStr
    password: str

class UserLogin(BaseModel):
    email: EmailStr
    password: str

class UserResponse(BaseModel):
    id: str
    name: str
    email: str
    created_at: str

class AuthResponse(BaseModel):
    success: bool
    session_token: str
    user: UserResponse

class QueryRequest(BaseModel):
    query: str
    userId: Optional[str] = None
    sessionId: Optional[str] = None

class QueryResponse(BaseModel):
    response: str
    query_id: Optional[str] = None
    sources: List[str] = []
    confidence_score: Optional[float] = None

class TranslationRequest(BaseModel):
    content: str
    chapter_id: Optional[str] = None
    source_language: str = "en"
    preserve_formatting: bool = True

# ============== HELPERS ==============

def hash_password(password: str) -> str:
    return hashlib.sha256(password.encode()).hexdigest()

def create_token(user_id: str) -> str:
    payload = {
        "sub": user_id,
        "exp": datetime.utcnow() + timedelta(days=7)
    }
    return jwt.encode(payload, SECRET_KEY, algorithm=ALGORITHM)

def verify_token(token: str) -> Optional[str]:
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload.get("sub")
    except:
        return None

async def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)):
    if not credentials:
        raise HTTPException(status_code=401, detail="Not authenticated")

    user_id = verify_token(credentials.credentials)
    if not user_id or user_id not in users_db:
        raise HTTPException(status_code=401, detail="Invalid token")

    return users_db[user_id]

def get_gemini_client():
    api_key = os.getenv("GEMINI_API_KEY")
    if api_key:
        genai.configure(api_key=api_key)
        return genai.GenerativeModel(os.getenv("GEMINI_MODEL", "gemini-2.5-flash"))
    return None

# ============== ROOT ENDPOINTS ==============

@app.get("/")
async def root():
    return {"message": "Physical AI Chatbot API", "status": "running", "version": "1.0.0"}

@app.get("/health")
async def health():
    return {"status": "healthy", "version": "1.0.0"}

# ============== AUTH ENDPOINTS ==============

@app.post("/api/v1/auth/register", response_model=AuthResponse)
async def register(data: UserRegister):
    # Check if email exists
    for user in users_db.values():
        if user["email"] == data.email:
            raise HTTPException(status_code=400, detail="Email already registered")

    # Create user
    user_id = secrets.token_hex(16)
    hashed_pw = hash_password(data.password)

    users_db[user_id] = {
        "id": user_id,
        "name": data.name,
        "email": data.email,
        "password": hashed_pw,
        "created_at": datetime.utcnow().isoformat()
    }

    # Create session token
    token = create_token(user_id)
    sessions_db[token] = {"user_id": user_id, "created_at": datetime.utcnow().isoformat()}

    return AuthResponse(
        success=True,
        session_token=token,
        user=UserResponse(
            id=user_id,
            name=data.name,
            email=data.email,
            created_at=users_db[user_id]["created_at"]
        )
    )

@app.post("/api/v1/auth/login", response_model=AuthResponse)
async def login(data: UserLogin):
    # Find user by email
    user = None
    for u in users_db.values():
        if u["email"] == data.email:
            user = u
            break

    if not user:
        raise HTTPException(status_code=401, detail="Invalid email or password")

    # Verify password
    if user["password"] != hash_password(data.password):
        raise HTTPException(status_code=401, detail="Invalid email or password")

    # Create session token
    token = create_token(user["id"])
    sessions_db[token] = {"user_id": user["id"], "created_at": datetime.utcnow().isoformat()}

    return AuthResponse(
        success=True,
        session_token=token,
        user=UserResponse(
            id=user["id"],
            name=user["name"],
            email=user["email"],
            created_at=user["created_at"]
        )
    )

@app.post("/api/v1/auth/logout")
async def logout(credentials: HTTPAuthorizationCredentials = Depends(security)):
    if credentials and credentials.credentials in sessions_db:
        del sessions_db[credentials.credentials]
    return {"success": True, "message": "Logged out successfully"}

@app.get("/api/v1/auth/me")
async def get_me(user: dict = Depends(get_current_user)):
    return UserResponse(
        id=user["id"],
        name=user["name"],
        email=user["email"],
        created_at=user["created_at"]
    )

# ============== QUERY ENDPOINT ==============

@app.post("/api/v1/query", response_model=QueryResponse)
async def query(request: QueryRequest):
    try:
        model = get_gemini_client()
        if not model:
            raise HTTPException(status_code=500, detail="AI model not configured")

        prompt = f"""You are a helpful assistant for Physical AI and Humanoid Robotics.
        You help users learn about robotics, AI in physical systems, sensors, actuators,
        control systems, and related topics.

        Answer the following question concisely and helpfully:

        Question: {request.query}
        """

        response = model.generate_content(prompt)

        return QueryResponse(
            response=response.text,
            query_id=secrets.token_hex(8),
            sources=["Physical AI & Humanoid Robotics Documentation"],
            confidence_score=0.9
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# ============== PROFILE ENDPOINTS ==============

@app.get("/api/v1/profile")
async def get_profile(user: dict = Depends(get_current_user)):
    return {
        "id": user["id"],
        "name": user["name"],
        "email": user["email"],
        "created_at": user["created_at"],
        "background": user.get("background", {})
    }

@app.put("/api/v1/profile")
async def update_profile(data: dict, user: dict = Depends(get_current_user)):
    if "name" in data:
        users_db[user["id"]]["name"] = data["name"]
    return {"success": True, "message": "Profile updated"}

@app.post("/api/v1/profile/background")
async def update_background(data: dict, user: dict = Depends(get_current_user)):
    users_db[user["id"]]["background"] = data
    return {"success": True, "message": "Background updated"}

# ============== TRANSLATION ENDPOINT ==============

@app.post("/api/v1/translation/translate-to-urdu")
async def translate_to_urdu(
    request: TranslationRequest,
    credentials: HTTPAuthorizationCredentials = Depends(security)
):
    if not credentials:
        raise HTTPException(status_code=401, detail="Authentication required")

    try:
        model = get_gemini_client()
        if not model:
            raise HTTPException(status_code=500, detail="AI model not configured")

        prompt = f"""Translate the following English text to Urdu.
        Maintain the meaning and tone. If there are technical terms, keep them in English with Urdu explanation.

        Text to translate:
        {request.content}
        """

        response = model.generate_content(prompt)

        return {
            "translated_content": response.text,
            "source_language": request.source_language,
            "target_language": "ur",
            "chapter_id": request.chapter_id
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/v1/translation/status")
async def translation_status():
    return {
        "status": "healthy",
        "supported_languages": [{"code": "ur", "name": "Urdu", "direction": "rtl"}]
    }

# ============== PERSONALIZATION ENDPOINT ==============

@app.get("/api/v1/personalization/content/{chapter_id}")
async def get_personalized_content(
    chapter_id: str,
    user: dict = Depends(get_current_user)
):
    return {
        "chapter_id": chapter_id,
        "personalized": True,
        "user_background": user.get("background", {}),
        "message": "Personalization based on user background"
    }

@app.post("/api/v1/personalization/preferences")
async def update_preferences(data: dict, user: dict = Depends(get_current_user)):
    users_db[user["id"]]["preferences"] = data
    return {"success": True, "message": "Preferences updated"}

# ============== MAIN ==============

if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("PORT", "7860"))
    uvicorn.run(app, host="0.0.0.0", port=port)
