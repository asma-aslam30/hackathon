"""
API package initialization.
"""
from fastapi import APIRouter

# Main API router
api_router = APIRouter()

# Import and include all API routes here
from . import auth, background, profile, personalization, translation

# Include routers
api_router.include_router(auth.router, prefix="/auth", tags=["auth"])
api_router.include_router(background.router, prefix="/background", tags=["background"])
api_router.include_router(profile.router, prefix="/profile", tags=["profile"])
api_router.include_router(personalization.router, prefix="/personalization", tags=["personalization"])
api_router.include_router(translation.router, prefix="/translation", tags=["translation"])

# Try to include query router if available (RAG chatbot)
try:
    from .routers import query_router
    api_router.include_router(query_router.router, tags=["query"])
except ImportError as e:
    print(f"Note: Query router not available: {e}")