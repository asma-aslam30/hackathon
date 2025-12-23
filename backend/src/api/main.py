"""
Main FastAPI application for the Agent Retrieval System.

This module initializes the FastAPI application and includes the API routes.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import logging
import sys
from dotenv import load_dotenv
from datetime import datetime
from src.api.routers import query_router
from src.api import translation as translation_router
from src.api import auth as auth_router
from src.utils.logging_utils import app_logger
from src.config.settings import settings
from src.config.database import async_engine, Base


# Load environment variables
load_dotenv()

# Initialize logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = app_logger.logger  # Use the configured logger

# Create FastAPI instance
app = FastAPI(
    title="Agent Retrieval API",
    description="API for the intelligent agent that retrieves and answers questions based on embeddings stored in Qdrant",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, configure this properly
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.on_event("startup")
async def startup_event():
    """Initialize the agent when the application starts."""
    logger.info("Application starting up...")

    # Create database tables
    async with async_engine.begin() as conn:
        # Import all models to ensure they're registered with Base
        from src.models import user, translation, background, personalization, session
        await conn.run_sync(Base.metadata.create_all)
        logger.info("Database tables created/verified")


@app.on_event("shutdown")
async def shutdown_event():
    """Clean up resources when the application shuts down."""
    logger.info("Application shutting down...")


# Include API routes
app.include_router(query_router.router, prefix="/api/v1", tags=["query"])
app.include_router(translation_router.router, prefix="/api/v1/translation", tags=["translation"])
app.include_router(auth_router.router, prefix="/api/v1/auth", tags=["auth"])


@app.get("/")
async def root():
    """Root endpoint for basic service information."""
    return {
        "message": "Agent Retrieval API",
        "version": "1.0.0",
        "description": "API for intelligent agent with retrieval capabilities",
        "endpoints": [
            "/api/v1/query - Submit a query to the agent",
            "/api/v1/health - Health check endpoint",
            "/api/v1/metrics - Get service metrics"
        ],
        "timestamp": datetime.now().isoformat()
    }


@app.get("/health", response_model=dict)
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "version": "1.0.0"
    }


@app.exception_handler(Exception)
async def global_exception_handler(request, exc):
    """Global exception handler for the application."""
    logger.error(f"Unhandled exception: {str(exc)}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={"detail": f"Internal server error: {str(exc)}"}
    )


if __name__ == "__main__":
    import uvicorn
    logger.info("Starting Agent Retrieval API server...")
    uvicorn.run(
        "src.api.main:app",
        host=settings.server_host,
        port=settings.server_port,
        reload=True
    )