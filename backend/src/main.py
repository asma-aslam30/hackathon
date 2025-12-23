"""
Main entry point for the User Authentication API.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .api import api_router
from .config.settings import settings
from contextlib import asynccontextmanager

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup logic
    print("Starting User Authentication API...")

    # Create database tables if needed
    from .database.migrations import create_tables
    create_tables()

    # Initialize retrieval agent if available
    try:
        from .api.routers.query_router import retrieval_agent
        success = await retrieval_agent.initialize()
        if success:
            print("Retrieval agent initialized successfully")
        else:
            print("Warning: Failed to initialize retrieval agent")
    except Exception as e:
        print(f"Note: Retrieval agent not available: {e}")

    yield

    # Shutdown logic
    print("Shutting down User Authentication API...")

    # Cleanup retrieval agent if available
    try:
        from .api.routers.query_router import retrieval_agent
        await retrieval_agent.cleanup()
    except Exception:
        pass

# Create FastAPI app
app = FastAPI(
    title=settings.app_name,
    version=settings.app_version,
    debug=settings.debug_mode,
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[settings.frontend_url, "http://localhost:3000", "http://localhost:3001"],  # Adjust as needed
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(api_router, prefix="/api/v1")

@app.get("/")
async def root():
    return {"message": "User Authentication API", "version": settings.app_version}

@app.get("/health")
async def health_check():
    return {"status": "healthy", "version": settings.app_version}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.main:app",
        host=settings.server_host,
        port=settings.server_port,
        reload=True  # Set to False in production
    )