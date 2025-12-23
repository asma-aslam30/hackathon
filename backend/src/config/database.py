from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import declarative_base
from typing import AsyncGenerator
from .settings import settings
import os
import ssl

# Get database URL from settings
if settings.neon_database_url:
    SQLALCHEMY_DATABASE_URL = settings.neon_database_url
else:
    SQLALCHEMY_DATABASE_URL = settings.database_url

# Convert asyncpg URL to psycopg2 URL for sync engine
# asyncpg uses ssl=require, psycopg2 uses sslmode=require
SYNC_DATABASE_URL = SQLALCHEMY_DATABASE_URL.replace("postgresql+asyncpg://", "postgresql://").replace("?ssl=require", "?sslmode=require")

# For async operations
async_engine = create_async_engine(
    SQLALCHEMY_DATABASE_URL,
    echo=settings.debug_mode,  # Debug mode controls echo
    pool_pre_ping=True,  # Test connections before use
    pool_recycle=300,  # Recycle connections after 5 minutes
)

# Synchronous engine (for compatibility if needed) - uses regular postgresql driver
sync_engine = create_engine(
    SYNC_DATABASE_URL,
    echo=settings.debug_mode,  # Debug mode controls echo
)

# Session classes
AsyncSessionLocal = sessionmaker(
    autocommit=False,
    autoflush=False,
    bind=async_engine,
    class_=AsyncSession,
    expire_on_commit=False,  # Prevent expired object errors
)

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=sync_engine)

# Base class for models
Base = declarative_base()

# Dependency to get async database session
async def get_async_db() -> AsyncGenerator[AsyncSession, None]:
    async with AsyncSessionLocal() as db:
        try:
            yield db
        finally:
            await db.close()