"""
Configuration settings for the Agent Retrieval System.

This module manages all configuration settings using Pydantic settings.
"""
from pydantic_settings import BaseSettings
from typing import Optional
import os
from functools import lru_cache


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Application info
    app_name: str = os.getenv("APP_NAME", "User Authentication API")
    app_version: str = os.getenv("APP_VERSION", "1.0.0")

    # AI Provider settings
    ai_provider: str = os.getenv("AI_PROVIDER", "openai")  # Can be 'openai' or 'gemini'
    openai_api_key: str = os.getenv("OPENAI_API_KEY", "")
    openai_model: str = os.getenv("OPENAI_MODEL", "gpt-4-turbo-preview")
    gemini_api_key: str = os.getenv("GEMINI_API_KEY", "")
    gemini_model: str = os.getenv("GEMINI_MODEL", "gemini-2.5-flash")

    # Qdrant settings
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_host: str = os.getenv("QDRANT_HOST", "localhost")
    qdrant_port: int = int(os.getenv("QDRANT_PORT", "6333"))
    qdrant_https: bool = os.getenv("QDRANT_HTTPS", "false").lower() == "true"
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "rag_embeddings")

    # Authentication settings
    secret_key: str = os.getenv("SECRET_KEY", "your-secret-key-here")
    algorithm: str = os.getenv("ALGORITHM", "HS256")
    access_token_expire_minutes: int = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))
    session_expire_days: int = int(os.getenv("SESSION_EXPIRE_DAYS", "7"))
    better_auth_secret: str = os.getenv("BETTER_AUTH_SECRET", "")

    # Database settings
    database_url: str = os.getenv("DATABASE_URL", "postgresql+asyncpg://user:password@localhost/dbname")
    neon_database_url: str = os.getenv("NEON_DATABASE_URL", "")

    # Application settings
    debug_mode: bool = os.getenv("DEBUG_MODE", "false").lower() == "true"
    log_level: str = os.getenv("LOG_LEVEL", "INFO")
    max_query_length: int = int(os.getenv("MAX_QUERY_LENGTH", "1000"))
    response_timeout: int = int(os.getenv("RESPONSE_TIMEOUT", "30"))

    # Server settings
    server_host: str = os.getenv("SERVER_HOST", "0.0.0.0")
    server_port: int = int(os.getenv("SERVER_PORT", "8000"))
    frontend_url: str = os.getenv("FRONTEND_URL", "http://localhost:3000")

    # Additional settings for embedding generation
    source_content_path: str = os.getenv("SOURCE_CONTENT_PATH", "./test_data/original_content.json")
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
    target_url: str = os.getenv("TARGET_URL", "https://example.com")

    # Translation settings
    translation_cache_ttl_hours: int = int(os.getenv("TRANSLATION_CACHE_TTL_HOURS", "24"))
    translation_max_content_length: int = int(os.getenv("TRANSLATION_MAX_CONTENT_LENGTH", "100000"))
    translation_rate_limit_per_minute: int = int(os.getenv("TRANSLATION_RATE_LIMIT_PER_MINUTE", "10"))
    translation_provider: str = os.getenv("TRANSLATION_PROVIDER", "gemini")  # gemini or openai

    class Config:
        env_file = ".env"
        case_sensitive = False

    def get_qdrant_url(self) -> str:
        """Get the complete Qdrant URL."""
        protocol = "https" if self.qdrant_https else "http"
        return f"{protocol}://{self.qdrant_host}:{self.qdrant_port}"

    def get_openai_client_params(self) -> dict:
        """Get parameters for initializing OpenAI client."""
        return {
            "api_key": self.openai_api_key,
        }

    def get_gemini_client_params(self) -> dict:
        """Get parameters for initializing Gemini client."""
        return {
            "api_key": self.gemini_api_key,
            "model_name": self.gemini_model,
        }


@lru_cache()
def get_settings() -> Settings:
    """Cached settings instance to avoid reloading from environment."""
    return Settings()


def validate_settings(settings: Settings) -> list[str]:
    """Validate settings and return list of validation errors."""
    errors = []

    # Validate AI provider configuration
    if settings.ai_provider.lower() not in ['openai', 'gemini']:
        errors.append("AI_PROVIDER must be either 'openai' or 'gemini'")

    # Validate required API key based on provider
    if settings.ai_provider.lower() == 'openai' and not settings.openai_api_key:
        errors.append("OPENAI_API_KEY is required when AI_PROVIDER is 'openai'")
    elif settings.ai_provider.lower() == 'gemini' and not settings.gemini_api_key:
        errors.append("GEMINI_API_KEY is required when AI_PROVIDER is 'gemini'")

    if not settings.qdrant_api_key:
        errors.append("QDRANT_API_KEY is required")

    if not settings.qdrant_host:
        errors.append("QDRANT_HOST is required")

    # Validate model name format if provided
    if settings.ai_provider.lower() == 'openai' and settings.openai_model and not settings.openai_model.startswith("gpt-"):
        errors.append("OPENAI_MODEL should start with 'gpt-'")
    elif settings.ai_provider.lower() == 'gemini' and settings.gemini_model and not settings.gemini_model.startswith("gemini-"):
        errors.append("GEMINI_MODEL should start with 'gemini-'")

    # Validate port range
    if settings.qdrant_port < 1 or settings.qdrant_port > 65535:
        errors.append("QDRANT_PORT must be between 1 and 65535")

    # Validate cohere API key if needed for embedding generation
    if not settings.cohere_api_key:
        errors.append("COHERE_API_KEY is required for embedding generation")

    return errors


# Create a global settings instance
settings = get_settings()

# Validate settings at startup (warnings only, don't fail)
validation_errors = validate_settings(settings)
if validation_errors:
    import warnings
    for error in validation_errors:
        warnings.warn(f"Configuration warning: {error}")


def get_openai_client():
    """Initialize and return OpenAI client with configured parameters."""
    from openai import OpenAI

    # Simple initialization with just api_key (newer OpenAI SDK)
    return OpenAI(api_key=settings.openai_api_key)


def get_gemini_client():
    """Initialize and return Google Generative AI client with configured parameters."""
    try:
        import google.generativeai as genai

        params = settings.get_gemini_client_params()
        genai.configure(api_key=params["api_key"])
        model = genai.GenerativeModel(params["model_name"])
        return model
    except ImportError:
        raise ImportError("google-generativeai package is required for Gemini integration. Install it with: pip install google-generativeai")


def get_ai_client():
    """Initialize and return the appropriate AI client based on the configured provider."""
    if settings.ai_provider.lower() == 'gemini':
        return get_gemini_client()
    elif settings.ai_provider.lower() == 'openai':
        return get_openai_client()
    else:
        raise ValueError(f"Unsupported AI provider: {settings.ai_provider}. Use 'openai' or 'gemini'.")


def get_qdrant_client():
    """Initialize and return Qdrant client with configured parameters."""
    from qdrant_client import QdrantClient

    if settings.qdrant_host.startswith("http"):
        # Cloud instance
        client = QdrantClient(
            url=settings.qdrant_host,
            api_key=settings.qdrant_api_key,
            timeout=10.0
        )
    else:
        # Local instance
        client = QdrantClient(
            host=settings.qdrant_host,
            port=settings.qdrant_port,
            api_key=settings.qdrant_api_key,
            timeout=10.0
        )

    return client