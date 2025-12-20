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

    # Application settings
    debug_mode: bool = os.getenv("DEBUG_MODE", "false").lower() == "true"
    log_level: str = os.getenv("LOG_LEVEL", "INFO")
    max_query_length: int = int(os.getenv("MAX_QUERY_LENGTH", "1000"))
    response_timeout: int = int(os.getenv("RESPONSE_TIMEOUT", "30"))

    # Server settings
    server_host: str = os.getenv("SERVER_HOST", "0.0.0.0")
    server_port: int = int(os.getenv("SERVER_PORT", "8000"))

    # Additional settings for embedding generation
    source_content_path: str = os.getenv("SOURCE_CONTENT_PATH", "./test_data/original_content.json")
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
    target_url: str = os.getenv("TARGET_URL", "https://example.com")

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

# Validate settings at startup
validation_errors = validate_settings(settings)
if validation_errors:
    raise ValueError(f"Configuration validation failed: {'; '.join(validation_errors)}")


def get_openai_client():
    """Initialize and return OpenAI client with configured parameters."""
    from openai import OpenAI

    params = settings.get_openai_client_params()
    return OpenAI(**params)


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