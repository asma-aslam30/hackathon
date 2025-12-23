"""
Pydantic schemas for Urdu translation API.

This module defines request/response schemas for the translation endpoints.
"""
from datetime import datetime
from typing import Optional
from pydantic import BaseModel, Field, field_validator
import re


class TranslationRequest(BaseModel):
    """Request body for translation endpoint."""
    content: str = Field(
        ...,
        min_length=1,
        max_length=100000,
        description="HTML content to translate"
    )
    chapter_id: Optional[str] = Field(
        None,
        max_length=255,
        description="Optional chapter identifier for analytics"
    )
    source_language: str = Field(
        default="en",
        description="ISO 639-1 source language code"
    )
    preserve_formatting: bool = Field(
        default=True,
        description="Whether to preserve HTML formatting"
    )

    @field_validator('source_language')
    @classmethod
    def validate_language_code(cls, v: str) -> str:
        """Validate ISO 639-1 language code format."""
        if not re.match(r'^[a-z]{2}$', v):
            raise ValueError('source_language must be a 2-letter ISO 639-1 code')
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "content": "<p>Hello, world!</p>",
                "chapter_id": "chapter-1-introduction",
                "source_language": "en",
                "preserve_formatting": True
            }
        }


class TranslationMetadata(BaseModel):
    """Metadata about the translation operation."""
    word_count: int = Field(..., ge=1, description="Word count of original content")
    translation_time_ms: int = Field(..., ge=0, description="Translation time in milliseconds")
    source_language: str = Field(..., description="Source language code")
    target_language: str = Field(default="ur", description="Target language (Urdu)")
    cached_at: Optional[datetime] = Field(None, description="When the translation was cached (null if fresh)")


class TranslationResponse(BaseModel):
    """Response body for translation endpoint."""
    success: bool = Field(..., description="Whether translation succeeded")
    translated_content: str = Field(..., description="Urdu translated HTML content with RTL attributes")
    metadata: TranslationMetadata = Field(..., description="Translation metadata")
    cache_hit: bool = Field(..., description="Whether result was served from cache")

    class Config:
        json_schema_extra = {
            "example": {
                "success": True,
                "translated_content": "<p dir='rtl' lang='ur'>ہیلو، دنیا!</p>",
                "metadata": {
                    "word_count": 2,
                    "translation_time_ms": 1250,
                    "source_language": "en",
                    "target_language": "ur",
                    "cached_at": None
                },
                "cache_hit": False
            }
        }


class TranslationError(BaseModel):
    """Error response schema."""
    success: bool = Field(default=False)
    error_code: str = Field(..., description="Machine-readable error code")
    error_message: str = Field(..., description="Human-readable error message")
    retry_after: Optional[int] = Field(None, ge=0, description="Seconds to wait before retrying (for rate limits)")

    class Config:
        json_schema_extra = {
            "example": {
                "success": False,
                "error_code": "RATE_LIMITED",
                "error_message": "Translation service busy. Please try again.",
                "retry_after": 30
            }
        }


class ServiceStatus(BaseModel):
    """Service health status response."""
    status: str = Field(..., description="Current service health status")
    cache_entries: int = Field(..., ge=0, description="Number of cached translations")
    avg_translation_time_ms: int = Field(..., ge=0, description="Average translation time in milliseconds")
    supported_languages: list[dict] = Field(
        default_factory=lambda: [
            {"code": "ur", "name": "Urdu", "direction": "rtl"}
        ],
        description="List of supported target languages"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "status": "healthy",
                "cache_entries": 150,
                "avg_translation_time_ms": 1200,
                "supported_languages": [
                    {"code": "ur", "name": "Urdu", "direction": "rtl"}
                ]
            }
        }


class CacheClearResponse(BaseModel):
    """Response for cache clear operation."""
    success: bool = Field(..., description="Whether cache was cleared successfully")
    cleared_count: int = Field(..., ge=0, description="Number of cache entries cleared")
