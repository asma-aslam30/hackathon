"""
Translation API endpoints for Urdu translation.

This module provides REST API endpoints for translating content to Urdu.
"""
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import HTTPBearer
from sqlalchemy.ext.asyncio import AsyncSession
import logging

from ..config.database import get_async_db
from ..auth.better_auth import auth_manager
from ..models.user import User
from ..models.translation_schemas import (
    TranslationRequest,
    TranslationResponse,
    TranslationError,
    ServiceStatus,
    CacheClearResponse
)
from ..services.translation_service import TranslationService

logger = logging.getLogger(__name__)
router = APIRouter()
security = HTTPBearer()


@router.post(
    "/translate-to-urdu",
    response_model=TranslationResponse,
    responses={
        400: {"model": TranslationError, "description": "Invalid request"},
        401: {"model": TranslationError, "description": "Authentication required"},
        429: {"model": TranslationError, "description": "Rate limited"},
        500: {"model": TranslationError, "description": "Translation service error"}
    },
    summary="Translate content to Urdu",
    description="Translates HTML content from the source language to Urdu. Requires authentication. Results are cached for 24 hours."
)
async def translate_to_urdu(
    request: TranslationRequest,
    current_user: User = Depends(auth_manager.get_current_user),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Translate content to Urdu.

    - **content**: HTML content to translate (1-100,000 characters)
    - **chapter_id**: Optional chapter identifier for analytics
    - **source_language**: ISO 639-1 source language code (default: en)
    - **preserve_formatting**: Whether to preserve HTML formatting (default: true)

    Returns translated content with RTL attributes and translation metadata.
    """
    try:
        service = TranslationService(db)
        response = await service.translate_to_urdu(
            request=request,
            user_id=current_user.id
        )
        return response

    except ValueError as e:
        logger.warning(f"Translation validation error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=TranslationError(
                success=False,
                error_code="INVALID_CONTENT",
                error_message=str(e),
                retry_after=None
            ).model_dump()
        )

    except RuntimeError as e:
        logger.error(f"Translation service error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=TranslationError(
                success=False,
                error_code="SERVICE_ERROR",
                error_message="Translation service unavailable. Please try again later.",
                retry_after=60
            ).model_dump()
        )

    except Exception as e:
        logger.error(f"Unexpected translation error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=TranslationError(
                success=False,
                error_code="SERVICE_ERROR",
                error_message="An unexpected error occurred. Please try again.",
                retry_after=30
            ).model_dump()
        )


@router.get(
    "/status",
    response_model=ServiceStatus,
    summary="Get translation service status",
    description="Returns the current status of the translation service (health check)"
)
async def get_translation_status(
    db: AsyncSession = Depends(get_async_db)
):
    """
    Get translation service status.

    Returns service health, cache statistics, and supported languages.
    """
    try:
        service = TranslationService(db)
        status_info = await service.get_service_status()
        return ServiceStatus(**status_info)

    except Exception as e:
        logger.error(f"Failed to get service status: {str(e)}")
        return ServiceStatus(
            status="degraded",
            cache_entries=0,
            avg_translation_time_ms=0,
            supported_languages=[
                {"code": "ur", "name": "Urdu", "direction": "rtl"}
            ]
        )


@router.delete(
    "/cache/clear",
    response_model=CacheClearResponse,
    summary="Clear user's translation cache",
    description="Clears all cached translations for the authenticated user"
)
async def clear_translation_cache(
    current_user: User = Depends(auth_manager.get_current_user),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Clear the current user's translation cache.

    This removes all cached translations associated with the authenticated user.
    """
    try:
        service = TranslationService(db)
        cleared_count = await service.clear_user_cache(current_user.id)

        return CacheClearResponse(
            success=True,
            cleared_count=cleared_count
        )

    except Exception as e:
        logger.error(f"Failed to clear cache: {str(e)}")
        return CacheClearResponse(
            success=False,
            cleared_count=0
        )
