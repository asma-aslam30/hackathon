"""
Background information API endpoints.
"""
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from ..config.database import get_async_db
from ..auth.better_auth import auth_manager
from ..services.background_service import background_service, BackgroundCreateUpdate
from ..models.user import User
from pydantic import BaseModel
from typing import Optional

router = APIRouter()

class BackgroundResponse(BaseModel):
    success: bool
    background: Optional[dict] = None

class BackgroundData(BaseModel):
    software_tools: Optional[list] = None
    hardware_setup: Optional[dict] = None
    programming_languages: Optional[list] = None
    technical_preferences: Optional[dict] = None
    experience_level: Optional[str] = None
    primary_domain: Optional[str] = None

@router.get("/me", response_model=dict)
async def get_current_user_background(
    current_user: User = Depends(auth_manager.get_current_user),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Get current user's background information.
    """
    background_info = await background_service.get_background_by_user_id(db, str(current_user.id))

    if not background_info:
        # Return empty background info if none exists
        return {
            "id": None,
            "user_id": str(current_user.id),
            "software_tools": [],
            "hardware_setup": {},
            "programming_languages": [],
            "technical_preferences": {},
            "experience_level": None,
            "primary_domain": None,
            "created_at": None,
            "updated_at": None
        }

    return {
        "id": str(background_info.id),
        "user_id": str(background_info.user_id),
        "software_tools": background_info.software_tools or [],
        "hardware_setup": background_info.hardware_setup or {},
        "programming_languages": background_info.programming_languages or [],
        "technical_preferences": background_info.technical_preferences or {},
        "experience_level": background_info.experience_level,
        "primary_domain": background_info.primary_domain,
        "created_at": background_info.created_at.isoformat() if background_info.created_at else None,
        "updated_at": background_info.updated_at.isoformat() if background_info.updated_at else None
    }

@router.post("/me", response_model=BackgroundResponse)
async def create_or_update_user_background(
    background_data: BackgroundCreateUpdate,
    current_user: User = Depends(auth_manager.get_current_user),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Create or update user's background information.
    """
    background_info = await background_service.create_or_update_background(
        db, str(current_user.id), background_data
    )

    return {
        "success": True,
        "background": {
            "id": str(background_info.id),
            "user_id": str(background_info.user_id),
            "software_tools": background_info.software_tools or [],
            "hardware_setup": background_info.hardware_setup or {},
            "programming_languages": background_info.programming_languages or [],
            "technical_preferences": background_info.technical_preferences or {},
            "experience_level": background_info.experience_level,
            "primary_domain": background_info.primary_domain,
            "created_at": background_info.created_at.isoformat() if background_info.created_at else None,
            "updated_at": background_info.updated_at.isoformat() if background_info.updated_at else None
        }
    }