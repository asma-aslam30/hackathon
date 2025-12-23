"""
User profile API endpoints.
"""
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from ..config.database import get_async_db
from ..auth.better_auth import auth_manager
from ..services.user_service import user_service, UserUpdate
from ..models.user import User
from pydantic import BaseModel
from typing import Optional

router = APIRouter()

class ProfileResponse(BaseModel):
    success: bool
    profile: Optional[dict] = None

class ProfileUpdateData(BaseModel):
    name: Optional[str] = None
    avatar_url: Optional[str] = None

@router.get("/me", response_model=dict)
async def get_current_user_profile(
    current_user: User = Depends(auth_manager.get_current_user),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Get current user's profile information.
    """
    return {
        "id": str(current_user.id),
        "email": current_user.email,
        "name": current_user.name,
        "avatar_url": current_user.avatar_url,
        "created_at": current_user.created_at.isoformat() if current_user.created_at else None,
        "updated_at": current_user.updated_at.isoformat() if current_user.updated_at else None,
        "last_login_at": current_user.last_login_at.isoformat() if current_user.last_login_at else None,
        "is_active": current_user.is_active,
        "email_verified": current_user.email_verified
    }

@router.put("/me", response_model=ProfileResponse)
async def update_current_user_profile(
    profile_data: ProfileUpdateData,
    current_user: User = Depends(auth_manager.get_current_user),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Update current user's profile information.
    """
    updated_user = await user_service.update_user(
        db, str(current_user.id), UserUpdate(name=profile_data.name, avatar_url=profile_data.avatar_url)
    )

    if not updated_user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )

    return {
        "success": True,
        "profile": {
            "id": str(updated_user.id),
            "email": updated_user.email,
            "name": updated_user.name,
            "avatar_url": updated_user.avatar_url,
            "created_at": updated_user.created_at.isoformat() if updated_user.created_at else None,
            "updated_at": updated_user.updated_at.isoformat() if updated_user.updated_at else None,
            "last_login_at": updated_user.last_login_at.isoformat() if updated_user.last_login_at else None,
            "is_active": updated_user.is_active,
            "email_verified": updated_user.email_verified
        }
    }

@router.get("/{user_id}", response_model=dict)
async def get_user_profile(
    user_id: str,
    current_user: User = Depends(auth_manager.get_current_user),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Get a specific user's profile information (for other agents to access).
    This endpoint is designed to allow other agents to access user profile information
    for personalization purposes, following MCP standards.
    """
    # In a real implementation, you might want to add additional authorization checks here
    # For now, we'll allow access to any user's profile for demonstration purposes

    target_user = await user_service.get_user_by_id(db, user_id)

    if not target_user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )

    # Return a simplified profile for other agents
    return {
        "id": str(target_user.id),
        "email": target_user.email,  # In real implementation, you might not want to expose email
        "name": target_user.name,
        "avatar_url": target_user.avatar_url,
        "is_active": target_user.is_active,
        "created_at": target_user.created_at.isoformat() if target_user.created_at else None,
        "updated_at": target_user.updated_at.isoformat() if target_user.updated_at else None
    }