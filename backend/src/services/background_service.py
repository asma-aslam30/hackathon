"""
Background information service for handling user background data.
"""
from typing import Optional
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from fastapi import HTTPException, status
from datetime import datetime, timezone
from ..models.background import BackgroundInformation
from ..models.user import User
from pydantic import BaseModel
import json


class BackgroundCreateUpdate(BaseModel):
    software_tools: Optional[list] = None
    hardware_setup: Optional[dict] = None
    programming_languages: Optional[list] = None
    technical_preferences: Optional[dict] = None
    experience_level: Optional[str] = None  # beginner, intermediate, advanced, expert
    primary_domain: Optional[str] = None  # web, mobile, data, ai, etc.


class BackgroundService:
    def __init__(self):
        pass

    async def create_or_update_background(self, db: AsyncSession, user_id: str, background_data: BackgroundCreateUpdate) -> BackgroundInformation:
        """
        Create or update background information for a user.
        """
        # Check if background info already exists for this user
        result = await db.execute(
            select(BackgroundInformation).filter(BackgroundInformation.user_id == user_id)
        )
        existing_bg = result.scalar_one_or_none()

        if existing_bg:
            # Update existing background
            if background_data.software_tools is not None:
                existing_bg.software_tools = background_data.software_tools
            if background_data.hardware_setup is not None:
                existing_bg.hardware_setup = background_data.hardware_setup
            if background_data.programming_languages is not None:
                existing_bg.programming_languages = background_data.programming_languages
            if background_data.technical_preferences is not None:
                existing_bg.technical_preferences = background_data.technical_preferences
            if background_data.experience_level is not None:
                existing_bg.experience_level = background_data.experience_level
            if background_data.primary_domain is not None:
                existing_bg.primary_domain = background_data.primary_domain

            existing_bg.updated_at = datetime.now(timezone.utc)
            await db.commit()
            await db.refresh(existing_bg)
            return existing_bg
        else:
            # Create new background information
            db_background = BackgroundInformation(
                user_id=user_id,
                software_tools=background_data.software_tools,
                hardware_setup=background_data.hardware_setup,
                programming_languages=background_data.programming_languages,
                technical_preferences=background_data.technical_preferences,
                experience_level=background_data.experience_level,
                primary_domain=background_data.primary_domain
            )

            db.add(db_background)
            await db.commit()
            await db.refresh(db_background)
            return db_background

    async def get_background_by_user_id(self, db: AsyncSession, user_id: str) -> Optional[BackgroundInformation]:
        """
        Retrieve background information for a user.
        """
        result = await db.execute(
            select(BackgroundInformation).filter(BackgroundInformation.user_id == user_id)
        )
        return result.scalar_one_or_none()

    async def get_background_by_id(self, db: AsyncSession, bg_id: str) -> Optional[BackgroundInformation]:
        """
        Retrieve background information by its ID.
        """
        result = await db.execute(
            select(BackgroundInformation).filter(BackgroundInformation.id == bg_id)
        )
        return result.scalar_one_or_none()

    async def delete_background(self, db: AsyncSession, user_id: str) -> bool:
        """
        Delete background information for a user.
        """
        result = await db.execute(
            select(BackgroundInformation).filter(BackgroundInformation.user_id == user_id)
        )
        bg_info = result.scalar_one_or_none()

        if not bg_info:
            return False

        await db.delete(bg_info)
        await db.commit()
        return True


# Global instance
background_service = BackgroundService()


async def get_user_background_profile(db: AsyncSession, user_id: str) -> Optional[dict]:
    """
    Get user background profile as a dictionary for personalization.

    This is a convenience function used by personalization_service.
    """
    background = await background_service.get_background_by_user_id(db, user_id)
    if not background:
        return None

    return {
        "experience_level": background.experience_level,
        "primary_domain": background.primary_domain,
        "programming_languages": background.programming_languages or [],
        "software_tools": background.software_tools or [],
        "hardware_setup": background.hardware_setup or {},
        "technical_preferences": background.technical_preferences or {}
    }