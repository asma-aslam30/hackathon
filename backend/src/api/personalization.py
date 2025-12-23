"""
Personalization API endpoints
"""

from fastapi import APIRouter, HTTPException, Depends, Request
from typing import Dict, Any
from ..services.personalization_service import personalization_service
from ..services.background_service import get_user_background_profile
from ..auth.middleware import get_current_user
import uuid

router = APIRouter()

@router.get("/chapter/{chapter_id}")
async def get_personalized_chapter(
    chapter_id: str,
    request: Request,
    current_user: Dict = Depends(get_current_user)
):
    """
    Get personalized chapter content based on user's background
    """
    try:
        # Get the user ID from the authenticated user
        user_id = str(current_user.get("id", current_user.get("user_id", "unknown")))

        # Get personalized content
        result = await personalization_service.get_personalized_content(chapter_id, user_id)

        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting personalized content: {str(e)}")


@router.get("/preferences")
async def get_personalization_preferences(
    current_user: Dict = Depends(get_current_user)
):
    """
    Get user's personalization preferences
    """
    try:
        user_id = str(current_user.get("id", current_user.get("user_id", "unknown")))

        # In a real implementation, this would fetch from a database
        # For now, return default preferences
        preferences = {
            "id": str(uuid.uuid4()),
            "user_id": user_id,
            "intensity": "medium",
            "customization_types": {
                "examples": True,
                "complexity": True,
                "domain_focus": True
            }
        }

        return preferences
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting preferences: {str(e)}")


@router.put("/preferences")
async def update_personalization_preferences(
    request: Request,
    current_user: Dict = Depends(get_current_user)
):
    """
    Update user's personalization preferences
    """
    try:
        user_id = str(current_user.get("id", current_user.get("user_id", "unknown")))
        preferences_data = await request.json()

        result = await personalization_service.update_personalization_preferences(
            user_id, preferences_data
        )

        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating preferences: {str(e)}")


@router.post("/toggle")
async def toggle_personalization(
    request: Request,
    current_user: Dict = Depends(get_current_user)
):
    """
    Toggle personalization for a specific chapter
    """
    try:
        body = await request.json()
        chapter_id = body.get("chapter_id")
        enabled = body.get("enabled")

        if not chapter_id:
            raise HTTPException(status_code=400, detail="chapter_id is required")

        # In a real implementation, this would save the toggle preference
        # For now, just return the status
        return {
            "chapter_id": chapter_id,
            "personalization_enabled": enabled
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error toggling personalization: {str(e)}")


@router.get("/attribution/{chapter_id}")
async def get_personalization_attribution(
    chapter_id: str,
    current_user: Dict = Depends(get_current_user)
):
    """
    Get information about which aspects of user profile influenced personalization
    """
    try:
        user_id = str(current_user.get("id", current_user.get("user_id", "unknown")))

        result = await personalization_service.get_personalization_attribution(chapter_id, user_id)

        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting attribution: {str(e)}")


@router.post("/feedback")
async def submit_personalization_feedback(
    request: Request,
    current_user: Dict = Depends(get_current_user)
):
    """
    Submit feedback on personalization quality
    """
    try:
        user_id = str(current_user.get("id", current_user.get("user_id", "unknown")))
        feedback_data = await request.json()

        chapter_id = feedback_data.get("chapter_id")
        feedback_type = feedback_data.get("feedback_type")
        feedback_text = feedback_data.get("feedback_text", "")

        if not chapter_id:
            raise HTTPException(status_code=400, detail="chapter_id is required")
        if feedback_type not in ["positive", "negative", "neutral"]:
            raise HTTPException(status_code=400, detail="feedback_type must be positive, negative, or neutral")

        result = await personalization_service.submit_personalization_feedback(
            user_id, chapter_id, feedback_type, feedback_text
        )

        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error submitting feedback: {str(e)}")


