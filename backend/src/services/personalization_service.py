"""
Personalization service for handling content personalization logic
"""

from typing import Dict, Any, Optional
from ..utils.content_transformer import content_transformer, UserProfile
from ..services.background_service import get_user_background_profile
import json
import time


class PersonalizationService:
    """
    Service for handling content personalization based on user background
    """

    def __init__(self):
        self.cache = {}  # Simple in-memory cache
        self.cache_ttl = 300  # 5 minutes TTL

    async def get_personalized_content(self, chapter_id: str, user_id: str) -> Dict[str, Any]:
        """
        Get personalized content for a chapter based on user's background
        """
        cache_key = f"{user_id}:{chapter_id}"

        # Check if we have cached content
        if cache_key in self.cache:
            cached_data, timestamp = self.cache[cache_key]
            if time.time() - timestamp < self.cache_ttl:
                return cached_data

        # Fetch user profile
        user_profile_data = await get_user_background_profile(user_id)

        # Create user profile object
        user_profile = UserProfile(
            user_id=user_id,
            programming_languages=user_profile_data.get("programming_languages", []),
            experience_level=user_profile_data.get("experience_level", "intermediate"),
            domain_expertise=user_profile_data.get("domain_expertise", []),
            tools_familiarity=user_profile_data.get("tools_familiarity", []),
            os_familiarity=user_profile_data.get("os_familiarity", [])
        )

        # For now, we'll use a placeholder for the original chapter content
        # In a real implementation, this would come from a content management system
        original_content = self._get_chapter_content(chapter_id)

        # Transform the content based on user profile
        result = content_transformer.personalize_content(original_content, user_profile)

        # Add to cache
        self.cache[cache_key] = (result, time.time())

        return result

    def _get_chapter_content(self, chapter_id: str) -> str:
        """
        Get original chapter content by ID
        In a real implementation, this would fetch from a content management system
        """
        # Placeholder implementation - in reality this would fetch from database or CMS
        placeholder_content = f"""
        <h1>Chapter {chapter_id} - Example Content</h1>
        <p>This is an example chapter that will be personalized based on your background.</p>
        <p>For example, if you're a web developer, we might show JavaScript examples.</p>
        <p>If you're a data scientist, we might show Python examples with pandas.</p>
        <pre><code>
        function exampleFunction() {{
            console.log("This is a generic code example");
            let result = "transform this based on user background";
            return result;
        }}
        </code></pre>
        <p>The content complexity and examples will be adjusted based on your experience level.</p>
        """
        return placeholder_content

    async def update_personalization_preferences(self, user_id: str, preferences: Dict[str, Any]) -> Dict[str, Any]:
        """
        Update user's personalization preferences
        """
        # In a real implementation, this would save to a database
        # For now, we'll just return the preferences
        return {
            "user_id": user_id,
            "preferences": preferences,
            "updated_at": time.time()
        }

    async def get_personalization_attribution(self, chapter_id: str, user_id: str) -> Dict[str, Any]:
        """
        Get information about which aspects of user profile influenced personalization
        """
        user_profile_data = await get_user_background_profile(user_id)

        # Create attribution information
        attribution = {
            "chapter_id": chapter_id,
            "personalization_factors": [
                {
                    "factor": "experience_level",
                    "weight": 0.3,
                    "description": f"Content complexity adjusted for {user_profile_data.get('experience_level', 'intermediate')} level"
                },
                {
                    "factor": "programming_languages",
                    "weight": 0.5,
                    "description": f"Examples customized for {', '.join(user_profile_data.get('programming_languages', []))}"
                },
                {
                    "factor": "domain_expertise",
                    "weight": 0.2,
                    "description": f"Context tailored for {', '.join(user_profile_data.get('domain_expertise', []))}"
                }
            ],
            "profile_elements_used": [
                "experience_level",
                "programming_languages",
                "domain_expertise"
            ]
        }

        return attribution

    async def submit_personalization_feedback(self, user_id: str, chapter_id: str, feedback_type: str, feedback_text: str = "") -> Dict[str, Any]:
        """
        Submit feedback on personalization quality
        """
        # In a real implementation, this would save to a database for analysis
        feedback_record = {
            "user_id": user_id,
            "chapter_id": chapter_id,
            "feedback_type": feedback_type,
            "feedback_text": feedback_text,
            "timestamp": time.time()
        }

        # For now, just return confirmation
        return {
            "message": "Feedback submitted successfully",
            "feedback_record": feedback_record
        }

    async def clear_cache_for_user(self, user_id: str):
        """
        Clear cached personalization for a user (e.g., when profile is updated)
        """
        keys_to_remove = [key for key in self.cache.keys() if key.startswith(user_id + ":")]
        for key in keys_to_remove:
            del self.cache[key]


# Global instance for use in API
personalization_service = PersonalizationService()