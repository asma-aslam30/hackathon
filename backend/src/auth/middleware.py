"""
Authentication middleware for handling session management and token validation.
"""
from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse
from ..models.session import Session
from ..models.user import User
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from datetime import datetime, timezone
from typing import Optional
import uuid

class AuthMiddleware:
    def __init__(self):
        pass

    async def validate_session(self, db: AsyncSession, token: str) -> Optional[User]:
        """
        Validate a session token and return the associated user if valid.
        """
        # Query the session table for the token
        result = await db.execute(
            select(Session)
            .join(User)
            .filter(Session.session_token == token)
            .filter(Session.expires_at > datetime.now(timezone.utc))
        )

        session = result.scalar_one_or_none()

        if session and session.user.is_active:
            # Update last accessed time
            session.last_accessed_at = datetime.now(timezone.utc)
            await db.commit()

            return session.user

        return None

    async def process_request(self, request: Request, db: AsyncSession) -> Optional[User]:
        """
        Process the request and return the authenticated user if available.
        """
        # Check for authorization header
        auth_header = request.headers.get("authorization")

        if not auth_header or not auth_header.startswith("Bearer "):
            return None

        token = auth_header.split(" ")[1]

        user = await self.validate_session(db, token)
        return user

# Global instance
auth_middleware = AuthMiddleware()


# Re-export get_current_user for convenience
from .better_auth import auth_manager
from fastapi import Depends

async def get_current_user(credentials = Depends(auth_manager.get_current_user)):
    """
    Dependency wrapper for getting the current authenticated user.
    Re-exports from better_auth for convenience.
    """
    return credentials