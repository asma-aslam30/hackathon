"""
Authentication API endpoints.
"""
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import HTTPBearer
from sqlalchemy.ext.asyncio import AsyncSession
from ..config.database import get_async_db
from ..auth.better_auth import auth_manager
from ..services.user_service import user_service, UserCreate
from pydantic import BaseModel
from typing import Optional
import uuid

router = APIRouter()
security = HTTPBearer()

class UserLogin(BaseModel):
    email: str
    password: str

class TokenResponse(BaseModel):
    success: bool
    user: dict
    session_token: str

class LoginResponse(BaseModel):
    success: bool
    user: dict
    session_token: str

class ErrorResponse(BaseModel):
    success: bool
    error: str
    details: Optional[list] = None

@router.post("/register", response_model=TokenResponse)
async def register(user_data: UserCreate, db: AsyncSession = Depends(get_async_db)):
    """
    Register a new user.
    """
    try:
        # Create the user
        db_user = await user_service.create_user(db, user_data)

        # Create access token
        access_token = auth_manager.create_access_token(data={"sub": db_user.email})

        # Return user data and token
        user_response = {
            "id": str(db_user.id),
            "email": db_user.email,
            "name": db_user.name,
            "created_at": db_user.created_at.isoformat() if db_user.created_at else None
        }

        return {
            "success": True,
            "user": user_response,
            "session_token": access_token
        }
    except HTTPException as e:
        # Re-raise HTTP exceptions
        raise e
    except Exception as e:
        # Handle other exceptions
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )

@router.post("/login", response_model=LoginResponse)
async def login(user_data: UserLogin, db: AsyncSession = Depends(get_async_db)):
    """
    Authenticate a user and return a token.
    """
    # Authenticate user
    user = await auth_manager.authenticate_user(db, user_data.email, user_data.password)

    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password"
        )

    # Update last login time
    await user_service.update_last_login(db, str(user.id))

    # Create access token
    access_token = auth_manager.create_access_token(data={"sub": user.email})

    # Return user data and token
    user_response = {
        "id": str(user.id),
        "email": user.email,
        "name": user.name
    }

    return {
        "success": True,
        "user": user_response,
        "session_token": access_token
    }