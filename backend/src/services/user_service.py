"""
User service for handling user-related operations.
"""
from typing import Optional, List
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy.exc import IntegrityError
from fastapi import HTTPException, status
from datetime import datetime, timezone
from ..models.user import User
from ..auth.better_auth import auth_manager
from pydantic import BaseModel


class UserCreate(BaseModel):
    email: str
    password: str
    name: Optional[str] = None


class UserUpdate(BaseModel):
    name: Optional[str] = None
    avatar_url: Optional[str] = None


class UserService:
    def __init__(self):
        pass

    async def create_user(self, db: AsyncSession, user_data: UserCreate) -> User:
        """
        Create a new user with hashed password.
        """
        # Check if user already exists
        existing_user = await self.get_user_by_email(db, user_data.email)
        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="A user with this email already exists"
            )

        # Hash the password
        hashed_password = auth_manager.get_password_hash(user_data.password)

        # Create new user
        db_user = User(
            email=user_data.email,
            hashed_password=hashed_password,
            name=user_data.name
        )

        db.add(db_user)
        await db.commit()
        await db.refresh(db_user)

        return db_user

    async def get_user_by_id(self, db: AsyncSession, user_id: str) -> Optional[User]:
        """
        Retrieve a user by their ID.
        """
        result = await db.execute(select(User).filter(User.id == user_id))
        return result.scalar_one_or_none()

    async def get_user_by_email(self, db: AsyncSession, email: str) -> Optional[User]:
        """
        Retrieve a user by their email.
        """
        result = await db.execute(select(User).filter(User.email == email))
        return result.scalar_one_or_none()

    async def update_user(self, db: AsyncSession, user_id: str, user_update: UserUpdate) -> Optional[User]:
        """
        Update user information.
        """
        result = await db.execute(select(User).filter(User.id == user_id))
        db_user = result.scalar_one_or_none()

        if not db_user:
            return None

        # Update fields if provided
        if user_update.name is not None:
            db_user.name = user_update.name
        if user_update.avatar_url is not None:
            db_user.avatar_url = user_update.avatar_url

        db_user.updated_at = datetime.now(timezone.utc)

        await db.commit()
        await db.refresh(db_user)

        return db_user

    async def update_last_login(self, db: AsyncSession, user_id: str) -> Optional[User]:
        """
        Update the last login time for a user.
        """
        result = await db.execute(select(User).filter(User.id == user_id))
        db_user = result.scalar_one_or_none()

        if not db_user:
            return None

        db_user.last_login_at = datetime.now(timezone.utc)
        await db.commit()
        await db.refresh(db_user)

        return db_user

    async def delete_user(self, db: AsyncSession, user_id: str) -> bool:
        """
        Mark a user as inactive (soft delete).
        """
        result = await db.execute(select(User).filter(User.id == user_id))
        db_user = result.scalar_one_or_none()

        if not db_user:
            return False

        db_user.is_active = False
        await db.commit()

        return True

    async def search_users(self, db: AsyncSession, query: str) -> List[User]:
        """
        Search for users by email or name.
        """
        result = await db.execute(
            select(User)
            .filter(
                (User.email.contains(query)) |
                (User.name.contains(query))
            )
            .filter(User.is_active == True)
        )
        return result.scalars().all()


# Global instance
user_service = UserService()