"""
Authentication service using JWT tokens (similar to Better-Auth functionality).
This module handles user registration, login, and token management.
"""
from datetime import datetime, timedelta
from typing import Optional
import jwt
from fastapi import HTTPException, status, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from passlib.context import CryptContext
from pydantic import BaseModel
from ..config.settings import settings
from ..models.user import User
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from ..config.database import get_async_db
from datetime import timezone

# Security settings
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
security = HTTPBearer()

class TokenData(BaseModel):
    email: Optional[str] = None

class AuthManager:
    def __init__(self):
        self.secret_key = settings.secret_key
        self.algorithm = settings.algorithm
        self.access_token_expire_minutes = settings.access_token_expire_minutes

    def verify_password(self, plain_password: str, hashed_password: str) -> bool:
        """Verify a plain password against a hashed password."""
        # Truncate to 72 bytes for bcrypt compatibility
        truncated = plain_password[:72] if len(plain_password.encode('utf-8')) > 72 else plain_password
        return pwd_context.verify(truncated, hashed_password)

    def get_password_hash(self, password: str) -> str:
        """Hash a plain password."""
        # Truncate to 72 bytes for bcrypt compatibility
        truncated = password[:72] if len(password.encode('utf-8')) > 72 else password
        return pwd_context.hash(truncated)

    def create_access_token(self, data: dict, expires_delta: Optional[timedelta] = None) -> str:
        """Create a JWT access token."""
        to_encode = data.copy()
        if expires_delta:
            expire = datetime.now(timezone.utc) + expires_delta
        else:
            expire = datetime.now(timezone.utc) + timedelta(minutes=self.access_token_expire_minutes)

        to_encode.update({"exp": expire})
        encoded_jwt = jwt.encode(to_encode, self.secret_key, algorithm=self.algorithm)
        return encoded_jwt

    async def authenticate_user(self, db: AsyncSession, email: str, password: str) -> Optional[User]:
        """Authenticate a user by email and password."""
        result = await db.execute(select(User).filter(User.email == email))
        user = result.scalar_one_or_none()

        if not user or not self.verify_password(password, user.hashed_password):
            return None

        return user

    async def get_current_user(self, token: str = Depends(security), db: AsyncSession = Depends(get_async_db)) -> User:
        """Get the current user based on the JWT token."""
        credentials_exception = HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

        try:
            payload = jwt.decode(token.credentials, self.secret_key, algorithms=[self.algorithm])
            email: str = payload.get("sub")
            if email is None:
                raise credentials_exception
            token_data = TokenData(email=email)
        except jwt.PyJWTError:
            raise credentials_exception

        result = await db.execute(select(User).filter(User.email == token_data.email))
        user = result.scalar_one_or_none()

        if user is None:
            raise credentials_exception

        return user

# Create a global instance of AuthManager
auth_manager = AuthManager()