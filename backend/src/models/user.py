from sqlalchemy import Column, Integer, String, DateTime, Boolean
from sqlalchemy.sql import func
from ..config.database import Base
from datetime import datetime
import uuid
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship

class User(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String, unique=True, index=True, nullable=False)
    name = Column(String, nullable=True)
    avatar_url = Column(String, nullable=True)
    hashed_password = Column(String, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
    last_login_at = Column(DateTime(timezone=True), nullable=True)
    is_active = Column(Boolean, default=True)
    email_verified = Column(Boolean, default=False)
    preferred_language = Column(String(10), nullable=True, default=None)

    # Relationships
    background_info = relationship("BackgroundInformation", back_populates="user", uselist=False)
    sessions = relationship("Session", back_populates="user")
    personalization_profile = relationship("PersonalizationProfile", back_populates="user", uselist=False)
    translation_cache_entries = relationship("TranslationCache", back_populates="user", lazy="dynamic")

    def __repr__(self):
        return f"<User(id={self.id}, email='{self.email}', name='{self.name}')>"