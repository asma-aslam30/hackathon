"""
TranslationCache model for storing cached Urdu translations.

This module defines the SQLAlchemy model for the translation_cache table,
which stores translated content to reduce API calls and improve performance.
"""
import uuid
from datetime import datetime, timedelta, timezone
from sqlalchemy import Column, String, Text, Integer, DateTime, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func

from ..config.database import Base


class TranslationCache(Base):
    """
    Model for caching translated content.

    Stores translations with a 24-hour TTL to avoid redundant API calls.
    Uses content hash for efficient deduplication.
    """
    __tablename__ = "translation_cache"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    content_hash = Column(String(64), nullable=False, index=True)
    source_language = Column(String(10), nullable=False, default="en")
    target_language = Column(String(10), nullable=False, default="ur")
    original_content = Column(Text, nullable=False)
    translated_content = Column(Text, nullable=False)
    word_count = Column(Integer, nullable=False)
    translation_time_ms = Column(Integer, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    expires_at = Column(DateTime(timezone=True), nullable=False)
    created_by = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="SET NULL"), nullable=True)
    hit_count = Column(Integer, nullable=False, default=0)

    # Relationship to User
    user = relationship("User", back_populates="translation_cache_entries")

    def __repr__(self) -> str:
        return f"<TranslationCache(id={self.id}, hash={self.content_hash[:8]}..., lang={self.target_language})>"

    @property
    def is_expired(self) -> bool:
        """Check if the cache entry has expired."""
        return datetime.now(timezone.utc) > self.expires_at

    def increment_hit_count(self) -> None:
        """Increment the hit count for this cache entry."""
        self.hit_count += 1

    @classmethod
    def create_expiration_time(cls, hours: int = 24) -> datetime:
        """Create an expiration time from now."""
        return datetime.now(timezone.utc) + timedelta(hours=hours)
