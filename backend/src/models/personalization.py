from sqlalchemy import Column, Integer, String, DateTime, Boolean, Text, ForeignKey
from sqlalchemy.sql import func
from sqlalchemy.dialects.postgresql import UUID, JSONB
from ..config.database import Base
import uuid
from sqlalchemy.orm import relationship

class PersonalizationProfile(Base):
    __tablename__ = "personalization_profiles"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), unique=True, nullable=False)
    recommended_content = Column(JSONB, nullable=True)  # Array of content recommendations
    preferred_categories = Column(JSONB, nullable=True)  # Array of category preferences
    last_personalization_update = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Relationship
    user = relationship("User", back_populates="personalization_profile")

    def __repr__(self):
        return f"<PersonalizationProfile(user_id={self.user_id})>"

