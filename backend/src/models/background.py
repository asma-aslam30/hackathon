from sqlalchemy import Column, Integer, String, DateTime, Boolean, Text, ForeignKey
from sqlalchemy.sql import func
from sqlalchemy.dialects.postgresql import UUID, JSONB
from ..config.database import Base
import uuid
from sqlalchemy.orm import relationship

class BackgroundInformation(Base):
    __tablename__ = "background_information"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), unique=True, nullable=False)
    software_tools = Column(JSONB, nullable=True)  # Array of strings representing software tools
    hardware_setup = Column(JSONB, nullable=True)  # Object describing hardware configuration
    programming_languages = Column(JSONB, nullable=True)  # Array of strings representing programming languages
    technical_preferences = Column(JSONB, nullable=True)  # Object with technical preferences
    experience_level = Column(String, nullable=True)  # Enum: beginner, intermediate, advanced, expert
    primary_domain = Column(String, nullable=True)  # e.g., web, mobile, data, ai
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Relationship
    user = relationship("User", back_populates="background_info")

    def __repr__(self):
        return f"<BackgroundInformation(user_id={self.user_id}, experience_level='{self.experience_level}')>"

