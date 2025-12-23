# Data Model: Urdu Translation

**Feature**: 001-urdu-translation | **Date**: 2025-12-20

## Overview

This document defines the data entities, relationships, and validation rules for the Urdu translation feature.

---

## Entity Relationship Diagram

```
┌─────────────────────┐         ┌──────────────────────────┐
│       User          │         │    TranslationCache      │
├─────────────────────┤         ├──────────────────────────┤
│ id: UUID (PK)       │────┐    │ id: UUID (PK)            │
│ email: String       │    │    │ content_hash: String(64) │
│ preferred_language: │    │    │ source_language: String  │
│   String (NEW)      │    │    │ target_language: String  │
│ ...existing fields  │    │    │ original_content: Text   │
└─────────────────────┘    │    │ translated_content: Text │
                           │    │ word_count: Integer      │
                           │    │ translation_time_ms: Int │
                           │    │ created_at: DateTime     │
                           │    │ expires_at: DateTime     │
                           │    │ created_by: UUID (FK)────┘
                           │    │ hit_count: Integer       │
                           │    └──────────────────────────┘
                           │
                           │    ┌──────────────────────────┐
                           │    │   TranslationRequest     │
                           │    │     (API Schema)         │
                           └───>├──────────────────────────┤
                                │ content: String          │
                                │ chapter_id: String       │
                                │ source_language: String  │
                                │ preserve_formatting: Bool│
                                └──────────────────────────┘
```

---

## Entities

### 1. TranslationCache (New Table)

Stores cached translations to avoid redundant API calls.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, NOT NULL | Primary key |
| `content_hash` | VARCHAR(64) | NOT NULL, INDEX | SHA-256 hash of source+lang+content |
| `source_language` | VARCHAR(10) | NOT NULL, DEFAULT 'en' | ISO 639-1 code |
| `target_language` | VARCHAR(10) | NOT NULL, DEFAULT 'ur' | ISO 639-1 code (Urdu) |
| `original_content` | TEXT | NOT NULL | Original HTML content |
| `translated_content` | TEXT | NOT NULL | Translated Urdu HTML content |
| `word_count` | INTEGER | NOT NULL | Word count of original content |
| `translation_time_ms` | INTEGER | NOT NULL | Time taken for translation |
| `created_at` | TIMESTAMPTZ | NOT NULL, DEFAULT NOW() | Cache entry creation time |
| `expires_at` | TIMESTAMPTZ | NOT NULL | Cache expiration (created_at + 24h) |
| `created_by` | UUID | FK → users.id, NULL OK | User who requested translation |
| `hit_count` | INTEGER | NOT NULL, DEFAULT 0 | Number of cache hits |

**Indexes**:
- `idx_translation_cache_hash` on `content_hash` (primary lookup)
- `idx_translation_cache_expires` on `expires_at` (cleanup queries)

**Validation Rules**:
- `content_hash` must be exactly 64 characters (SHA-256)
- `source_language` and `target_language` must be valid ISO 639-1 codes
- `expires_at` must be > `created_at`
- `word_count` must be > 0

```python
# SQLAlchemy Model
class TranslationCache(Base):
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
    created_by = Column(UUID(as_uuid=True), ForeignKey("users.id"), nullable=True)
    hit_count = Column(Integer, nullable=False, default=0)

    # Relationship
    user = relationship("User", back_populates="translation_cache_entries")
```

---

### 2. User (Extended)

Add `preferred_language` field to existing User model.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `preferred_language` | VARCHAR(10) | NULL OK, DEFAULT NULL | User's preferred content language |

**Migration**:
```sql
ALTER TABLE users
ADD COLUMN preferred_language VARCHAR(10) DEFAULT NULL;
```

---

## API Schemas (Pydantic)

### TranslationRequest

Request body for translation endpoint.

```python
class TranslationRequest(BaseModel):
    content: str = Field(
        ...,
        min_length=1,
        max_length=100000,
        description="HTML content to translate"
    )
    chapter_id: Optional[str] = Field(
        None,
        max_length=255,
        description="Optional chapter identifier for analytics"
    )
    source_language: str = Field(
        default="en",
        pattern="^[a-z]{2}$",
        description="ISO 639-1 source language code"
    )
    preserve_formatting: bool = Field(
        default=True,
        description="Whether to preserve HTML formatting"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "content": "<p>Hello, world!</p>",
                "chapter_id": "chapter-1-introduction",
                "source_language": "en",
                "preserve_formatting": True
            }
        }
```

### TranslationResponse

Response body for translation endpoint.

```python
class TranslationResponse(BaseModel):
    success: bool = Field(..., description="Whether translation succeeded")
    translated_content: str = Field(..., description="Urdu translated HTML")
    metadata: TranslationMetadata = Field(..., description="Translation metadata")
    cache_hit: bool = Field(..., description="Whether result was from cache")

class TranslationMetadata(BaseModel):
    word_count: int = Field(..., description="Word count of original")
    translation_time_ms: int = Field(..., description="Translation time in ms")
    source_language: str = Field(..., description="Source language code")
    target_language: str = Field(default="ur", description="Target language (Urdu)")
    cached_at: Optional[datetime] = Field(None, description="Cache creation time")
```

### TranslationError

Error response schema.

```python
class TranslationError(BaseModel):
    success: bool = Field(default=False)
    error_code: str = Field(..., description="Machine-readable error code")
    error_message: str = Field(..., description="Human-readable error message")
    retry_after: Optional[int] = Field(None, description="Seconds to wait before retry")

    class Config:
        json_schema_extra = {
            "example": {
                "success": False,
                "error_code": "RATE_LIMITED",
                "error_message": "Translation service busy. Please try again.",
                "retry_after": 30
            }
        }
```

---

## State Transitions

### Translation Session State Machine

```
┌─────────────┐
│   IDLE      │ ◄──────────────────────────────────────┐
│  (Original) │                                         │
└──────┬──────┘                                         │
       │                                                │
       │ User clicks "Translate to Urdu"                │
       ▼                                                │
┌─────────────┐                                         │
│  LOADING    │ ────► [Error] ───────────────┐          │
│             │                               │          │
└──────┬──────┘                               │          │
       │                                      ▼          │
       │ Translation complete        ┌─────────────┐    │
       ▼                             │   ERROR     │    │
┌─────────────┐                      │             │    │
│ TRANSLATED  │                      └──────┬──────┘    │
│   (Urdu)    │                             │           │
└──────┬──────┘                             │ Dismiss   │
       │                                    │           │
       │ User clicks "Show Original"        │           │
       └────────────────────────────────────┴───────────┘
```

### State Definitions

| State | Description | UI Indication |
|-------|-------------|---------------|
| IDLE | Showing original content | "Translate to Urdu" button visible |
| LOADING | Translation in progress | Spinner, button disabled |
| TRANSLATED | Showing Urdu content | "Show Original" button, RTL layout |
| ERROR | Translation failed | Error message, "Try Again" button |

---

## Relationships

### User → TranslationCache (One-to-Many)

- One user can create many cached translations
- `created_by` FK allows tracking who requested translations
- Used for analytics and rate limiting

```python
# In User model
translation_cache_entries = relationship(
    "TranslationCache",
    back_populates="user",
    lazy="dynamic"
)
```

---

## Validation Rules Summary

| Entity | Field | Rule |
|--------|-------|------|
| TranslationCache | content_hash | Exactly 64 chars (SHA-256) |
| TranslationCache | source_language | Valid ISO 639-1 code |
| TranslationCache | target_language | Valid ISO 639-1 code |
| TranslationCache | word_count | > 0 |
| TranslationCache | expires_at | > created_at |
| TranslationRequest | content | 1-100,000 chars |
| TranslationRequest | source_language | 2 lowercase letters |
| User | preferred_language | NULL or valid ISO 639-1 |

---

## Database Migration

```sql
-- Migration: 001_add_translation_cache.sql

-- Create translation cache table
CREATE TABLE IF NOT EXISTS translation_cache (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    content_hash VARCHAR(64) NOT NULL,
    source_language VARCHAR(10) NOT NULL DEFAULT 'en',
    target_language VARCHAR(10) NOT NULL DEFAULT 'ur',
    original_content TEXT NOT NULL,
    translated_content TEXT NOT NULL,
    word_count INTEGER NOT NULL,
    translation_time_ms INTEGER NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    expires_at TIMESTAMPTZ NOT NULL,
    created_by UUID REFERENCES users(id) ON DELETE SET NULL,
    hit_count INTEGER NOT NULL DEFAULT 0
);

-- Create indexes
CREATE INDEX idx_translation_cache_hash ON translation_cache(content_hash);
CREATE INDEX idx_translation_cache_expires ON translation_cache(expires_at);

-- Add preferred_language to users
ALTER TABLE users ADD COLUMN IF NOT EXISTS preferred_language VARCHAR(10) DEFAULT NULL;

-- Cleanup job (run periodically)
-- DELETE FROM translation_cache WHERE expires_at < NOW();
```
