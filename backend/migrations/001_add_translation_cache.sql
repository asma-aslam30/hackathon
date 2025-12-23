-- Migration: 001_add_translation_cache.sql
-- Feature: Urdu Translation
-- Description: Create translation_cache table and add preferred_language to users
-- Date: 2025-12-20

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

-- Create indexes for efficient lookups
CREATE INDEX IF NOT EXISTS idx_translation_cache_hash ON translation_cache(content_hash);
CREATE INDEX IF NOT EXISTS idx_translation_cache_expires ON translation_cache(expires_at);
CREATE INDEX IF NOT EXISTS idx_translation_cache_user ON translation_cache(created_by);

-- Add preferred_language column to users table (if not exists)
DO $$
BEGIN
    IF NOT EXISTS (
        SELECT 1 FROM information_schema.columns
        WHERE table_name = 'users' AND column_name = 'preferred_language'
    ) THEN
        ALTER TABLE users ADD COLUMN preferred_language VARCHAR(10) DEFAULT NULL;
    END IF;
END $$;

-- Comment on table
COMMENT ON TABLE translation_cache IS 'Stores cached Urdu translations to reduce API calls';
COMMENT ON COLUMN translation_cache.content_hash IS 'SHA-256 hash of source_lang:target_lang:content for deduplication';
COMMENT ON COLUMN translation_cache.hit_count IS 'Number of times this cached translation was served';
COMMENT ON COLUMN translation_cache.expires_at IS 'Cache expiration time (24 hours from creation by default)';
