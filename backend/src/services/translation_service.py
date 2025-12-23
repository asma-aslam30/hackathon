"""
Translation Service for Urdu content translation.

This module provides the core translation functionality using Gemini API
with caching support for improved performance.
"""
import asyncio
import hashlib
import time
import logging
from datetime import datetime, timezone, timedelta
from typing import Optional, Tuple
from uuid import UUID

from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, delete

from ..config.settings import get_settings
from ..models.translation import TranslationCache
from ..models.translation_schemas import (
    TranslationRequest,
    TranslationResponse,
    TranslationMetadata,
    TranslationError
)
from ..utils.rtl_helper import add_rtl_attributes, wrap_content_with_rtl
from ..utils.html_processor import HTMLProcessor, count_words

logger = logging.getLogger(__name__)
settings = get_settings()


# Urdu translation prompt template
URDU_TRANSLATION_PROMPT = """
You are a professional translator specializing in English to Urdu translation.
Translate the following content to Urdu while:

1. Preserving HTML structure and formatting exactly
2. Maintaining technical terminology accuracy (transliterate if no Urdu equivalent exists)
3. Using proper Nastaliq script
4. Adapting idioms to equivalent Urdu expressions that convey the same meaning
5. Keeping code blocks, inline code, and programming syntax UNCHANGED (do not translate anything inside <pre>, <code> tags)
6. Preserving all HTML attributes, class names, and IDs unchanged
7. Maintaining the same paragraph structure and line breaks
8. Using formal Urdu suitable for technical documentation

IMPORTANT: Return ONLY the translated content without any explanations or additional text.
Do not add any prefixes like "Here is the translation:" or similar.

Content to translate:
{content}
"""


class TranslationService:
    """
    Service for translating content to Urdu using Gemini API.

    Provides translation with caching to reduce API calls and improve performance.
    """

    def __init__(self, db: AsyncSession):
        """
        Initialize the translation service.

        Args:
            db: Async database session
        """
        self.db = db
        self.cache_ttl_hours = settings.translation_cache_ttl_hours
        self.max_content_length = settings.translation_max_content_length

    async def translate_to_urdu(
        self,
        request: TranslationRequest,
        user_id: Optional[UUID] = None
    ) -> TranslationResponse:
        """
        Translate content to Urdu.

        Args:
            request: Translation request with content and options
            user_id: Optional user ID for tracking

        Returns:
            TranslationResponse with translated content and metadata
        """
        start_time = time.time()

        # Validate content length
        if len(request.content) > self.max_content_length:
            raise ValueError(f"Content exceeds maximum length of {self.max_content_length} characters")

        # Generate cache key
        cache_key = self._generate_cache_key(
            request.content,
            request.source_language,
            "ur"
        )

        # Check cache
        cached_result = await self._get_from_cache(cache_key)
        if cached_result:
            logger.info(f"Cache hit for translation (hash: {cache_key[:8]}...)")
            return TranslationResponse(
                success=True,
                translated_content=cached_result.translated_content,
                metadata=TranslationMetadata(
                    word_count=cached_result.word_count,
                    translation_time_ms=int((time.time() - start_time) * 1000),
                    source_language=cached_result.source_language,
                    target_language=cached_result.target_language,
                    cached_at=cached_result.created_at
                ),
                cache_hit=True
            )

        # Perform translation
        translated_content = await self._perform_translation(request.content)

        # Add RTL attributes
        if request.preserve_formatting:
            translated_content = add_rtl_attributes(translated_content, "ur")
        else:
            translated_content = wrap_content_with_rtl(translated_content, "ur")

        # Calculate metrics
        translation_time_ms = int((time.time() - start_time) * 1000)
        word_count = count_words(request.content)

        # Store in cache
        await self._store_in_cache(
            cache_key=cache_key,
            original_content=request.content,
            translated_content=translated_content,
            source_language=request.source_language,
            target_language="ur",
            word_count=word_count,
            translation_time_ms=translation_time_ms,
            user_id=user_id
        )

        logger.info(f"Translation completed in {translation_time_ms}ms for {word_count} words")

        return TranslationResponse(
            success=True,
            translated_content=translated_content,
            metadata=TranslationMetadata(
                word_count=word_count,
                translation_time_ms=translation_time_ms,
                source_language=request.source_language,
                target_language="ur",
                cached_at=None
            ),
            cache_hit=False
        )

    async def _perform_translation(self, content: str) -> str:
        """
        Perform the actual translation using Gemini API.

        Args:
            content: Content to translate

        Returns:
            Translated content
        """
        max_retries = 3
        retry_delay = 2  # seconds

        for attempt in range(max_retries):
            try:
                import google.generativeai as genai

                # Configure Gemini
                genai.configure(api_key=settings.gemini_api_key)

                # Configure model with timeout settings
                generation_config = {
                    "temperature": 0.3,
                    "top_p": 0.95,
                    "top_k": 40,
                    "max_output_tokens": 8192,
                }

                safety_settings = [
                    {"category": "HARM_CATEGORY_HARASSMENT", "threshold": "BLOCK_NONE"},
                    {"category": "HARM_CATEGORY_HATE_SPEECH", "threshold": "BLOCK_NONE"},
                    {"category": "HARM_CATEGORY_SEXUALLY_EXPLICIT", "threshold": "BLOCK_NONE"},
                    {"category": "HARM_CATEGORY_DANGEROUS_CONTENT", "threshold": "BLOCK_NONE"},
                ]

                model = genai.GenerativeModel(
                    settings.gemini_model,
                    generation_config=generation_config,
                    safety_settings=safety_settings
                )

                # Prepare prompt
                prompt = URDU_TRANSLATION_PROMPT.format(content=content)

                # Generate translation with request options
                response = model.generate_content(
                    prompt,
                    request_options={"timeout": 90}  # 90 second timeout
                )

                if response.text:
                    return response.text.strip()
                else:
                    raise ValueError("Empty response from translation API")

            except ImportError:
                logger.error("google-generativeai package not installed")
                raise ImportError("google-generativeai package is required for translation")
            except Exception as e:
                error_msg = str(e)
                logger.warning(f"Translation attempt {attempt + 1}/{max_retries} failed: {error_msg}")

                if attempt < max_retries - 1:
                    # Wait before retrying
                    await asyncio.sleep(retry_delay)
                    retry_delay *= 2  # Exponential backoff
                    continue
                else:
                    logger.error(f"Translation API error after {max_retries} attempts: {error_msg}")
                    raise RuntimeError(f"Translation failed: {error_msg}")

    def _generate_cache_key(
        self,
        content: str,
        source_language: str,
        target_language: str
    ) -> str:
        """
        Generate a cache key for the translation.

        Args:
            content: Content to translate
            source_language: Source language code
            target_language: Target language code

        Returns:
            SHA-256 hash as cache key
        """
        key_string = f"{source_language}:{target_language}:{content}"
        return hashlib.sha256(key_string.encode()).hexdigest()

    async def _get_from_cache(self, cache_key: str) -> Optional[TranslationCache]:
        """
        Get translation from cache.

        Args:
            cache_key: The cache key to look up

        Returns:
            Cached translation if found and not expired, None otherwise
        """
        try:
            result = await self.db.execute(
                select(TranslationCache)
                .where(TranslationCache.content_hash == cache_key)
                .where(TranslationCache.expires_at > datetime.now(timezone.utc))
            )
            cache_entry = result.scalar_one_or_none()

            if cache_entry:
                # Increment hit count
                cache_entry.hit_count += 1
                await self.db.commit()
                return cache_entry

            return None
        except Exception as e:
            logger.warning(f"Cache lookup failed: {str(e)}")
            return None

    async def _store_in_cache(
        self,
        cache_key: str,
        original_content: str,
        translated_content: str,
        source_language: str,
        target_language: str,
        word_count: int,
        translation_time_ms: int,
        user_id: Optional[UUID] = None
    ) -> None:
        """
        Store translation in cache.

        Args:
            cache_key: Cache key
            original_content: Original content
            translated_content: Translated content
            source_language: Source language
            target_language: Target language
            word_count: Word count
            translation_time_ms: Translation time in ms
            user_id: Optional user ID
        """
        try:
            cache_entry = TranslationCache(
                content_hash=cache_key,
                original_content=original_content,
                translated_content=translated_content,
                source_language=source_language,
                target_language=target_language,
                word_count=word_count,
                translation_time_ms=translation_time_ms,
                expires_at=TranslationCache.create_expiration_time(self.cache_ttl_hours),
                created_by=user_id,
                hit_count=0
            )
            self.db.add(cache_entry)
            await self.db.commit()
            logger.debug(f"Stored translation in cache (hash: {cache_key[:8]}...)")
        except Exception as e:
            logger.warning(f"Failed to store in cache: {str(e)}")
            await self.db.rollback()

    async def clear_user_cache(self, user_id: UUID) -> int:
        """
        Clear all cached translations for a user.

        Args:
            user_id: The user ID

        Returns:
            Number of cache entries cleared
        """
        try:
            result = await self.db.execute(
                delete(TranslationCache)
                .where(TranslationCache.created_by == user_id)
            )
            await self.db.commit()
            return result.rowcount
        except Exception as e:
            logger.error(f"Failed to clear user cache: {str(e)}")
            await self.db.rollback()
            return 0

    async def get_service_status(self) -> dict:
        """
        Get the current status of the translation service.

        Returns:
            Dictionary with service status information
        """
        try:
            # Count cache entries
            result = await self.db.execute(
                select(TranslationCache)
                .where(TranslationCache.expires_at > datetime.now(timezone.utc))
            )
            cache_entries = len(result.scalars().all())

            return {
                "status": "healthy",
                "cache_entries": cache_entries,
                "avg_translation_time_ms": 1200,  # Placeholder
                "supported_languages": [
                    {"code": "ur", "name": "Urdu", "direction": "rtl"}
                ]
            }
        except Exception as e:
            logger.error(f"Failed to get service status: {str(e)}")
            return {
                "status": "degraded",
                "cache_entries": 0,
                "avg_translation_time_ms": 0,
                "supported_languages": [
                    {"code": "ur", "name": "Urdu", "direction": "rtl"}
                ]
            }
