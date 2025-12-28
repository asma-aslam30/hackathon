/**
 * Translation Service for Urdu Translation
 *
 * This module provides client-side API calls for translating content to Urdu.
 */
import API_CONFIG from '../config/api';

const API_BASE_URL = `${API_CONFIG.BACKEND_URL}/api/v1`;

/**
 * Get the authentication token from localStorage
 * @returns {string|null} The auth token or null
 */
const getAuthToken = () => {
  if (typeof window !== 'undefined') {
    return localStorage.getItem('session_token') || localStorage.getItem('authToken');
  }
  return null;
};

/**
 * Check if user is authenticated
 * @returns {boolean} Whether user has a valid auth token
 */
export const isAuthenticated = () => {
  return !!getAuthToken();
};

/**
 * Translate content to Urdu
 *
 * @param {Object} params - Translation parameters
 * @param {string} params.content - HTML content to translate
 * @param {string} [params.chapterId] - Optional chapter identifier
 * @param {string} [params.sourceLanguage='en'] - Source language code
 * @param {boolean} [params.preserveFormatting=true] - Whether to preserve HTML formatting
 * @returns {Promise<Object>} Translation response with translated content and metadata
 * @throws {Error} If translation fails or user is not authenticated
 */
export const translateToUrdu = async ({
  content,
  chapterId = null,
  sourceLanguage = 'en',
  preserveFormatting = true
}) => {
  const token = getAuthToken();

  if (!token) {
    throw new Error('Authentication required. Please log in to use translation.');
  }

  try {
    const response = await fetch(`${API_BASE_URL}/translation/translate-to-urdu`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${token}`
      },
      body: JSON.stringify({
        content,
        chapter_id: chapterId,
        source_language: sourceLanguage,
        preserve_formatting: preserveFormatting
      })
    });

    if (!response.ok) {
      const errorData = await response.json();
      const errorDetail = errorData.detail || errorData;

      // Handle specific error codes
      if (response.status === 401) {
        throw new Error('Please log in to use translation.');
      }

      if (response.status === 429) {
        const retryAfter = errorDetail.retry_after || 30;
        throw new Error(`Translation service busy. Please try again in ${retryAfter} seconds.`);
      }

      throw new Error(errorDetail.error_message || 'Translation failed. Please try again.');
    }

    return await response.json();
  } catch (error) {
    if (error.name === 'TypeError' && error.message.includes('fetch')) {
      throw new Error('Connection error. Check your internet and try again.');
    }
    throw error;
  }
};

/**
 * Get translation service status
 *
 * @returns {Promise<Object>} Service status with health info and statistics
 */
export const getTranslationStatus = async () => {
  try {
    const response = await fetch(`${API_BASE_URL}/translation/status`);

    if (!response.ok) {
      throw new Error('Failed to get service status');
    }

    return await response.json();
  } catch (error) {
    console.error('Failed to get translation status:', error);
    return {
      status: 'unknown',
      cache_entries: 0,
      avg_translation_time_ms: 0,
      supported_languages: [{ code: 'ur', name: 'Urdu', direction: 'rtl' }]
    };
  }
};

/**
 * Clear the user's translation cache
 *
 * @returns {Promise<Object>} Result with success status and cleared count
 */
export const clearTranslationCache = async () => {
  const token = getAuthToken();

  if (!token) {
    throw new Error('Authentication required');
  }

  try {
    const response = await fetch(`${API_BASE_URL}/translation/cache/clear`, {
      method: 'DELETE',
      headers: {
        'Authorization': `Bearer ${token}`
      }
    });

    if (!response.ok) {
      throw new Error('Failed to clear cache');
    }

    return await response.json();
  } catch (error) {
    console.error('Failed to clear translation cache:', error);
    throw error;
  }
};

export default {
  translateToUrdu,
  getTranslationStatus,
  clearTranslationCache,
  isAuthenticated
};
