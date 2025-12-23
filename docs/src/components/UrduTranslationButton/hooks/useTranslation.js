/**
 * useTranslation Hook
 *
 * Custom React hook for managing translation state and operations.
 */
import { useState, useCallback, useRef } from 'react';
import { translateToUrdu, isAuthenticated } from '../../../services/translationService';

// Translation states
export const TRANSLATION_STATES = {
  IDLE: 'IDLE',
  LOADING: 'LOADING',
  TRANSLATED: 'TRANSLATED',
  ERROR: 'ERROR'
};

/**
 * Custom hook for translation functionality
 *
 * @param {Object} options - Hook options
 * @param {string} options.chapterId - Optional chapter identifier
 * @returns {Object} Translation state and handlers
 */
export const useTranslation = ({ chapterId = null } = {}) => {
  const [state, setState] = useState(TRANSLATION_STATES.IDLE);
  const [originalContent, setOriginalContent] = useState(null);
  const [translatedContent, setTranslatedContent] = useState(null);
  const [error, setError] = useState(null);
  const [metadata, setMetadata] = useState(null);
  const scrollPositionRef = useRef(0);

  /**
   * Check if user can use translation (is authenticated)
   */
  const canTranslate = useCallback(() => {
    return isAuthenticated();
  }, []);

  /**
   * Save current scroll position
   */
  const saveScrollPosition = useCallback(() => {
    if (typeof window !== 'undefined') {
      scrollPositionRef.current = window.scrollY;
    }
  }, []);

  /**
   * Restore saved scroll position
   */
  const restoreScrollPosition = useCallback(() => {
    if (typeof window !== 'undefined') {
      // Use requestAnimationFrame to ensure DOM has updated
      requestAnimationFrame(() => {
        window.scrollTo({
          top: scrollPositionRef.current,
          behavior: 'instant'
        });
      });
    }
  }, []);

  /**
   * Translate content to Urdu
   *
   * @param {string} content - HTML content to translate
   * @returns {Promise<string|null>} Translated content or null on error
   */
  const translate = useCallback(async (content) => {
    if (!content) {
      setError('No content to translate');
      setState(TRANSLATION_STATES.ERROR);
      return null;
    }

    if (!canTranslate()) {
      setError('Please log in to use translation.');
      setState(TRANSLATION_STATES.ERROR);
      return null;
    }

    // Save original content and scroll position
    setOriginalContent(content);
    saveScrollPosition();
    setState(TRANSLATION_STATES.LOADING);
    setError(null);

    try {
      const response = await translateToUrdu({
        content,
        chapterId,
        sourceLanguage: 'en',
        preserveFormatting: true
      });

      if (response.success) {
        setTranslatedContent(response.translated_content);
        setMetadata(response.metadata);
        setState(TRANSLATION_STATES.TRANSLATED);
        restoreScrollPosition();
        return response.translated_content;
      } else {
        throw new Error('Translation unsuccessful');
      }
    } catch (err) {
      console.error('Translation error:', err);
      setError(err.message || 'Translation failed. Please try again.');
      setState(TRANSLATION_STATES.ERROR);
      return null;
    }
  }, [chapterId, canTranslate, saveScrollPosition, restoreScrollPosition]);

  /**
   * Revert to original content
   */
  const revert = useCallback(() => {
    saveScrollPosition();
    setState(TRANSLATION_STATES.IDLE);
    setTranslatedContent(null);
    setError(null);
    restoreScrollPosition();
  }, [saveScrollPosition, restoreScrollPosition]);

  /**
   * Retry translation after error
   */
  const retry = useCallback(async () => {
    if (originalContent) {
      return translate(originalContent);
    }
    return null;
  }, [originalContent, translate]);

  /**
   * Clear error state
   */
  const clearError = useCallback(() => {
    setError(null);
    if (state === TRANSLATION_STATES.ERROR) {
      setState(TRANSLATION_STATES.IDLE);
    }
  }, [state]);

  return {
    // State
    state,
    isLoading: state === TRANSLATION_STATES.LOADING,
    isTranslated: state === TRANSLATION_STATES.TRANSLATED,
    hasError: state === TRANSLATION_STATES.ERROR,
    error,
    metadata,
    originalContent,
    translatedContent,
    canTranslate: canTranslate(),

    // Actions
    translate,
    revert,
    retry,
    clearError
  };
};

export default useTranslation;
