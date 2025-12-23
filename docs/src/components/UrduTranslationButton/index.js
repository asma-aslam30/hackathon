/**
 * UrduTranslationButton Component
 *
 * A React component that provides Urdu translation functionality for chapter content.
 * Displays a button to translate content to Urdu and allows reverting to original.
 */
import React, { useEffect, useRef, useCallback } from 'react';
import styles from './styles.module.css';
import { useTranslation, TRANSLATION_STATES } from './hooks/useTranslation';

/**
 * UrduTranslationButton Component
 *
 * @param {Object} props - Component props
 * @param {string} props.contentSelector - CSS selector for the content to translate
 * @param {string} [props.chapterId] - Optional chapter identifier for analytics
 * @param {Function} [props.onTranslate] - Callback when translation completes
 * @param {Function} [props.onRevert] - Callback when content reverts to original
 * @param {Function} [props.onError] - Callback when error occurs
 * @returns {JSX.Element} The translation button component
 */
const UrduTranslationButton = ({
  contentSelector = '.theme-doc-markdown',
  chapterId = null,
  onTranslate = null,
  onRevert = null,
  onError = null
}) => {
  const contentRef = useRef(null);
  const originalHTMLRef = useRef(null);

  const {
    state,
    isLoading,
    isTranslated,
    hasError,
    error,
    metadata,
    canTranslate,
    translate,
    revert,
    retry,
    clearError
  } = useTranslation({ chapterId });

  // Find and store reference to content element
  useEffect(() => {
    if (typeof document !== 'undefined') {
      const contentElement = document.querySelector(contentSelector);
      if (contentElement) {
        contentRef.current = contentElement;
        // Store original HTML on first load
        if (!originalHTMLRef.current) {
          originalHTMLRef.current = contentElement.innerHTML;
        }
      }
    }
  }, [contentSelector]);

  /**
   * Handle translate button click
   */
  const handleTranslate = useCallback(async () => {
    if (!contentRef.current) {
      console.error('Content element not found');
      return;
    }

    // Store original HTML if not already stored
    if (!originalHTMLRef.current) {
      originalHTMLRef.current = contentRef.current.innerHTML;
    }

    const translatedContent = await translate(originalHTMLRef.current);

    if (translatedContent) {
      // Update DOM with translated content
      contentRef.current.innerHTML = translatedContent;
      contentRef.current.setAttribute('dir', 'rtl');
      contentRef.current.setAttribute('lang', 'ur');
      contentRef.current.classList.add(styles.translatedContent);

      // Announce to screen readers
      announceToScreenReader('Content translated to Urdu');

      if (onTranslate) {
        onTranslate(translatedContent);
      }
    }
  }, [translate, onTranslate]);

  /**
   * Handle revert button click
   */
  const handleRevert = useCallback(() => {
    if (contentRef.current && originalHTMLRef.current) {
      // Restore original HTML
      contentRef.current.innerHTML = originalHTMLRef.current;
      contentRef.current.removeAttribute('dir');
      contentRef.current.removeAttribute('lang');
      contentRef.current.classList.remove(styles.translatedContent);

      revert();

      // Announce to screen readers
      announceToScreenReader('Content reverted to original');

      if (onRevert) {
        onRevert();
      }
    }
  }, [revert, onRevert]);

  /**
   * Handle retry after error
   */
  const handleRetry = useCallback(async () => {
    await retry();
  }, [retry]);

  /**
   * Handle error dismissal
   */
  const handleDismissError = useCallback(() => {
    clearError();
    if (onError) {
      onError(null);
    }
  }, [clearError, onError]);

  /**
   * Announce message to screen readers
   */
  const announceToScreenReader = (message) => {
    if (typeof document !== 'undefined') {
      const announcement = document.createElement('div');
      announcement.setAttribute('role', 'status');
      announcement.setAttribute('aria-live', 'polite');
      announcement.className = styles.srOnly;
      announcement.textContent = message;
      document.body.appendChild(announcement);
      setTimeout(() => announcement.remove(), 1000);
    }
  };

  // Handle keyboard navigation
  const handleKeyDown = useCallback((event, action) => {
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      action();
    }
  }, []);

  // Don't render if user is not authenticated
  if (!canTranslate) {
    return (
      <div className={styles.translationContainer}>
        <p className={styles.loginPrompt}>
          <span
            className={styles.loginLink}
            onClick={() => {
              if (typeof window !== 'undefined') {
                window.location.href = '/login';
              }
            }}
            role="button"
            tabIndex={0}
            onKeyDown={(e) => handleKeyDown(e, () => window.location.href = '/login')}
          >
            Log in
          </span>{' '}
          to translate content to Urdu
        </p>
      </div>
    );
  }

  return (
    <div className={styles.translationContainer}>
      <div className={styles.buttonGroup}>
        {/* Translate Button */}
        {!isTranslated && (
          <button
            className={`${styles.translateButton} ${isLoading ? styles.loading : ''}`}
            onClick={handleTranslate}
            onKeyDown={(e) => handleKeyDown(e, handleTranslate)}
            disabled={isLoading}
            aria-label="Translate to Urdu"
            aria-busy={isLoading}
            aria-describedby={hasError ? 'translation-error' : undefined}
          >
            {isLoading ? (
              <>
                <span className={styles.spinner} aria-hidden="true" />
                Translating...
              </>
            ) : (
              <>
                <span aria-hidden="true">🇵🇰</span>
                Translate to Urdu
              </>
            )}
          </button>
        )}

        {/* Show Original Button */}
        {isTranslated && (
          <>
            <button
              className={`${styles.translateButton} ${styles.originalButton}`}
              onClick={handleRevert}
              onKeyDown={(e) => handleKeyDown(e, handleRevert)}
              aria-label="Show original content"
            >
              <span aria-hidden="true">↩️</span>
              Show Original
            </button>

            <span className={styles.rtlIndicator} aria-label="Content is in Urdu, reading right-to-left">
              <span aria-hidden="true">←</span>
              Urdu (RTL)
            </span>
          </>
        )}
      </div>

      {/* Error Message */}
      {hasError && error && (
        <div
          id="translation-error"
          className={styles.errorContainer}
          role="alert"
          aria-live="assertive"
        >
          <svg
            className={styles.errorIcon}
            fill="currentColor"
            viewBox="0 0 20 20"
            aria-hidden="true"
          >
            <path
              fillRule="evenodd"
              d="M10 18a8 8 0 100-16 8 8 0 000 16zM8.707 7.293a1 1 0 00-1.414 1.414L8.586 10l-1.293 1.293a1 1 0 101.414 1.414L10 11.414l1.293 1.293a1 1 0 001.414-1.414L11.414 10l1.293-1.293a1 1 0 00-1.414-1.414L10 8.586 8.707 7.293z"
              clipRule="evenodd"
            />
          </svg>
          <span className={styles.errorMessage}>{error}</span>
          <button
            className={styles.dismissButton}
            onClick={handleDismissError}
            aria-label="Dismiss error"
          >
            ✕
          </button>
          <button
            className={styles.translateButton}
            onClick={handleRetry}
            style={{ marginLeft: '0.5rem', padding: '0.25rem 0.5rem', fontSize: '0.8rem' }}
          >
            Try Again
          </button>
        </div>
      )}

      {/* Translation Metadata */}
      {isTranslated && metadata && (
        <div className={styles.metadata} aria-label="Translation information">
          {metadata.cache_hit ? 'Cached translation' : 'Fresh translation'} •{' '}
          {metadata.word_count} words • {metadata.translation_time_ms}ms
        </div>
      )}
    </div>
  );
};

export default UrduTranslationButton;
