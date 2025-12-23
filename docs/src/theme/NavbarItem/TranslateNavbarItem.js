/**
 * Custom Navbar Item for Urdu Translation Toggle
 */
import React, { useState, useEffect, useCallback } from 'react';
import { translateToUrdu, isAuthenticated } from '../../services/translationService';

const styles = {
  container: {
    display: 'flex',
    alignItems: 'center',
    gap: '8px',
  },
  button: {
    display: 'flex',
    alignItems: 'center',
    gap: '6px',
    padding: '6px 12px',
    borderRadius: '6px',
    border: 'none',
    cursor: 'pointer',
    fontSize: '14px',
    fontWeight: '500',
    transition: 'all 0.2s ease',
    backgroundColor: 'var(--ifm-color-primary)',
    color: 'white',
  },
  buttonDisabled: {
    opacity: 0.6,
    cursor: 'not-allowed',
  },
  buttonTranslated: {
    backgroundColor: '#059669',
  },
  buttonLoading: {
    backgroundColor: '#6366f1',
  },
  spinner: {
    width: '14px',
    height: '14px',
    border: '2px solid transparent',
    borderTopColor: 'white',
    borderRadius: '50%',
    animation: 'spin 1s linear infinite',
  },
  loginLink: {
    color: 'var(--ifm-color-primary)',
    textDecoration: 'none',
    fontSize: '14px',
    fontWeight: '500',
    padding: '6px 12px',
    borderRadius: '6px',
    backgroundColor: 'var(--ifm-color-primary-lightest)',
  },
};

// Add keyframes for spinner
if (typeof document !== 'undefined') {
  const styleSheet = document.createElement('style');
  styleSheet.textContent = `
    @keyframes spin {
      to { transform: rotate(360deg); }
    }
  `;
  document.head.appendChild(styleSheet);
}

export default function TranslateNavbarItem() {
  const [isTranslated, setIsTranslated] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [originalContent, setOriginalContent] = useState(null);
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const [isDocPage, setIsDocPage] = useState(false);

  useEffect(() => {
    // Check if user is logged in
    setIsLoggedIn(isAuthenticated());

    // Check if we're on a doc page
    if (typeof window !== 'undefined') {
      const path = window.location.pathname;
      setIsDocPage(path.includes('/docs/'));
    }
  }, []);

  // Reset translation state on navigation
  useEffect(() => {
    const handleRouteChange = () => {
      setIsTranslated(false);
      setOriginalContent(null);
      const path = window.location.pathname;
      setIsDocPage(path.includes('/docs/'));
    };

    if (typeof window !== 'undefined') {
      // Listen for popstate (browser back/forward)
      window.addEventListener('popstate', handleRouteChange);

      // Listen for Docusaurus route changes
      const observer = new MutationObserver(() => {
        const path = window.location.pathname;
        if (isDocPage !== path.includes('/docs/')) {
          handleRouteChange();
        }
      });

      observer.observe(document.body, { childList: true, subtree: true });

      return () => {
        window.removeEventListener('popstate', handleRouteChange);
        observer.disconnect();
      };
    }
  }, [isDocPage]);

  const handleTranslate = useCallback(async () => {
    if (isLoading) return;

    const contentElement = document.querySelector('.theme-doc-markdown');
    if (!contentElement) {
      alert('No content found to translate. Please navigate to a documentation page.');
      return;
    }

    if (isTranslated && originalContent) {
      // Revert to original
      contentElement.innerHTML = originalContent;
      contentElement.removeAttribute('dir');
      contentElement.removeAttribute('lang');
      contentElement.style.fontFamily = '';
      setIsTranslated(false);
      return;
    }

    // Save original content
    setOriginalContent(contentElement.innerHTML);
    setIsLoading(true);

    try {
      const response = await translateToUrdu({
        content: contentElement.innerHTML,
        chapterId: window.location.pathname,
        preserveFormatting: true,
      });

      if (response.success && response.translated_content) {
        contentElement.innerHTML = response.translated_content;
        contentElement.setAttribute('dir', 'rtl');
        contentElement.setAttribute('lang', 'ur');
        contentElement.style.fontFamily = "'Noto Nastaliq Urdu', serif";
        setIsTranslated(true);
      }
    } catch (error) {
      console.error('Translation error:', error);
      alert(error.message || 'Translation failed. Please try again.');
    } finally {
      setIsLoading(false);
    }
  }, [isLoading, isTranslated, originalContent]);

  // Don't show if not on a doc page
  if (!isDocPage) {
    return null;
  }

  // Show login prompt if not authenticated
  if (!isLoggedIn) {
    return (
      <a href="/hackathon/login" style={styles.loginLink}>
        Login to Translate
      </a>
    );
  }

  return (
    <div style={styles.container}>
      <button
        onClick={handleTranslate}
        disabled={isLoading}
        style={{
          ...styles.button,
          ...(isLoading ? styles.buttonLoading : {}),
          ...(isTranslated ? styles.buttonTranslated : {}),
          ...(isLoading ? styles.buttonDisabled : {}),
        }}
        title={isTranslated ? 'Show Original' : 'Translate to Urdu'}
      >
        {isLoading ? (
          <>
            <span style={styles.spinner} />
            Translating...
          </>
        ) : isTranslated ? (
          <>
            <span>EN</span>
            Show Original
          </>
        ) : (
          <>
            <span>UR</span>
            Translate to Urdu
          </>
        )}
      </button>
    </div>
  );
}
