import React, { useState } from 'react';
import './PersonalizationButton.css';

/**
 * PersonalizationButton Component
 * A button that triggers content personalization based on user's background
 */
const PersonalizationButton = ({ chapterId, onPersonalize }) => {
  const [isLoading, setIsLoading] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);

  const handleClick = async () => {
    setIsLoading(true);
    try {
      // Call the personalization function passed from parent
      if (onPersonalize && typeof onPersonalize === 'function') {
        await onPersonalize(chapterId);
        setIsPersonalized(true);
      }
    } catch (error) {
      console.error('Error during personalization:', error);
      // Optionally show user feedback about the error
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <button
      className={`personalization-button ${isPersonalized ? 'personalized' : ''} ${isLoading ? 'loading' : ''}`}
      onClick={handleClick}
      disabled={isLoading}
      title={isPersonalized ? 'Content is personalized' : 'Personalize this chapter based on your background'}
    >
      {isLoading ? (
        <span className="loading-spinner">🔄</span>
      ) : isPersonalized ? (
        <span>✨ Personalized</span>
      ) : (
        <span>💡 Personalize Content</span>
      )}
    </button>
  );
};

export default PersonalizationButton;