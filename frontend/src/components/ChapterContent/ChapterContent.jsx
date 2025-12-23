import React, { useState, useEffect } from 'react';
import PersonalizationButton from '../PersonalizationButton/PersonalizationButton';
import { fetchPersonalizedContent, fetchUserProfile } from '../../services/personalizationService';
import { applyPersonalization, sanitizeHTML } from '../../utils/contentRenderer';
import './ChapterContent.css';

/**
 * ChapterContent Component
 * Displays chapter content with optional personalization
 */
const ChapterContent = ({ chapterId, defaultContent }) => {
  const [content, setContent] = useState(defaultContent);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [userProfile, setUserProfile] = useState(null);

  // Fetch user profile when component mounts
  useEffect(() => {
    const loadUserProfile = async () => {
      try {
        const profile = await fetchUserProfile();
        setUserProfile(profile);
      } catch (err) {
        console.error('Error fetching user profile:', err);
        // Use a default profile if fetching fails
        setUserProfile({
          experience_level: 'intermediate',
          programming_languages: ['javascript'],
          domain_expertise: ['web development']
        });
      }
    };

    loadUserProfile();
  }, []);

  const handlePersonalize = async (chapterId) => {
    setIsLoading(true);
    setError(null);

    try {
      if (!userProfile) {
        throw new Error('User profile not available');
      }

      // Fetch personalized content from backend
      const response = await fetchPersonalizedContent(chapterId);

      // Apply any additional frontend personalization if needed
      const personalizedContent = applyPersonalization(
        response.personalized_content,
        userProfile
      );

      setContent(sanitizeHTML(personalizedContent));
      setIsPersonalized(true);
    } catch (err) {
      setError(err.message);
      console.error('Personalization error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  // Reset to default content when chapter changes
  useEffect(() => {
    setContent(sanitizeHTML(defaultContent));
    setIsPersonalized(false);
  }, [chapterId, defaultContent]);

  return (
    <div className="chapter-content-container">
      <div className="chapter-controls">
        <PersonalizationButton
          chapterId={chapterId}
          onPersonalize={handlePersonalize}
        />
        {isPersonalized && (
          <button
            className="reset-button"
            onClick={() => {
              setContent(sanitizeHTML(defaultContent));
              setIsPersonalized(false);
            }}
          >
            Show Original
          </button>
        )}
      </div>

      {error && (
        <div className="error-message">
          Error loading personalized content: {error}
        </div>
      )}

      {isLoading && !isPersonalized && (
        <div className="loading-indicator">
          Applying personalization...
        </div>
      )}

      <div
        className={`chapter-content ${isPersonalized ? 'personalized' : ''}`}
        dangerouslySetInnerHTML={{ __html: content }}
      />
    </div>
  );
};

export default ChapterContent;