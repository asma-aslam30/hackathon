import React, { useState, useEffect } from 'react';
import './PersonalizationButton.css';
import API_CONFIG from '../../config/api';

const getApiBaseUrl = () => API_CONFIG.BACKEND_URL;

/**
 * PersonalizationButton Component for Docusaurus
 * A button that triggers content personalization based on user's background
 */
const PersonalizationButton = ({ chapterId }) => {
  const [isLoading, setIsLoading] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [userProfile, setUserProfile] = useState(null);

  // Fetch user profile when component mounts
  useEffect(() => {
    const fetchUserProfile = async () => {
      try {
        // Check if we're in a browser environment
        if (typeof window !== 'undefined') {
          const token = localStorage.getItem('auth_token');
          if (!token) {
            console.log('No auth token found, user may not be logged in');
            return;
          }

          const response = await fetch(`${getApiBaseUrl()}/api/v1/profile`, {
            method: 'GET',
            headers: {
              'Authorization': `Bearer ${token}`,
              'Content-Type': 'application/json',
            },
          });

          if (response.ok) {
            const profile = await response.json();
            setUserProfile(profile);
          } else {
            console.error('Failed to fetch user profile:', response.status);
          }
        }
      } catch (error) {
        console.error('Error fetching user profile:', error);
      }
    };

    fetchUserProfile();
  }, []);

  const handleClick = async () => {
    if (!userProfile) {
      alert('Please log in to personalize content');
      return;
    }

    setIsLoading(true);
    try {
      // Find the chapter content container to update
      const chapterContainer = document.querySelector('.theme-doc-markdown');
      if (!chapterContainer) {
        console.error('Chapter container not found');
        return;
      }

      // Fetch personalized content from backend
      const token = localStorage.getItem('auth_token');
      const response = await fetch(`${getApiBaseUrl()}/api/v1/personalization/chapter/${chapterId || 'current'}`, {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        throw new Error(`Failed to fetch personalized content: ${response.status}`);
      }

      const result = await response.json();

      // Update the chapter content with personalized version
      chapterContainer.innerHTML = result.personalized_content;
      setIsPersonalized(true);

      // Add personalized styling
      chapterContainer.classList.add('personalized-content');
    } catch (error) {
      console.error('Error personalizing content:', error);
      alert('Failed to personalize content. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <button
      className={`personalization-button ${isPersonalized ? 'personalized' : ''} ${isLoading ? 'loading' : ''}`}
      onClick={handleClick}
      disabled={isLoading || !userProfile}
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