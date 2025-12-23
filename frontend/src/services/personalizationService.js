/**
 * Service for interacting with the personalization API
 * Fetches user profile JSON from backend and applies personalization
 */

const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000/api/v1';

/**
 * Fetches user profile from backend
 * @returns {Promise<Object>} User profile data
 */
export const fetchUserProfile = async () => {
  try {
    const token = localStorage.getItem('auth_token'); // Assuming token is stored in localStorage
    const response = await fetch(`${API_BASE_URL}/profile`, {
      method: 'GET',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      throw new Error(`Failed to fetch user profile: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error fetching user profile:', error);
    throw error;
  }
};

/**
 * Fetches personalized content for a chapter based on user profile
 * @param {string} chapterId - ID of the chapter to personalize
 * @returns {Promise<Object>} Personalized chapter content
 */
export const fetchPersonalizedContent = async (chapterId) => {
  try {
    const token = localStorage.getItem('auth_token'); // Assuming token is stored in localStorage
    const response = await fetch(`${API_BASE_URL}/personalization/chapter/${chapterId}`, {
      method: 'GET',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      throw new Error(`Failed to fetch personalized content: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error fetching personalized content:', error);
    throw error;
  }
};

/**
 * Updates personalization preferences
 * @param {Object} preferences - User's personalization preferences
 * @returns {Promise<Object>} Updated preferences
 */
export const updatePreferences = async (preferences) => {
  try {
    const token = localStorage.getItem('auth_token');
    const response = await fetch(`${API_BASE_URL}/personalization/preferences`, {
      method: 'PUT',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(preferences),
    });

    if (!response.ok) {
      throw new Error(`Failed to update preferences: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error updating preferences:', error);
    throw error;
  }
};

/**
 * Submits feedback on personalization quality
 * @param {string} chapterId - ID of the chapter
 * @param {string} feedbackType - Type of feedback (positive/negative/neutral)
 * @param {string} feedbackText - Optional detailed feedback
 * @returns {Promise<Object>} Response from the API
 */
export const submitFeedback = async (chapterId, feedbackType, feedbackText = '') => {
  try {
    const token = localStorage.getItem('auth_token');
    const response = await fetch(`${API_BASE_URL}/personalization/feedback`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        chapter_id: chapterId,
        feedback_type: feedbackType,
        feedback_text: feedbackText,
      }),
    });

    if (!response.ok) {
      throw new Error(`Failed to submit feedback: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  } catch (error) {
    console.error('Error submitting feedback:', error);
    throw error;
  }
};