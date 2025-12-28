/**
 * API Configuration for the Physical AI & Humanoid Robotics Platform
 *
 * This file centralizes all backend API URLs and configuration.
 * Update BACKEND_URL when deploying to different environments.
 */

// Backend API URL - Deployed on Hugging Face Spaces
// Live URL: https://asma-aslam30-physical-ai-chatbot.hf.space
const BACKEND_URL = 'https://asma-aslam30-physical-ai-chatbot.hf.space';

// Fallback to localhost for development
const getBackendUrl = (): string => {
  // Check if we're in development mode (localhost)
  if (typeof window !== 'undefined') {
    const hostname = window.location.hostname;
    if (hostname === 'localhost' || hostname === '127.0.0.1') {
      return 'http://localhost:8000';
    }
  }
  return BACKEND_URL;
};

export const API_CONFIG = {
  // Base URL for the backend API
  BACKEND_URL: getBackendUrl(),

  // API endpoints
  ENDPOINTS: {
    QUERY: '/api/v1/query',
    HEALTH: '/health',
    AUTH_LOGIN: '/api/v1/auth/login',
    AUTH_REGISTER: '/api/v1/auth/register',
    AUTH_LOGOUT: '/api/v1/auth/logout',
    AUTH_ME: '/api/v1/auth/me',
    PROFILE: '/api/v1/profile',
    TRANSLATE: '/api/v1/translation/translate',
    PERSONALIZATION: '/api/v1/personalization',
  },

  // Request timeout in milliseconds
  TIMEOUT: 30000,

  // Maximum query length
  MAX_QUERY_LENGTH: 1000,
};

export default API_CONFIG;
