// Frontend configuration
const config = {
  // API settings
  apiUrl: process.env.REACT_APP_API_URL || 'http://localhost:8000/api',
  backendUrl: process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000',

  // Auth settings
  tokenKey: 'user_auth_token',

  // Feature flags
  enableBackgroundCollection: true,
  enablePersonalization: true,
};

export default config;