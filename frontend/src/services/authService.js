import apiClient from './api';
import config from '../config/config';

class AuthService {
  // Register a new user
  async register(userData) {
    try {
      const response = await apiClient.post('/auth/register', userData);

      if (response.data.success && response.data.session_token) {
        // Store the token in localStorage
        localStorage.setItem(config.tokenKey, response.data.session_token);

        // Return user data
        return {
          success: true,
          user: response.data.user,
          token: response.data.session_token
        };
      } else {
        return {
          success: false,
          error: response.data.error || 'Registration failed'
        };
      }
    } catch (error) {
      console.error('Registration error:', error);
      return {
        success: false,
        error: error.response?.data?.error || error.message || 'Registration failed'
      };
    }
  }

  // Login user
  async login(credentials) {
    try {
      const response = await apiClient.post('/auth/login', credentials);

      if (response.data.success && response.data.session_token) {
        // Store the token in localStorage
        localStorage.setItem(config.tokenKey, response.data.session_token);

        // Return user data
        return {
          success: true,
          user: response.data.user,
          token: response.data.session_token
        };
      } else {
        return {
          success: false,
          error: response.data.error || 'Login failed'
        };
      }
    } catch (error) {
      console.error('Login error:', error);
      return {
        success: false,
        error: error.response?.data?.error || error.message || 'Invalid credentials'
      };
    }
  }

  // Logout user
  logout() {
    localStorage.removeItem(config.tokenKey);
  }

  // Check if user is authenticated
  isAuthenticated() {
    const token = localStorage.getItem(config.tokenKey);
    return !!token;
  }

  // Get current user profile
  async getCurrentUser() {
    try {
      const response = await apiClient.get('/profile/me');
      return {
        success: true,
        user: response.data
      };
    } catch (error) {
      console.error('Get user error:', error);
      return {
        success: false,
        error: error.response?.data?.error || error.message || 'Failed to get user data'
      };
    }
  }

  // Update user profile
  async updateProfile(profileData) {
    try {
      const response = await apiClient.put('/profile/me', profileData);
      return {
        success: true,
        user: response.data.profile
      };
    } catch (error) {
      console.error('Update profile error:', error);
      return {
        success: false,
        error: error.response?.data?.error || error.message || 'Failed to update profile'
      };
    }
  }

  // Get user token
  getToken() {
    return localStorage.getItem(config.tokenKey);
  }
}

// Export a singleton instance
const authService = new AuthService();
export default authService;