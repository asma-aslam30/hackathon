import axios from 'axios';
import API_CONFIG from '../../config/api';

// API configuration constants
const API_BASE_URL = API_CONFIG.BACKEND_URL;
const API_TIMEOUT = API_CONFIG.TIMEOUT;

// Create axios instance with default configuration
const apiClient = axios.create({
  baseURL: `${API_BASE_URL}/api/v1`,
  timeout: API_TIMEOUT,
  headers: {
    'Content-Type': 'application/json',
  },
});

/**
 * Service class for handling communication with the backend API
 */
class ChatbotService {
  /**
   * Submit a query to the backend RAG system
   * @param {string} query - The user's question/query
   * @param {string} [userId] - Optional user identifier
   * @param {string} [sessionId] - Optional session identifier
   * @returns {Promise<Object>} The response from the backend
   */
  async submitQuery(query, userId = null, sessionId = null) {
    // Validate input
    if (!query || typeof query !== 'string' || query.trim().length === 0) {
      throw new Error('Query is required and must be a non-empty string');
    }

    // Limit query length as per specification (max 1000 characters)
    if (query.length > 1000) {
      throw new Error('Query exceeds maximum length of 1000 characters');
    }

    try {
      const requestBody = {
        query: query.trim(),
      };

      // Add optional fields if provided
      if (userId) {
        requestBody.userId = userId;
      }
      if (sessionId) {
        requestBody.sessionId = sessionId;
      }

      const response = await apiClient.post('/query', requestBody);

      // Validate response structure
      if (!response.data || !response.data.response) {
        throw new Error('Invalid response format from backend');
      }

      return {
        response: response.data.response,
        queryId: response.data.query_id,
        retrievedChunks: response.data.retrieved_chunks || [],
        confidenceScore: response.data.confidence_score,
        sources: response.data.sources || [],
        timestamp: new Date().toISOString()
      };
    } catch (error) {
      // Handle different types of errors
      if (error.response) {
        // Server responded with error status
        const status = error.response.status;
        const message = error.response.data?.detail || `Server error: ${status}`;

        switch (status) {
          case 400:
            throw new Error(`Bad request: ${message}`);
          case 401:
            throw new Error('Unauthorized: Please check your API credentials');
          case 403:
            throw new Error('Forbidden: Access denied');
          case 404:
            throw new Error('API endpoint not found');
          case 500:
            throw new Error(`Server error: ${message}`);
          default:
            throw new Error(`API error (${status}): ${message}`);
        }
      } else if (error.request) {
        // Request was made but no response received
        throw new Error('Network error: Unable to reach the server. Please check your connection.');
      } else {
        // Something else happened
        throw new Error(`Request error: ${error.message}`);
      }
    }
  }

  /**
   * Check if the backend API is healthy
   * @returns {Promise<boolean>} True if the API is healthy, false otherwise
   */
  async checkHealth() {
    try {
      const response = await apiClient.get('/health');
      return response.data.status === 'healthy';
    } catch (error) {
      console.warn('Health check failed:', error.message);
      return false;
    }
  }

  /**
   * Get API configuration for debugging purposes
   * @returns {Object} Configuration object
   */
  getApiConfig() {
    return {
      baseUrl: apiClient.defaults.baseURL,
      timeout: apiClient.defaults.timeout,
      headers: { ...apiClient.defaults.headers.common }
    };
  }
}

// Export a singleton instance
const chatbotService = new ChatbotService();
export default chatbotService;

// Also export for direct import if needed
export { ChatbotService, apiClient, API_BASE_URL, API_TIMEOUT };