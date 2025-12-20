import axios from 'axios';
import { QueryRequest, QueryResponse } from './types';

// API configuration constants
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';
const API_TIMEOUT = 30000; // 30 seconds

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
   * Submit a query to the backend RAG system with retry mechanism
   * @param {string} query - The user's question/query
   * @param {string} [userId] - Optional user identifier
   * @param {string} [sessionId] - Optional session identifier
   * @param {number} [retryCount] - Number of retries attempted (internal use)
   * @returns {Promise<QueryResponse>} The response from the backend
   */
  async submitQuery(query: string, userId?: string, sessionId?: string, retryCount: number = 0): Promise<any> {
    // Validate input
    if (!query || typeof query !== 'string' || query.trim().length === 0) {
      throw new Error('Query is required and must be a non-empty string');
    }

    // Limit query length as per specification (max 1000 characters)
    if (query.length > 1000) {
      throw new Error('Query exceeds maximum length of 1000 characters');
    }

    try {
      const requestBody: QueryRequest = {
        query: query.trim(),
      };

      // Add optional fields if provided
      if (userId) {
        requestBody.userId = userId;
      }
      if (sessionId) {
        requestBody.sessionId = sessionId;
      }

      const response = await apiClient.post<QueryResponse>('/query', requestBody);

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
      if (axios.isAxiosError(error) && error.response) {
        // Server responded with error status
        const status = error.response.status;
        const message = error.response.data?.detail || `Server error: ${status}`;

        // For 5xx errors, implement retry mechanism (up to 3 attempts)
        if (status >= 500 && status < 600 && retryCount < 3) {
          console.warn(`Server error (${status}), attempt ${retryCount + 1}/3. Retrying...`);
          // Wait for increasing time before retry (exponential backoff)
          await this.delay(Math.pow(2, retryCount) * 1000);
          return this.submitQuery(query, userId, sessionId, retryCount + 1);
        }

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
      } else if (error instanceof Error) {
        // Request was made but no response received or other error
        if (error.message.includes('timeout')) {
          // For timeout errors, implement retry mechanism (up to 3 attempts)
          if (retryCount < 3) {
            console.warn(`Request timeout, attempt ${retryCount + 1}/3. Retrying...`);
            // Wait for increasing time before retry (exponential backoff)
            await this.delay(Math.pow(2, retryCount) * 1000);
            return this.submitQuery(query, userId, sessionId, retryCount + 1);
          }
          throw new Error('Request timeout: The server took too long to respond. Please try again.');
        } else if (error.message.includes('Network Error')) {
          // For network errors, implement retry mechanism (up to 3 attempts)
          if (retryCount < 3) {
            console.warn(`Network error, attempt ${retryCount + 1}/3. Retrying...`);
            // Wait for increasing time before retry (exponential backoff)
            await this.delay(Math.pow(2, retryCount) * 1000);
            return this.submitQuery(query, userId, sessionId, retryCount + 1);
          }
          throw new Error('Network error: Unable to reach the server. Please check your connection.');
        } else {
          throw new Error(`Request error: ${error.message}`);
        }
      } else {
        // Something else happened
        throw new Error(`Unknown error occurred: ${error}`);
      }
    }
  }

  /**
   * Helper function to create a delay
   * @param {number} ms - Number of milliseconds to delay
   * @returns {Promise<void>} Promise that resolves after the delay
   */
  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  /**
   * Check if the backend API is healthy
   * @returns {Promise<boolean>} True if the API is healthy, false otherwise
   */
  async checkHealth(): Promise<boolean> {
    try {
      const response = await apiClient.get('/health');
      return response.data.status === 'healthy';
    } catch (error) {
      console.warn('Health check failed:', error instanceof Error ? error.message : 'Unknown error');
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