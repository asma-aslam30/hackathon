/**
 * Utility functions for formatting responses and other common operations
 */

/**
 * Format a confidence score as a percentage string
 * @param {number} confidence - Confidence score between 0 and 1
 * @returns {string} Formatted percentage string
 */
export function formatConfidence(confidence: number): string {
  if (typeof confidence !== 'number' || confidence < 0 || confidence > 1) {
    return 'N/A';
  }
  return `${(confidence * 100).toFixed(0)}%`;
}

/**
 * Format a timestamp for display
 * @param {string | Date} timestamp - ISO string or Date object
 * @returns {string} Formatted time string
 */
export function formatTime(timestamp: string | Date): string {
  try {
    const date = typeof timestamp === 'string' ? new Date(timestamp) : timestamp;
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  } catch (error) {
    return 'Invalid time';
  }
}

/**
 * Format source URLs for display
 * @param {string[]} sources - Array of source URLs
 * @returns {string} Formatted sources string
 */
export function formatSources(sources: string[]): string {
  if (!Array.isArray(sources) || sources.length === 0) {
    return '';
  }

  // Limit to first 3 sources to avoid overwhelming the UI
  const limitedSources = sources.slice(0, 3);
  return limitedSources.join(', ');
}

/**
 * Truncate text to a specified length
 * @param {string} text - Text to truncate
 * @param {number} maxLength - Maximum length
 * @param {string} suffix - Suffix to add if truncated
 * @returns {string} Truncated text
 */
export function truncateText(text: string, maxLength: number = 100, suffix: string = '...'): string {
  if (!text || typeof text !== 'string') {
    return '';
  }

  if (text.length <= maxLength) {
    return text;
  }

  return text.substring(0, maxLength - suffix.length) + suffix;
}

/**
 * Sanitize text for safe display (basic HTML escaping)
 * @param {string} text - Text to sanitize
 * @returns {string} Sanitized text
 */
export function sanitizeText(text: string): string {
  if (!text || typeof text !== 'string') {
    return '';
  }

  return text
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#039;');
}

/**
 * Format response text with basic markdown-like formatting
 * @param {string} text - Raw response text
 * @returns {string} Formatted text
 */
export function formatResponseText(text: string): string {
  if (!text || typeof text !== 'string') {
    return '';
  }

  // Basic formatting: convert newlines to <br> tags if needed
  // Note: In React components, use \n for line breaks
  return text;
}