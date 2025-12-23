/**
 * Content rendering utilities for personalization
 */

/**
 * Sanitizes HTML content to prevent XSS attacks
 * In a real implementation, use a library like DOMPurify
 * @param {string} html - HTML content to sanitize
 * @returns {string} Sanitized HTML
 */
export const sanitizeHTML = (html) => {
  // Basic sanitization - in production, use a proper library like DOMPurify
  const temp = document.createElement('div');
  temp.textContent = html;
  return temp.innerHTML;
};

/**
 * Applies personalization transformations to content
 * @param {string} content - Original content
 * @param {Object} userProfile - User's profile information
 * @returns {string} Personalized content
 */
export const applyPersonalization = (content, userProfile) => {
  let personalizedContent = content;

  // Adjust content based on experience level
  if (userProfile.experience_level === 'beginner') {
    // Add more explanations for beginners
    personalizedContent = addBeginnerExplanations(personalizedContent);
  } else if (userProfile.experience_level === 'advanced' || userProfile.experience_level === 'expert') {
    // Add more advanced concepts for experienced users
    personalizedContent = addAdvancedContent(personalizedContent);
  }

  // Customize examples based on programming languages
  if (userProfile.programming_languages && userProfile.programming_languages.length > 0) {
    personalizedContent = customizeExamples(personalizedContent, userProfile.programming_languages);
  }

  // Focus on relevant domains
  if (userProfile.domain_expertise && userProfile.domain_expertise.length > 0) {
    personalizedContent = applyDomainFocus(personalizedContent, userProfile.domain_expertise);
  }

  return personalizedContent;
};

/**
 * Adds beginner-friendly explanations to content
 * @param {string} content - Original content
 * @returns {string} Content with added explanations
 */
const addBeginnerExplanations = (content) => {
  // Replace complex terms with simpler explanations
  let updatedContent = content.replace(/\bAPI\b/g, '<abbr title="Application Programming Interface">API</abbr>');
  updatedContent = updatedContent.replace(/\bJSON\b/g, '<abbr title="JavaScript Object Notation">JSON</abbr>');

  // Add more descriptive text for beginners
  updatedContent = updatedContent.replace(/<code>/g, '<code class="beginner-friendly">');

  return updatedContent;
};

/**
 * Adds advanced content for experienced users
 * @param {string} content - Original content
 * @returns {string} Content with advanced concepts
 */
const addAdvancedContent = (content) => {
  // Add more technical depth
  return content;
};

/**
 * Customizes code examples based on user's programming languages
 * @param {string} content - Original content
 * @param {Array} languages - User's preferred programming languages
 * @returns {string} Content with customized examples
 */
const customizeExamples = (content, languages) => {
  let updatedContent = content;

  // Replace generic examples with language-specific ones
  if (languages.includes('javascript') || languages.includes('typescript')) {
    updatedContent = updatedContent.replace(/function (\w+)\(\)/g, 'function $1()');
    updatedContent = updatedContent.replace(/console\.log/g, 'console.log');
  }

  if (languages.includes('python')) {
    updatedContent = updatedContent.replace(/function (\w+)\(\)/g, 'def $1():');
    updatedContent = updatedContent.replace(/console\.log/g, 'print');
  }

  if (languages.includes('java')) {
    updatedContent = updatedContent.replace(/function (\w+)\(\)/g, 'public void $1()');
    updatedContent = updatedContent.replace(/console\.log/g, 'System.out.println');
  }

  return updatedContent;
};

/**
 * Applies domain-specific focus to content
 * @param {string} content - Original content
 * @param {Array} domains - User's domain expertise
 * @returns {string} Content with domain focus
 */
const applyDomainFocus = (content, domains) => {
  let updatedContent = content;

  if (domains.includes('web development') || domains.includes('web_dev')) {
    updatedContent = updatedContent.replace(/generic example/g, 'web development example');
  }

  if (domains.includes('data science') || domains.includes('data_science') ||
      domains.includes('machine learning') || domains.includes('ml')) {
    updatedContent = updatedContent.replace(/generic example/g, 'data science example');
  }

  return updatedContent;
};

/**
 * Renders content with proper styling based on personalization
 * @param {string} content - Content to render
 * @param {boolean} isPersonalized - Whether content is personalized
 * @returns {HTMLElement} Rendered content element
 */
export const renderContent = (content, isPersonalized = false) => {
  const container = document.createElement('div');
  container.innerHTML = content;
  container.className = `content-container ${isPersonalized ? 'personalized' : ''}`;

  return container;
};