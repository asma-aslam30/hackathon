import React, { useState, useEffect } from 'react';
import './Chatbot.css';
import chatbotService from './ChatbotService';

/**
 * Chatbot component for interacting with the RAG system
 * Allows users to submit queries about book content and receive AI-generated responses
 */
const Chatbot = () => {
  const [query, setQuery] = useState('');
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);

  // Check service health on component mount
  useEffect(() => {
    const checkHealth = async () => {
      try {
        const isHealthy = await chatbotService.checkHealth();
        if (!isHealthy) {
          console.warn('Chatbot service is not healthy');
          // Show a message to the user that the service is unavailable
          setMessages(prev => [...prev, {
            id: 'service-warning',
            text: 'Note: The chatbot backend service is currently unavailable. This may be because the backend is not deployed yet.',
            sender: 'system',
            timestamp: new Date(),
            warning: true
          }]);
        }
      } catch (error) {
        console.error('Error checking service health:', error.message);
        // Show a message to the user that the service is unavailable
        setMessages(prev => [...prev, {
          id: 'service-error',
          text: 'Note: The chatbot backend service is currently unavailable. This may be because the backend is not deployed yet.',
          sender: 'system',
          timestamp: new Date(),
          warning: true
        }]);
      }
    };

    checkHealth();
  }, []);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!query.trim() || isLoading) return;

    // Add user query to messages
    const userMessage = {
      id: Date.now(),
      text: query,
      sender: 'user',
      timestamp: new Date()
    };
    setMessages(prev => [...prev, userMessage]);

    setIsLoading(true);
    setQuery(''); // Clear input field

    try {
      // Call the backend API using the service
      const response = await chatbotService.submitQuery(query);

      const botMessage = {
        id: Date.now() + 1,
        text: response.response,
        sender: 'bot',
        timestamp: new Date(),
        sources: response.sources,
        confidence: response.confidenceScore,
        queryId: response.queryId
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: error.message || 'Sorry, I encountered an error processing your query. Please try again.',
        sender: 'bot',
        timestamp: new Date(),
        error: true
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="chatbot-container">
      <div className="chatbot-header">
        <h3>Book Assistant</h3>
      </div>

      <div className="chatbot-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Ask me anything about the book content!</p>
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.sender}-message ${message.warning ? 'warning-message' : ''}`}
            >
              <div className="message-text">{message.text}</div>
              {message.sources && message.sources.length > 0 && (
                <div className="message-sources">
                  Sources: {message.sources.join(', ')}
                </div>
              )}
              {message.confidence !== undefined && (
                <div className="message-confidence">
                  Confidence: {(message.confidence * 100).toFixed(0)}%
                </div>
              )}
            </div>
          ))
        )}

        {isLoading && (
          <div className="message bot-message">
            <div className="typing-indicator">
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        )}
      </div>

      <form className="chatbot-input-form" onSubmit={handleSubmit}>
        <input
          type="text"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          placeholder="Ask a question about the book..."
          disabled={isLoading}
          className="chatbot-input"
        />
        <button
          type="submit"
          disabled={!query.trim() || isLoading}
          className="chatbot-submit-button"
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </form>
    </div>
  );
};

export default Chatbot;