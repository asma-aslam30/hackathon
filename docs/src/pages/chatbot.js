import React from 'react';
import Layout from '@theme/Layout';
import Chatbot from '../components/Chatbot/Chatbot';
import './chatbot.css'; // Import the chatbot-specific styles

/**
 * Chatbot page component that renders the AI-powered chat interface
 * This page allows users to interact with the RAG system to ask questions about book content
 */
function ChatbotPage() {
  return (
    <Layout
      title="AI Chatbot - Book Assistant"
      description="Interact with our AI-powered assistant to ask questions about the book content">
      <div className="chatbot-page-container">
        <div className="container">
          <div className="row">
            <div className="col col--12">
              <div className="hero hero--primary text--center">
                <div className="hero__title">
                  <h1>AI Book Assistant</h1>
                </div>
                <div className="hero__subtitle">
                  <p>Ask questions about the book content and get AI-powered answers</p>
                </div>
              </div>
            </div>
          </div>

          <div className="row">
            <div className="col col--12">
              <div className="chatbot-wrapper">
                <Chatbot />
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default ChatbotPage;