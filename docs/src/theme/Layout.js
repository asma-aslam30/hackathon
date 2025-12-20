import React, { useState, useEffect } from 'react';
import OriginalLayout from '@theme-original/Layout';
import Link from '@docusaurus/Link';

// SVG Robot Icon
const RobotIcon = () => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    viewBox="0 0 24 24"
    fill="currentColor"
    style={{ width: '100%', height: '100%' }}
  >
    <path d="M20 9V7a2 2 0 0 0-2-2h-1.15a2 2 0 0 1-1.85-2 2 2 0 0 0-4 0A2 2 0 0 1 9.15 5H8a2 2 0 0 0-2 2v2a7 7 0 0 0 14 0ZM7 9a1 1 0 0 1 1-1h8a1 1 0 0 1 1 1v1a1 1 0 0 1-1 1H8a1 1 0 0 1-1-1V9Z" />
    <path d="M17 14a1 1 0 0 1 1 1v2a2 2 0 0 1-2 2H8a2 2 0 0 1-2-2v-2a1 1 0 0 1 1-1h12Z" />
    <circle cx="8" cy="11" r="1" />
    <circle cx="16" cy="11" r="1" />
    <path d="M12 16h.01" />
  </svg>
);

// Floating Chatbot Button Component
const FloatingChatbotButton = () => {
  const [isVisible, setIsVisible] = useState(true);

  useEffect(() => {
    // Handle scroll to show/hide button
    const handleScroll = () => {
      setIsVisible(window.scrollY < 100);
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  return (
    <Link
      to="/chatbot"
      className={`floating-chatbot-button ${isVisible ? 'show' : 'hide'}`}
      aria-label="AI Chatbot"
    >
      <RobotIcon />
    </Link>
  );
};

// Custom Layout wrapper
export default function LayoutWrapper(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <FloatingChatbotButton />
    </>
  );
}