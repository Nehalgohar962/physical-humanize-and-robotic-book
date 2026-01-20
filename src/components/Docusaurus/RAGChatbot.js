import React, { useState, useEffect } from 'react';
import ChatInterface from '../Chatbot/ChatInterface';
import './RAGChatbot.css';

const RAGChatbot = () => {
  const [sessionId, setSessionId] = useState('');
  const [isVisible, setIsVisible] = useState(false);
  const [selectedText, setSelectedText] = useState('');

  // Initialize session
  useEffect(() => {
    // Try to get session from localStorage
    const storedSessionId = localStorage.getItem('ragChatSessionId');
    if (storedSessionId) {
      setSessionId(storedSessionId);
    }
  }, []);

  // Handle session changes
  const handleSessionChange = (newSessionId) => {
    setSessionId(newSessionId);
    localStorage.setItem('ragChatSessionId', newSessionId);
  };

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection()?.toString().trim();
      if (selectedText) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const toggleChat = () => {
    setIsVisible(!isVisible);
  };

  return (
    <div className="rag-chatbot-container">
      {isVisible ? (
        <div className="chatbot-panel">
          <div className="chatbot-header">
            <h3>Textbook Assistant</h3>
            <button className="close-button" onClick={toggleChat}>
              ×
            </button>
          </div>
          <ChatInterface
            sessionId={sessionId}
            onSessionChange={handleSessionChange}
          />
        </div>
      ) : (
        <button className="chatbot-toggle" onClick={toggleChat}>
          💬 AI Assistant
        </button>
      )}

      {/* Floating button that appears when text is selected */}
      {selectedText && (
        <button
          className="context-button"
          onClick={() => {
            // Set context mode and pre-fill the context text
            // This would require passing context to the chat interface
            setIsVisible(true);
          }}
        >
          💬 Ask about selected text
        </button>
      )}
    </div>
  );
};

export default RAGChatbot;