import React, { useState, useEffect } from 'react';
import ChatInterface from './Chatbot/ChatInterface';

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [sessionId, setSessionId] = useState<string>('');
  const [hasInitialized, setHasInitialized] = useState(false);

  // Initialize session when component mounts
  useEffect(() => {
    if (!hasInitialized) {
      // Try to get existing session from localStorage or create new one
      const storedSessionId = localStorage.getItem('chatbot_session_id');
      if (storedSessionId) {
        setSessionId(storedSessionId);
      }
      setHasInitialized(true);
    }
  }, [hasInitialized]);

  const handleSessionChange = (newSessionId: string) => {
    setSessionId(newSessionId);
    localStorage.setItem('chatbot_session_id', newSessionId);
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const closeChat = () => {
    setIsOpen(false);
  };

  // Handle click outside to close chat
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (isOpen) {
        const chatWindow = document.querySelector('.chat-window');
        const chatButton = document.querySelector('.chat-button');

        if (chatWindow && !chatWindow.contains(event.target as Node) &&
            chatButton && !chatButton.contains(event.target as Node)) {
          // Only close if clicking outside both the chat window and button
          if (!(event.target as Element).closest('.chat-window') &&
              !(event.target as Element).closest('.chat-button')) {
            setIsOpen(false);
          }
        }
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

  return (
    <>
      {/* Floating Chat Button */}
      <button
        className="chat-button"
        onClick={toggleChat}
        aria-label="Open chat"
      >
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H17L14.25 20C13.9499 20.2376 13.5755 20.374 13.1875 20.387C12.7995 20.4 12.4258 20.2885 12.125 20.07L10.5 18.85C10.1875 18.6125 9.75 18.5 9.3125 18.5H5C4.46957 18.5 3.96086 18.2893 3.58579 17.9142C3.21071 17.5391 3 17.0304 3 16.5V7C3 6.46957 3.21071 5.96086 3.58579 5.58579C3.96086 5.21071 4.46957 5 5 5H19C19.5304 5 20.0391 5.21071 20.4142 5.58579C20.7893 5.96086 21 6.46957 21 7V15Z" stroke="white" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
          <path d="M15 10H9" stroke="white" stroke-width="2" stroke-linecap="round"/>
          <path d="M15 14H9" stroke="white" stroke-width="2" stroke-linecap="round"/>
        </svg>
      </button>

      {/* Chat Window - Rendered conditionally to ensure proper DOM structure */}
      {isOpen && (
        <div className="chat-window open">
          <div className="chat-header">
            <div className="header-content">
              <h3>AI Book Assistant</h3>
              <p>Ask questions about Physical AI & Humanoid Robotics</p>
            </div>
            <div className="header-controls">
              <button
                className="clear-session-btn"
                onClick={() => {
                  localStorage.removeItem('chatbot_session_id');
                  setSessionId('');
                  // Create new session will happen automatically in ChatInterface
                }}
                title="Start New Session"
              >
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M8 14C11.3137 14 14 11.3137 14 8C14 4.68629 11.3137 2 8 2C4.68629 2 2 4.68629 2 8C2 11.3137 4.68629 14 8 14Z" stroke="currentColor" stroke-width="2"/>
                  <path d="M5 8L8 11L11 8" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
                  <path d="M8 5V11" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                </svg>
              </button>
              <button className="close-chat-btn" onClick={closeChat}>
                <svg width="16" height="16" viewBox="0 0 16 16" fill="none" xmlns="http://www.w3.org/2000/svg">
                  <path d="M4 4L12 12" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                  <path d="M12 4L4 12" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
                </svg>
              </button>
            </div>
          </div>

          <ChatInterface
            sessionId={sessionId}
            onSessionChange={handleSessionChange}
          />
        </div>
      )}
    </>
  );
};

export default ChatWidget;