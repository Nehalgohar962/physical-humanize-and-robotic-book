import React from 'react';
import Layout from '@theme/Layout';
import ChatInterface from '../components/Chatbot/ChatInterface';
import '../css/custom.css';

const ChatPage: React.FC = () => {
  const [sessionId, setSessionId] = React.useState<string>('');

  const handleSessionChange = (newSessionId: string) => {
    setSessionId(newSessionId);
  };

  return (
    <Layout title="AI Book Assistant" description="Chat with the AI assistant for Physical AI & Humanoid Robotics">
      <div className="chat-page-container">
        <div className="chat-page-content">
          <div className="chat-header-section">
            <h1 className="chat-title">AI Book Assistant</h1>
            <p className="chat-subtitle">Ask questions about Physical AI & Humanoid Robotics</p>
          </div>

          <div className="chat-interface-container">
            <ChatInterface
              sessionId={sessionId}
              onSessionChange={handleSessionChange}
            />
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default ChatPage;