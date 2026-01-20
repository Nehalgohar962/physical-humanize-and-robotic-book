import React from 'react';
import { useLocation } from '@docusaurus/router';
import ChatWidget from '../components/ChatWidget';

const GlobalChatModule: React.FC = () => {
  const location = useLocation();

  // Don't show on the dedicated chat page to avoid duplication
  if (location.pathname === '/chat') {
    return null;
  }

  return <ChatWidget />;
};

export default GlobalChatModule;