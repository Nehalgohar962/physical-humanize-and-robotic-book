import React from 'react';
import { useLocation } from '@docusaurus/router';
import ChatWidget from './ChatWidget';

const RootWrapper: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const location = useLocation();

  return (
    <>
      {children}
      {/* Only show the chat widget on non-chat pages */}
      {location.pathname !== '/chat' && <ChatWidget />}
    </>
  );
};

export default RootWrapper;