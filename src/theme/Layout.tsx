import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { useLocation } from '@docusaurus/router';
import ChatWidget from '../components/ChatWidget';

type LayoutProps = {
  children: React.ReactNode;
  [key: string]: any;
};

// Verify layout override is active
console.log("CUSTOM LAYOUT ACTIVE");

const LayoutWrapper: React.FC<LayoutProps> = (props) => {
  const location = useLocation();

  return (
    <div className="page-wrapper">
      <OriginalLayout {...props} />
      {/* Show the chat widget on all pages except the /chat page */}
      {location.pathname !== '/chat' && <ChatWidget />}
    </div>
  );
};

export default LayoutWrapper;