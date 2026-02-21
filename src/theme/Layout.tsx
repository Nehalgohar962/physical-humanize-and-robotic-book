import React from 'react';
import OriginalLayout from '@theme-original/Layout';

type LayoutProps = {
  children: React.ReactNode;
  [key: string]: any;
};

// Verify layout override is active
console.log("CUSTOM LAYOUT ACTIVE");

const LayoutWrapper: React.FC<LayoutProps> = (props) => {
  return (
    <div className="page-wrapper">
      <OriginalLayout {...props} />
    </div>
  );
};

export default LayoutWrapper;