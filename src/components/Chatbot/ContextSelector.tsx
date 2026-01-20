import React from 'react';

interface ContextSelectorProps {
  contextText: string;
  onContextChange: (text: string) => void;
}

const ContextSelector: React.FC<ContextSelectorProps> = ({ contextText, onContextChange }) => {
  return (
    <div className="context-selector">
      <label htmlFor="context-input">Context Text:</label>
      <textarea
        id="context-input"
        value={contextText}
        onChange={(e) => onContextChange(e.target.value)}
        placeholder="Enter or paste the text context here..."
        rows={4}
      />
    </div>
  );
};

export default ContextSelector;