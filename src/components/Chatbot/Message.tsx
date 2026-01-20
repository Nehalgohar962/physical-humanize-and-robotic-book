import React from 'react';

interface MessageProps {
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  references?: string[]; // Optional references for assistant messages
}

const Message: React.FC<MessageProps> = ({ role, content, timestamp, references = [] }) => {
  const isUser = role === 'user';

  // Format timestamp
  const timeString = timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });

  return (
    <div className={`message message-${role}`}>
      <div className="message-avatar">
        {role === 'user' ? (
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M20 21V19C20 16.7909 18.2091 15 16 15H8C5.79086 15 4 16.7909 4 19V21" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            <path d="M12 11C14.2091 11 16 9.20914 16 7C16 4.79086 14.2091 3 12 3C9.79086 3 8 4.79086 8 7C8 9.20914 9.79086 11 12 11Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        ) : (
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M12 2C13.1046 2 14 2.89543 14 4C14 5.10457 13.1046 6 12 6C10.89543 6 10 5.10457 10 4C10 2.89543 10.89543 2 12 2Z" stroke="currentColor" strokeWidth="2"/>
            <path d="M20.59 22C20.59 18.13 16.74 15 12 15C7.26 15 3.41 18.13 3.41 22" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
            <path d="M8 7H16C17.1046 7 18 7.89543 18 9V13C18 14.1046 17.1046 15 16 15H8C6.89543 15 6 14.1046 6 13V9C6 7.89543 6.89543 7 8 7Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        )}
      </div>
      <div className="message-content">
        <div className="message-text">
          {content.split('\n').map((line, i) => (
            <React.Fragment key={i}>
              {line}
              {i < content.split('\n').length - 1 && <br />}
            </React.Fragment>
          ))}
        </div>

        {/* Display references for assistant messages */}
        {role === 'assistant' && references && references.length > 0 && (
          <div className="references-section">
            <h4 style={{color: '#00ffff'}}>📚 References from Book</h4>
            {references.map((ref, idx) => (
              <div key={idx} className="reference-item" style={{color: '#cccccc'}}>
                {typeof ref === 'string' ? ref : JSON.stringify(ref)}
              </div>
            ))}
          </div>
        )}

        <div className="message-timestamp">
          {timeString}
        </div>
      </div>
    </div>
  );
};

export default Message;