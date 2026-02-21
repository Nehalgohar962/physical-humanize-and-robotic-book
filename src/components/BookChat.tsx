import React, { useState } from 'react';
import '../css/custom.css';

const GENERAL_ANSWERS: Record<string, string> = {
  hi: 'Hello! How can I assist you with the book? 💬',
  hello: 'Hello! How can I assist you with the book? 💬',
  hey: 'Hey! I am here to help with the book modules 1-6.',
  'how are you': "I'm good! How are you? I can help you with the book modules.",
  bye: 'Goodbye! Happy reading 😊',
};

const MODULE_DETAILS: Record<string, string> = {
  'module 1': 'Module 1: Intro to Physical AI & Robotics. Covers basic robotics concepts and physical AI introduction.',
  'module 2': 'Module 2: AI & Robotics Basics. Learn about AI principles and fundamental robotics systems.',
  'module 3': 'Module 3: Motion & Humanoid Design. Human-like motion, kinematics, and designing humanoid robots.',
  'module 4': 'Module 4: Advanced Robotics Systems. Deep dive into complex robotic systems and integration.',
  'module 5': 'Module 5: Human-Robot Interaction. HRI, communication, safety, user-centered design, and interaction scenarios.',
  'module 6': 'Module 6: Future of Physical AI. Emerging trends, AI integration, and future of humanoid robotics.',
};

// Get current time in HH:MM format
const getCurrentTime = () => {
  const now = new Date();
  const hours = now.getHours().toString().padStart(2, '0');
  const minutes = now.getMinutes().toString().padStart(2, '0');
  return `${hours}:${minutes}`;
};

export default function BookChat() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<
    { type: 'user' | 'ai'; text: string; time: string }[]
  >([]);
  const [input, setInput] = useState('');
  const [isTyping, setIsTyping] = useState(false);

  const handleSend = () => {
    if (!input.trim()) return;

    const userMsg = { type: 'user', text: input, time: getCurrentTime() };
    setMessages(prev => [...prev, userMsg]);
    setInput('');
    setIsTyping(true);

    // AI response after delay
    setTimeout(() => {
      const lower = input.toLowerCase().trim();

      // Check GENERAL_ANSWERS for partial match
      let answer = Object.keys(GENERAL_ANSWERS).find(key => lower.includes(key));
      if (answer) {
        answer = GENERAL_ANSWERS[answer];
      } else if (MODULE_DETAILS[lower]) {
        answer = MODULE_DETAILS[lower];
      } else {
        answer = `I can answer basic book modules questions. About "${input}", you might want to check ROS2 documentation.`;
      }

      const aiMsg = { type: 'ai', text: answer, time: getCurrentTime() };
      setMessages(prev => [...prev, aiMsg]);
      setIsTyping(false);
    }, 1200);
  };

  const handleClear = () => {
    setMessages([]);
  };

  return (
    <div>
      {/* Floating Chat Button */}
      {!isOpen && (
        <button className="chat-floating-button" onClick={() => setIsOpen(true)}>
          💬
        </button>
      )}

      {/* Chat Interface */}
      {isOpen && (
        <div className="chat-interface">
          <div className="chat-header">
            <span>Book Assistant</span>
            <div>
              <button className="clear-chat-btn" onClick={handleClear}>Clear</button>
              <button className="chat-close-btn" onClick={() => setIsOpen(false)}>✕</button>
            </div>
          </div>

          <div className="chat-messages">
            {messages.length === 0 && (
              <div className="welcome-message">
                <div className="welcome-icon">💬</div>
                Hello! Ask me anything about book modules 1-6.
              </div>
            )}
            {messages.map((msg, idx) => (
              <div
                key={idx}
                className={`message-container ${msg.type === 'user' ? 'user' : 'assistant'}`}
              >
                <div className="message-content">{msg.text}</div>
                <span className="message-timestamp">{msg.time}</span>
              </div>
            ))}

            {/* Typing Indicator */}
            {isTyping && (
              <div className="typing-indicator">
                <span>AI is typing</span>
                <div className="typing-dots">
                  <span className="typing-dot"></span>
                  <span className="typing-dot"></span>
                  <span className="typing-dot"></span>
                </div>
              </div>
            )}
          </div>

          <div className="chat-input-form">
            <textarea
              placeholder="Type your question here..."
              value={input}
              onChange={e => setInput(e.target.value)}
              onKeyDown={e => e.key === 'Enter' && !e.shiftKey && handleSend()}
            ></textarea>
            <button className="send-button" onClick={handleSend}>➤</button>
          </div>
        </div>
      )}
    </div>
  );
}
