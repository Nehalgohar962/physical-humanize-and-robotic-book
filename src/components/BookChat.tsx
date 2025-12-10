import React, { useState } from 'react';
import '../css/chat.css'; // Correct path

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

export default function BookChat() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<{ type: 'user' | 'ai'; text: string }[]>([]);
  const [input, setInput] = useState('');

  const handleSend = () => {
    if (!input.trim()) return;

    const userMsg = { type: 'user', text: input };
    const lower = input.toLowerCase();
    const answer = GENERAL_ANSWERS[lower] || MODULE_DETAILS[lower] || 
                   "Sorry, I can only answer questions related to book modules 1-6.";
    const aiMsg = { type: 'ai', text: answer };

    setMessages(prev => [...prev, userMsg, aiMsg]);
    setInput('');
  };

  return (
    <div className={`chat-container ${isOpen ? 'open' : ''}`}>
      {/* Toggle button */}
      {!isOpen && (
        <button className="chat-toggle-btn" onClick={() => setIsOpen(true)}>
          💬
        </button>
      )}

      {/* Chat box */}
      {isOpen && (
        <div className="chat-box">
          {messages.length === 0 && (
            <div className="ai-msg">💬 Hello! Ask me anything about the book modules 1-6.</div>
          )}
          {messages.map((m, idx) => (
            <div
              key={idx}
              className={`message ${m.type === 'user' ? 'user-msg' : 'ai-msg'}`}
            >
              {m.text}
            </div>
          ))}

          {/* Input */}
          <div className="chat-input-container">
            <input
              type="text"
              placeholder="Type your question here..."
              className="chat-input"
              value={input}
              onChange={e => setInput(e.target.value)}
              onKeyDown={e => e.key === 'Enter' && handleSend()}
            />
            <button className="chat-send-btn" onClick={handleSend}>➤</button>
          </div>
        </div>
      )}
    </div>
  );
}
