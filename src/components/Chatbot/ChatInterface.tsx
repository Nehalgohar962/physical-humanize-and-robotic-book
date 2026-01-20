import React, { useState, useEffect, useRef } from 'react';
import Message from './Message';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  references?: string[]; // Optional references for assistant messages
}

interface ChatInterfaceProps {
  sessionId: string;
  onSessionChange: (sessionId: string) => void;
}

const ChatInterface: React.FC<ChatInterfaceProps> = ({ sessionId, onSessionChange }) => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);
  const chatMessagesRef = useRef<HTMLDivElement>(null);
  const shouldAutoScroll = useRef(true);

  // Initialize session
  useEffect(() => {
    console.log('DEBUG: ChatInterface mounted, sessionId:', sessionId); // Log mount
    if (!sessionId) {
      createNewSession();
    } else {
      // Load existing session messages if needed
      loadSessionMessages();
    }
  }, [sessionId]);

  // Set up scroll event listener to detect manual scrolling
  useEffect(() => {
    const chatContainer = chatMessagesRef.current;
    if (!chatContainer) return;

    const handleScroll = () => {
      if (!chatContainer) return;

      // Calculate if user has scrolled near the bottom (within 10px)
      const threshold = 10;
      const position = chatContainer.scrollTop + chatContainer.clientHeight;
      const height = chatContainer.scrollHeight;
      shouldAutoScroll.current = position >= height - threshold;
    };

    chatContainer.addEventListener('scroll', handleScroll);

    return () => {
      chatContainer.removeEventListener('scroll', handleScroll);
    };
  }, []);

  const loadSessionMessages = async () => {
    try {
      const response = await fetch(`/api/session/sessions/${sessionId}/messages`);
      if (response.ok) {
        const messageData = await response.json();
        setMessages(messageData.map((msg: any) => ({
          id: msg.id,
          role: msg.role,
          content: msg.content,
          timestamp: new Date(msg.timestamp)
        })));
      }
    } catch (error) {
      console.error('Error loading session messages:', error);
    }
  };

  const createNewSession = async () => {
    try {
      // Try the session router first
      let response = await fetch('/api/session/sessions', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({}),
      });

      if (!response.ok) {
        console.warn('Session router failed, trying chat router...');
        try {
          const errorText = await response.text(); // Get error text before trying chat router
          console.warn('Session router error:', errorText);
        } catch (e) {
          console.warn('Could not read session router error text:', e);
        }

        // Fallback to chat router if session router fails
        response = await fetch('/api/chat/sessions', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({}),
        });
      }

      if (response.ok) {
        const data = await response.json();
        // Handle both response formats: session router returns ChatSession with 'id',
        // chat router returns CreateSessionResponse with 'session_id'
        const sessionId = data.id || data.session_id;
        if (sessionId) {
          onSessionChange(sessionId);
          return sessionId;
        } else {
          console.error('Session created but no ID returned:', data);
        }
      } else {
        console.error('Both session endpoints failed');
        try {
          const errorText = await response.text();
          console.error('Error response:', errorText);
        } catch (e) {
          console.error('Could not read error response:', e);
        }
      }
    } catch (error) {
      console.error('Network error creating session:', error);
      // Don't return null immediately - let the chat endpoint handle session creation
    }

    // Return empty string to allow backend to auto-create session
    return '';
  };

  const scrollToBottom = () => {
    if (shouldAutoScroll.current) {
      // Use setTimeout to ensure DOM is updated before scrolling for smooth experience
      setTimeout(() => {
        if (messagesEndRef.current) {
          messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
        }
      }, 10); // Small delay to ensure DOM updates are complete
    }
  };

  useEffect(() => {
    console.log('DEBUG: Messages updated, current messages:', messages); // Log messages changes
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent | React.KeyboardEvent) => {
    e.preventDefault();
    if (!inputText.trim() || isLoading) return;

    // Try to get or create a session, but don't fail if session creation fails
    // The backend chat endpoint can auto-create sessions if needed
    let currentSessionId = sessionId;
    if (!currentSessionId) {
      console.log('DEBUG: No session_id found, attempting to create new session...');
      currentSessionId = await createNewSession();
      if (!currentSessionId) {
        console.warn('DEBUG: Session creation failed, proceeding with empty session ID - backend will auto-create');
        // Don't return here - let the backend handle session creation
        currentSessionId = ''; // Let backend auto-create session
      } else {
        onSessionChange(currentSessionId); // Update parent component with new session ID
      }
    }

    // Add user message to UI immediately
    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: inputText,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputText('');

    // Reset textarea height
    const textarea = e.target as HTMLTextAreaElement;
    if (textarea && textarea.tagName === 'TEXTAREA') {
      textarea.style.height = 'auto';
    } else {
      // If it's from form submission, find the textarea
      const textareaElement = document.querySelector('textarea[placeholder*="Ask a question"]') as HTMLTextAreaElement;
      if (textareaElement) {
        textareaElement.style.height = 'auto';
      }
    }

    setIsLoading(true);

    try {
      // Prepare the request payload - use full_book mode by default and specify all 6 modules
      const requestBody = {
        session_id: currentSessionId || '',  // Pass session ID or empty string (backend will auto-create)
        message: inputText,
        mode: 'full_book', // Always use full_book mode since mode selector is removed
        modules: [1, 2, 3, 4, 5, 6], // Explicitly specify all 6 modules for comprehensive search
      };

      // FULL DEBUG LOGGING - Log the request payload
      console.log('DEBUG: Request payload:', requestBody);

      const response = await fetch('/api/chat/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      // FULL DEBUG LOGGING - Log response status
      console.log('DEBUG: Response status:', response.status);

      // FULL DEBUG LOGGING - Get response text BEFORE json parsing
      const responseText = await response.text();
      console.log('DEBUG: Raw response text:', responseText);

      if (response.ok) {
        try {
          const data = JSON.parse(responseText);
          console.log('DEBUG: Parsed backend response:', data); // Log the parsed response

          if (data.response) {
            const assistantMessage: Message = {
              id: Date.now().toString(),
              role: 'assistant',
              content: data.response,
              timestamp: new Date(),
              references: data.references || [], // Include references from the response
            };

            console.log('DEBUG: Adding assistant message:', assistantMessage); // Log what we're adding
            setMessages(prev => [...prev, assistantMessage]);

            // Update session ID if backend created a new one
            if (data.session_id && !sessionId) {
              onSessionChange(data.session_id);
            }
          } else {
            // Backend returned 200 but no response field - this shouldn't happen with our improved backend
            const errorMessage: Message = {
              id: Date.now().toString(),
              role: 'assistant',
              content: `Error: Invalid response format from server. The response did not contain the expected 'response' field.`,
              timestamp: new Date(),
              references: [],
            };
            setMessages(prev => [...prev, errorMessage]);
          }
        } catch (parseError) {
          console.error('DEBUG: JSON parse error:', parseError);
          console.error('DEBUG: Response text that failed to parse:', responseText);

          // If the response is not valid JSON but the server returned 200, it might be an error page
          if (responseText.toLowerCase().includes('error') || responseText.length < 200) {
            const errorMessage: Message = {
              id: Date.now().toString(),
              role: 'assistant',
              content: `Error: ${responseText.substring(0, 200)}...`,
              timestamp: new Date(),
              references: [],
            };
            setMessages(prev => [...prev, errorMessage]);
          } else {
            // If it's a valid-looking response but not JSON, try to display it
            const assistantMessage: Message = {
              id: Date.now().toString(),
              role: 'assistant',
              content: responseText,
              timestamp: new Date(),
              references: [],
            };
            setMessages(prev => [...prev, assistantMessage]);
          }
        }
      } else {
        // Response was not OK - this could be API quota exceeded, server error, etc.
        console.log('DEBUG: Non-OK response received');

        try {
          // Try to parse the error response
          const errorData = JSON.parse(responseText);
          console.log('DEBUG: Parsed error response:', errorData);

          // Check if it's a specific API error that we should handle gracefully
          if (response.status === 500 && typeof errorData.detail === 'string' &&
              (errorData.detail.toLowerCase().includes('quota') ||
               errorData.detail.toLowerCase().includes('rate limit') ||
               errorData.detail.toLowerCase().includes('exceeded'))) {
            // This is likely an API quota/rate limit error - backend should have handled this gracefully
            // But if it bubbled up to us, show a more user-friendly message
            const errorMessage: Message = {
              id: Date.now().toString(),
              role: 'assistant',
              content: `I'm currently unable to process your request due to API limitations. However, I can still provide information based on the textbook content. What would you like to know about Physical AI & Humanoid Robotics?`,
              timestamp: new Date(),
              references: [],
            };
            setMessages(prev => [...prev, errorMessage]);
          } else {
            // Standard error handling
            const errorMessage: Message = {
              id: Date.now().toString(),
              role: 'assistant',
              content: `Error ${response.status}: ${errorData.detail || errorData.message || 'Request failed'}\n\nPlease try rephrasing your question or check if the backend is running.`,
              timestamp: new Date(),
              references: [],
            };
            setMessages(prev => [...prev, errorMessage]);
          }
        } catch (parseError) {
          // If error response is not JSON, handle based on status code
          console.error('DEBUG: Error response not JSON:', parseError);

          if (response.status === 500) {
            // Server error - likely API quota or connection issue
            const errorMessage: Message = {
              id: Date.now().toString(),
              role: 'assistant',
              content: `I'm currently unable to process your request due to API limitations. However, I have access to the full textbook content and can answer questions about all 6 modules. What would you like to know about Physical AI & Humanoid Robotics?`,
              timestamp: new Date(),
              references: [],
            };
            setMessages(prev => [...prev, errorMessage]);
          } else {
            // Other HTTP error
            const errorMessage: Message = {
              id: Date.now().toString(),
              role: 'assistant',
              content: `Error ${response.status}: ${responseText.substring(0, 200)}...\n\nPlease try again or check if the backend is running.`,
              timestamp: new Date(),
              references: [],
            };
            setMessages(prev => [...prev, errorMessage]);
          }
        }
      }
    } catch (error) {
      // FULL DEBUG LOGGING - Log the actual error
      console.error('DEBUG: Network/fetch error:', error);
      console.log('DEBUG: Full error details:', { error, sessionId: currentSessionId, message: inputText });

      // Check if this is a connection error (backend not running)
      if ((error as Error).message.toLowerCase().includes('fetch') ||
          (error as Error).message.toLowerCase().includes('network') ||
          (error as Error).message.toLowerCase().includes('failed to fetch')) {
        const errorMessage: Message = {
          id: Date.now().toString(),
          role: 'assistant',
          content: 'Error: Unable to connect to the backend server. The API service may be temporarily unavailable or experiencing high traffic.',
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, errorMessage]);
      } else {
        // Other network or fetch errors
        const errorMessage: Message = {
          id: Date.now().toString(),
          role: 'assistant',
          content: `Connection error: ${(error as Error).message || 'Failed to connect to the server'}\n\nPlease check if the backend is running and accessible.`,
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } finally {
      setIsLoading(false);
    }
  };


  const clearChat = async () => {
    setMessages([]);
    try {
      await createNewSession(); // Create a new session when clearing chat
    } catch (error) {
      console.error('Error creating new session:', error);
    }
  };

  return (
    <div className="chat-interface">
      <div className="chat-messages" ref={chatMessagesRef}>
        {messages.length === 0 ? (
          <div className="welcome-message">
            <div className="welcome-icon">
              <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H7L3 21V5C3 4.46957 3.21071 3.96086 3.58579 3.58579C3.96086 3.21071 4.46957 3 5 3H19C19.5304 3 20.0391 3.21071 20.4142 3.58579C20.7893 3.96086 21 4.46957 21 5V15Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
            </div>
            <h4>Welcome to the AI Assistant</h4>
            <p>Ask me anything about Physical AI & Humanoid Robotics, and I'll provide detailed answers based on the textbook content.</p>
          </div>
        ) : (
          messages.map((message, index) => (
            <div
              key={message.id}
              className={`message-container ${message.role}`}
              style={{ animationDelay: `${index * 0.1}s` }}
            >
              <Message
                role={message.role}
                content={message.content}
                timestamp={message.timestamp}
                references={message.references}
              />
            </div>
          ))
        )}
        {isLoading && (
          <div className="message-container assistant">
            <div className="message message-assistant">
              <div className="message-content">
                <div className="typing-indicator">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <form className="chat-input-form" onSubmit={(e) => {
          e.preventDefault();
          handleSubmit(e as React.FormEvent);
        }}>
        <textarea
          value={inputText}
          onChange={(e) => {
            setInputText(e.target.value);
            // Auto-resize textarea based on content
            e.target.style.height = 'auto'; // Reset height to calculate new height
            e.target.style.height = Math.min(e.target.scrollHeight, 150) + 'px'; // Max 150px height
          }}
          onKeyDown={(e) => {
            if (e.key === 'Enter' && !e.shiftKey) {
              e.preventDefault();
              handleSubmit(e as any); // Type assertion to handle form submission
            }
          }}
          placeholder="Ask a question about the textbook content..."
          disabled={isLoading}
        />
        <button
          type="submit"
          disabled={isLoading || !inputText.trim()}
          className="send-button"
        >
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M22 2L11 13" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            <path d="M22 2L15 22L11 13L2 9L22 2Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        </button>
      </form>
    </div>
  );
};

export default ChatInterface;