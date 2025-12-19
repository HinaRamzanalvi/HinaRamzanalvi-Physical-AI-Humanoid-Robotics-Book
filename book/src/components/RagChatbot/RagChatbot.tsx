import React, { useState, useEffect, useRef, useCallback } from 'react';
import { Message, ChatResponse, ChatRequest, Citation } from './types';
import TextSelectionHandler from './TextSelectionHandler';
import styles from './RagChatbot.module.css';

const RagChatbotComponent: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState('');
  const [explanationComplexity, setExplanationComplexity] = useState<'beginner' | 'intermediate' | 'advanced' | null>(null);

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Debug logging for state changes
  useEffect(() => {
    console.log('Chatbot open state changed:', isOpen);
  }, [isOpen]);

  useEffect(() => {
    console.log('Messages state changed:', messages);
  }, [messages]);

  useEffect(() => {
    console.log('Input value state changed:', inputValue);
  }, [inputValue]);

  useEffect(() => {
    console.log('Loading state changed:', isLoading);
  }, [isLoading]);

  useEffect(() => {
    console.log('Session ID state changed:', sessionId);
  }, [sessionId]);

  useEffect(() => {
    console.log('Selected text state changed:', selectedText);
  }, [selectedText]);

  useEffect(() => {
    console.log('Explanation complexity state changed:', explanationComplexity);
  }, [explanationComplexity]);

  // Initialize session
  useEffect(() => {
    console.log('Initializing session...'); // Debug logging
    const storedSessionId = localStorage.getItem('ragChatbotSessionId');
    console.log('Retrieved session ID from localStorage:', storedSessionId); // Debug logging
    if (storedSessionId) {
      setSessionId(storedSessionId);
      console.log('Set existing session ID:', storedSessionId); // Debug logging
    } else {
      console.log('No existing session found, creating new session...'); // Debug logging
      // Create a new session by sending a dummy request
      createNewSession();
    }
  }, []);

  const createNewSession = async () => {
    console.log('Starting createNewSession function'); // Debug logging
    try {
      const requestBody = {
        query_text: 'New session started',
        query_mode: 'AskBook',
        selected_text: undefined,
        explanation_complexity: undefined,
        session_id: undefined,
      };

      console.log('Creating new session with request:', requestBody); // Debug logging

      const response = await fetch(`${import.meta.env.VITE_API_URL || 'http://localhost:8000'}/api/v1/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      console.log('Session creation response status:', response.status); // Debug logging

      if (response.ok) {
        const data: ChatResponse = await response.json();
        console.log('Session creation response:', data); // Debug logging
        setSessionId(data.session_id);
        localStorage.setItem('ragChatbotSessionId', data.session_id);
        console.log('Session ID set and saved to localStorage:', data.session_id); // Debug logging
      } else {
        console.error('Session creation failed:', await response.text()); // Debug logging
      }
    } catch (error) {
      console.error('Error creating session:', error);
    }
    console.log('Completed createNewSession function'); // Debug logging
  };

  const handleTextSelected = (selectedText: string) => {
    console.log('Text selected:', selectedText); // Debug logging
    setSelectedText(selectedText);
    console.log('Updated selectedText state:', selectedText); // Debug logging
  };

  // Auto-scroll to bottom of messages
  useEffect(() => {
    console.log('Messages updated, triggering scroll to bottom'); // Debug logging
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    console.log('Scrolling to bottom of messages'); // Debug logging
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    console.log('Scroll to bottom completed'); // Debug logging
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading) return;

    // Add user message to UI immediately
    const userMessage: Message = {
      id: Date.now().toString(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const requestBody: ChatRequest = {
        query_text: inputValue,
        query_mode: selectedText ? 'AskSelectedText' : 'AskBook',
        selected_text: selectedText || undefined,
        explanation_complexity: explanationComplexity || undefined,
        session_id: sessionId || undefined,
      };

      console.log('Sending request to backend:', requestBody); // Debug logging

      const response = await fetch(`${import.meta.env.VITE_API_URL || 'http://localhost:8000'}/api/v1/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      console.log('Response status:', response.status); // Debug logging

      let data;
      let errorMessage = null;

      if (!response.ok) {
        // Handle error response from API
        try {
          const errorData = await response.json();
          console.error('Backend error response:', errorData); // Debug logging
          errorMessage = errorData.detail || errorData.error?.message || `Server error: ${response.status} ${response.statusText}`;
        } catch (jsonError) {
          console.error('Non-JSON error response:', await response.text()); // Debug logging
          // If response is not JSON, use status text
          errorMessage = `Server error: ${response.status} ${response.statusText}`;
        }

        throw new Error(errorMessage);
      }

      data = await response.json();
      console.log('Backend response:', data); // Debug logging

      // Update session ID if it changed
      if (data.session_id && data.session_id !== sessionId) {
        setSessionId(data.session_id);
        localStorage.setItem('ragChatbotSessionId', data.session_id);
      }

      const botMessage: Message = {
        id: Date.now().toString(),
        text: data.response_text,
        sender: 'bot',
        timestamp: new Date(),
        citations: data.citations,
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      // Extract user-friendly error message
      let userErrorMessage = 'Sorry, I encountered an error processing your request. Please try again.';

      if (error instanceof Error) {
        const errorText = error.message.toLowerCase();

        // Provide more specific error messages based on error type
        if (errorText.includes('network') || errorText.includes('fetch')) {
          userErrorMessage = 'Unable to connect to the server. Please check your internet connection and try again.';
        } else if (errorText.includes('429')) {
          userErrorMessage = 'Too many requests. Please wait a moment before trying again.';
        } else if (errorText.includes('400') || errorText.includes('validation')) {
          userErrorMessage = 'Invalid request. Please check your input and try again.';
        } else if (errorText.includes('500') || errorText.includes('internal')) {
          userErrorMessage = 'The server encountered an error. Please try again later.';
        } else if (errorText.includes('404')) {
          userErrorMessage = 'The service is temporarily unavailable. Please try again later.';
        } else if (errorText.includes('timeout') || errorText.includes('504')) {
          userErrorMessage = 'Request timed out. Please try again.';
        } else {
          // Use the specific error message if it's appropriate for users
          if (error.message && error.message.length < 200 && !error.message.includes('Error:')) {
            userErrorMessage = error.message;
          }
        }
      }

      const errorMessage: Message = {
        id: Date.now().toString(),
        text: userErrorMessage,
        sender: 'bot',
        timestamp: new Date(),
        isError: true,
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  };

  const clearChat = () => {
    setMessages([]);
  };

  const formatCitations = (citations: Citation[] = []) => {
    if (citations.length === 0) return null;

    return (
      <div className={styles.citations}>
        <div>Sources:</div>
        <ul>
          {citations.map((citation, index) => (
            <li key={index}>
              {citation.module} - {citation.chapter} - {citation.section_title}
            </li>
          ))}
        </ul>
      </div>
    );
  };

  const adjustTextareaHeight = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    const textarea = e.target;
    textarea.style.height = 'auto';
    textarea.style.height = `${Math.min(textarea.scrollHeight, 120)}px`;
  };

  return (
    <>
      <button
        className={`${styles.floatingButton} ${isOpen ? styles.open : ''}`}
        onClick={toggleChat}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
        title="Open RAG Chatbot"
      >
        {isOpen ? '×' : '?'}
      </button>

      {isOpen && (
        <div className={styles.chatModal}>
          <div className={styles.header}>
            <h2>RAG Chatbot</h2>
            <div className={styles.complexityControls}>
              <label htmlFor="complexity-select">Explanation:</label>
              <select
                id="complexity-select"
                value={explanationComplexity || ''}
                onChange={(e) => setExplanationComplexity(e.target.value as 'beginner' | 'intermediate' | 'advanced' | null)}
                className={styles.complexitySelect}
              >
                <option value="">Default</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>
            <button
              className={styles.closeButton}
              onClick={toggleChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          <div className={styles.messagesContainer}>
            {messages.length === 0 ? (
              <div className={styles.welcomeMessage}>
                <p>Hello! I'm your RAG Chatbot for the Physical AI & Humanoid Robotics textbook.</p>
                <p>Ask me anything about the textbook content!</p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`${styles.message} ${
                    message.sender === 'user'
                      ? styles.userMessage
                      : message.sender === 'bot' && message.isError
                        ? `${styles.botMessage} ${styles.errorMessage}`
                        : styles.botMessage
                  }`}
                >
                  <div>{message.text}</div>
                  {message.citations && formatCitations(message.citations)}
                </div>
              ))
            )}

            {isLoading && (
              <div className={`${styles.message} ${styles.botMessage}`}>
                <div className={styles.loading}>
                  <span>Thinking...</span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className={styles.inputArea}>
            <form onSubmit={handleSubmit}>
              <textarea
                ref={inputRef}
                value={inputValue}
                onChange={(e) => {
                  setInputValue(e.target.value);
                  adjustTextareaHeight(e);
                }}
                placeholder={selectedText
                  ? "Ask about the selected text..."
                  : "Ask a question about the textbook..."}
                className={styles.inputField}
                rows={1}
                disabled={isLoading}
              />
              <button
                type="submit"
                className={styles.sendButton}
                disabled={!inputValue.trim() || isLoading}
                aria-label="Send message"
              >
                <svg
                  width="16"
                  height="16"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                >
                  <line x1="22" y1="2" x2="11" y2="13"></line>
                  <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
                </svg>
              </button>
            </form>
          </div>
        </div>
      )}
      <TextSelectionHandler onTextSelected={handleTextSelected} />
    </>
  );
};

export default RagChatbotComponent;