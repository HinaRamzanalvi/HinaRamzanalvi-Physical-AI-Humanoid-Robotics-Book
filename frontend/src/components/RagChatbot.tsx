import React, { useState, useEffect, useRef, useCallback } from 'react';
import { Message, ChatResponse, ChatRequest, Citation } from './types';
import styles from './RagChatbot.module.css';

const RagChatbotComponent: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState('');

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Initialize session
  useEffect(() => {
    const storedSessionId = localStorage.getItem('ragChatbotSessionId');
    if (storedSessionId) {
      setSessionId(storedSessionId);
    } else {
      // Create a new session by sending a dummy request
      createNewSession();
    }
  }, []);

  const createNewSession = async () => {
    try {
      const response = await fetch(`${import.meta.env.VITE_API_URL || 'http://localhost:8000'}/api/v1/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query_text: 'New session started',
          query_mode: 'AskBook',
        }),
      });

      if (response.ok) {
        const data: ChatResponse = await response.json();
        setSessionId(data.session_id);
        localStorage.setItem('ragChatbotSessionId', data.session_id);
      }
    } catch (error) {
      console.error('Error creating session:', error);
    }
  };

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection()?.toString().trim() || '';
      if (selectedText) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Auto-scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
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
        selected_text: selectedText,
        session_id: sessionId || undefined,
      };

      const response = await fetch(`${import.meta.env.VITE_API_URL || 'http://localhost:8000'}/api/v1/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data: ChatResponse = await response.json();

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

      const errorMessage: Message = {
        id: Date.now().toString(),
        text: 'Sorry, I encountered an error processing your request. Please try again.',
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
    </>
  );
};

export default RagChatbotComponent;