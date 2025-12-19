import React, { useState, useEffect, useRef } from 'react';

// This is the self-contained Docusaurus-compatible RagChatbot component
const RagChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [selectedText, setSelectedText] = useState('');

  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Initialize session
  useEffect(() => {
    const initializeSession = async () => {
      const storedSessionId = localStorage.getItem('ragChatbotSessionId');

      // If we have a stored session ID, validate it by making a test request
      if (storedSessionId) {
        try {
          const testResponse = await fetch('http://localhost:8000/api/v1/chat/query', {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              query_text: 'Test session validity',
              query_mode: 'AskBook',
              session_id: storedSessionId,
            }),
          });

          if (testResponse.ok) {
            // Session is valid, use it
            setSessionId(storedSessionId);
          } else {
            // Session is invalid, clear it and create a new one
            const errorData = await testResponse.json().catch(() => ({}));
            if (testResponse.status === 422 && errorData.error?.message?.includes('Session not found')) {
              localStorage.removeItem('ragChatbotSessionId');
              await createNewSession();
            }
          }
        } catch (error) {
          // If there's a network error or other issue, clear the stored session and create new
          localStorage.removeItem('ragChatbotSessionId');
          await createNewSession();
        }
      } else {
        // No stored session, create a new one
        await createNewSession();
      }
    };

    initializeSession();
  }, []);

  const createNewSession = async () => {
    try {
      const response = await fetch('http://localhost:8000/api/v1/chat/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query_text: 'New session started',
          query_mode: 'AskBook',
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      if (data.session_id) {
        setSessionId(data.session_id);
        localStorage.setItem('ragChatbotSessionId', data.session_id);
        return data.session_id;
      } else {
        console.error('No session_id returned from server');
        return null;
      }
    } catch (error) {
      console.error('Error creating session:', error);
      // Create a temporary fallback by making the first user query create the session
      return null;
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

  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading) return;

    // Add user message to UI immediately
    const userMessage = {
      id: Date.now().toString(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const requestBody = {
        query_text: inputValue,
        query_mode: selectedText ? 'AskSelectedText' : 'AskBook',
        selected_text: selectedText,
      };

      // Only include session_id if it's a valid UUID format
      if (sessionId && /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i.test(sessionId)) {
        requestBody.session_id = sessionId;
      }

      const response = await fetch('http://localhost:8000/api/v1/chat/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        // If it's a session not found error, try to create a new session
        if (response.status === 422 && errorData.error?.message?.includes('Session not found')) {
          // Clear the invalid session ID and create a new one
          localStorage.removeItem('ragChatbotSessionId');
          setSessionId(null);
          const newSessionId = await createNewSession();

          // Retry the request without the session ID or with the new session ID
          const newRequestBody = {
            query_text: inputValue,
            query_mode: selectedText ? 'AskSelectedText' : 'AskBook',
            selected_text: selectedText,
          };

          // Only include session_id if we successfully created one and it's valid
          if (newSessionId && /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i.test(newSessionId)) {
            newRequestBody.session_id = newSessionId;
          } else {
            // If session creation failed, send request without session_id so backend creates one
            console.warn('Session creation failed, sending request without session_id');
          }

          const retryResponse = await fetch('http://localhost:8000/api/v1/chat/query', {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify(newRequestBody),
          });

          if (!retryResponse.ok) {
            // If retry also fails, try one more time without any session_id to force a fresh session
            const finalRequestBody = {
              query_text: inputValue,
              query_mode: selectedText ? 'AskSelectedText' : 'AskBook',
              selected_text: selectedText,
              // Don't include any session_id to force backend to create a new session
            };

            const finalResponse = await fetch('http://localhost:8000/api/v1/chat/query', {
              method: 'POST',
              headers: {
                'Content-Type': 'application/json',
              },
              body: JSON.stringify(finalRequestBody),
            });

            if (!finalResponse.ok) {
              throw new Error(`HTTP error on final attempt! status: ${finalResponse.status}`);
            }

            const finalData = await finalResponse.json();
            // Update session ID with the one returned from the server
            if (finalData.session_id) {
              setSessionId(finalData.session_id);
              localStorage.setItem('ragChatbotSessionId', finalData.session_id);
            }

            const botMessage = {
              id: Date.now().toString(),
              text: finalData.response_text,
              sender: 'bot',
              timestamp: new Date(),
              citations: finalData.citations,
            };

            setMessages(prev => [...prev, botMessage]);
            return; // Exit early after successful final attempt
          }

          const retryData = await retryResponse.json();
          // Update session ID with the one returned from the server
          if (retryData.session_id) {
            setSessionId(retryData.session_id);
            localStorage.setItem('ragChatbotSessionId', retryData.session_id);
          }

          const botMessage = {
            id: Date.now().toString(),
            text: retryData.response_text,
            sender: 'bot',
            timestamp: new Date(),
            citations: retryData.citations,
          };

          setMessages(prev => [...prev, botMessage]);
          return; // Exit early after successful retry
        } else {
          throw new Error(`HTTP error! status: ${response.status}`);
        }
      }

      const data = await response.json();

      // Update session ID if provided
      if (data.session_id) {
        setSessionId(data.session_id);
        localStorage.setItem('ragChatbotSessionId', data.session_id);
      }

      const botMessage = {
        id: Date.now().toString(),
        text: data.response_text,
        sender: 'bot',
        timestamp: new Date(),
        citations: data.citations,
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      const errorMessage = {
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
      setTimeout(() => inputRef.current.focus(), 100);
    }
  };

  const formatCitations = (citations = []) => {
    if (citations.length === 0) return null;

    return (
      <div style={{
        marginTop: '8px',
        paddingTop: '8px',
        borderTop: '1px dashed #d1d5db',
        fontSize: '12px',
        color: '#6b7280'
      }}>
        <div>Sources:</div>
        <ul style={{ margin: '4px 0 0 0', paddingLeft: '16px' }}>
          {citations.map((citation, index) => (
            <li key={index} style={{ marginBottom: '4px' }}>
              {citation.module} - {citation.chapter} - {citation.section_title}
            </li>
          ))}
        </ul>
      </div>
    );
  };

  const adjustTextareaHeight = (e) => {
    const textarea = e.target;
    textarea.style.height = 'auto';
    textarea.style.height = `${Math.min(textarea.scrollHeight, 120)}px`;
  };

  return (
    <>
      <button
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          backgroundColor: '#2563eb',
          color: 'white',
          border: 'none',
          cursor: 'pointer',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          fontSize: '24px',
          boxShadow: '0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06)',
          zIndex: 1000,
          transition: 'all 0.3s ease',
        }}
        onClick={toggleChat}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
        title="Open RAG Chatbot"
      >
        {isOpen ? '×' : (
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
            style={{ display: 'block' }}
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        )}
      </button>

      {isOpen && (
        <div style={{
          position: 'fixed',
          bottom: '80px',
          right: '20px',
          width: '400px',
          height: '500px',
          background: 'white',
          borderRadius: '8px',
          boxShadow: '0 10px 25px rgba(0, 0, 0, 0.15)',
          display: 'flex',
          flexDirection: 'column',
          zIndex: 1000,
          overflow: 'hidden',
        }}>
          <div style={{
            background: '#2563eb',
            color: 'white',
            padding: '16px',
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center',
          }}>
            <h2 style={{ margin: 0, fontSize: '18px', fontWeight: '600' }}>RAG Chatbot</h2>
            <button
              style={{
                background: 'none',
                border: 'none',
                color: 'white',
                fontSize: '20px',
                cursor: 'pointer',
                padding: '4px',
                borderRadius: '4px',
              }}
              onClick={toggleChat}
              aria-label="Close chat"
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
              style={{ display: 'block' }}
            >
              <line x1="18" y1="6" x2="6" y2="18"></line>
              <line x1="6" y1="6" x2="18" y2="18"></line>
            </svg>
            </button>
          </div>

          <div style={{
            flex: 1,
            overflowY: 'auto',
            padding: '16px',
            background: '#f9fafb',
            display: 'flex',
            flexDirection: 'column',
            gap: '12px',
          }}>
            {messages.length === 0 ? (
              <div style={{
                textAlign: 'center',
                color: '#6b7280',
                fontStyle: 'italic',
                margin: '20px 0',
              }}>
                <p>Hello! I'm your RAG Chatbot for the Physical AI & Humanoid Robotics textbook.</p>
                <p>Ask me anything about the textbook content!</p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  style={{
                    maxWidth: '80%',
                    padding: '12px 16px',
                    borderRadius: '18px',
                    position: 'relative',
                    wordWrap: 'break-word',
                    lineHeight: '1.5',
                    ...(message.sender === 'user'
                      ? {
                          alignSelf: 'flex-end',
                          backgroundColor: '#2563eb',
                          color: 'white',
                          marginLeft: 'auto',
                          borderBottomRightRadius: '4px',
                        }
                      : message.isError
                        ? {
                            backgroundColor: '#fee2e2',
                            color: '#dc2626',
                            border: '1px solid #fecaca',
                            marginRight: 'auto',
                            borderBottomLeftRadius: '4px',
                          }
                        : {
                            backgroundColor: 'white',
                            color: '#374151',
                            border: '1px solid #e5e7eb',
                            marginRight: 'auto',
                            borderBottomLeftRadius: '4px',
                          }),
                  }}
                >
                  <div>{message.text}</div>
                  {message.citations && formatCitations(message.citations)}
                </div>
              ))
            )}

            {isLoading && (
              <div style={{
                maxWidth: '80%',
                padding: '12px 16px',
                borderRadius: '18px',
                backgroundColor: 'white',
                color: '#374151',
                border: '1px solid #e5e7eb',
                marginRight: 'auto',
                borderBottomLeftRadius: '4px',
              }}>
                <div style={{
                  display: 'flex',
                  alignItems: 'center',
                  gap: '8px',
                  color: '#6b7280',
                  fontStyle: 'italic',
                }}>
                  <span>Thinking...</span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div style={{
            display: 'flex',
            padding: '16px',
            background: 'white',
            borderTop: '1px solid #e5e7eb',
            gap: '8px',
          }}>
            <form onSubmit={handleSubmit} style={{ display: 'flex', flex: 1, gap: '8px' }}>
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
                style={{
                  flex: 1,
                  padding: '12px 16px',
                  border: '1px solid #d1d5db',
                  borderRadius: '20px',
                  fontFamily: 'inherit',
                  fontSize: '14px',
                  resize: 'none',
                  minHeight: '40px',
                  maxHeight: '120px',
                  overflowY: 'auto',
                }}
                rows={1}
                disabled={isLoading}
              />
              <button
                type="submit"
                style={{
                  background: '#2563eb',
                  color: 'white',
                  border: 'none',
                  borderRadius: '50%',
                  width: '40px',
                  height: '40px',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  cursor: 'pointer',
                  flexShrink: 0,
                  ...(isLoading ? { background: '#d1d5db', cursor: 'not-allowed' } : {}),
                }}
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
                  style={{ display: 'block' }}
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

export default RagChatbot;