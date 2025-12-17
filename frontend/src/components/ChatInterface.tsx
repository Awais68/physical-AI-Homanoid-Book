import React, { useState, useRef, useEffect } from 'react';
import { gsap } from 'gsap';
import styles from './ChatInterface.module.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  timestamp: string;
  sources?: Source[];
}

interface Source {
  index: number;
  title: string;
  content_preview: string;
  source_url: string;
  relevance_score: number;
}

interface ChatResponse {
  answer: string;
  sources: Source[];
  citations: any[];
  confidence: number;
  has_sources: boolean;
}

const ChatInterface: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const robotRef = useRef<SVGSVGElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Robot animation
  useEffect(() => {
    if (robotRef.current) {
      // Idle floating animation
      gsap.to(robotRef.current, {
        y: -3,
        duration: 1.5,
        repeat: -1,
        yoyo: true,
        ease: 'power1.inOut',
      });
    }
  }, []);

  // Chat container animation
  useEffect(() => {
    if (isOpen && containerRef.current) {
      gsap.fromTo(
        containerRef.current,
        { opacity: 0, scale: 0.9, y: 20 },
        { opacity: 1, scale: 1, y: 0, duration: 0.3, ease: 'back.out(1.7)' }
      );
    }
  }, [isOpen]);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Save the message before clearing input
    const messageToSend = inputValue.trim();

    const userMessage: Message = {
      role: 'user',
      content: messageToSend,
      timestamp: new Date().toISOString(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Use backend URL - hardcoded for browser compatibility
      // const API_URL = 'http://localhost:8000';
      const API_URL = 'https://awais68-physical-ai-robotics.hf.space';
      console.log('Sending message to:', `${API_URL}/api/chat/message`);
      console.log('Message content:', messageToSend);

      const requestBody = {
        message: messageToSend,
        conversation_history: messages.map((m) => ({
          role: m.role,
          content: m.content,
        })),
        session_id: null,
      };

      console.log('Request body:', JSON.stringify(requestBody, null, 2));

      const response = await fetch(`${API_URL}/api/chat/message`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      console.log('Response status:', response.status);

      if (!response.ok) {
        const errorText = await response.text();
        console.error('API Error:', response.status, response.statusText);
        console.error('Error details:', errorText);
        throw new Error(`Failed to get response: ${response.status} - ${errorText}`);
      }

      const data: ChatResponse = await response.json();
      console.log('Received response:', data);

      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer,
        timestamp: new Date().toISOString(),
        sources: data.sources,
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Chat error:', error);
      const errorMessage: Message = {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date().toISOString(),
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <>
      {/* Chat Toggle Button - Animated Robot */}
      <button
        className={styles.chatToggle}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chat"
      >
        {isOpen ? (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <line x1="18" y1="6" x2="6" y2="18"></line>
            <line x1="6" y1="6" x2="18" y2="18"></line>
          </svg>
        ) : (
          <svg
            ref={robotRef}
            width="32"
            height="32"
            viewBox="0 0 64 64"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
            className={styles.robotIcon}
          >
            {/* Robot Head */}
            <rect x="14" y="18" width="36" height="28" rx="6" fill="currentColor" opacity="0.9" />
            {/* Antenna */}
            <line x1="32" y1="18" x2="32" y2="8" stroke="currentColor" strokeWidth="3" strokeLinecap="round" />
            <circle cx="32" cy="6" r="4" fill="#4ade80" className={styles.antennaLight} />
            {/* Eyes */}
            <circle cx="24" cy="30" r="5" fill="#1e3a5f" />
            <circle cx="40" cy="30" r="5" fill="#1e3a5f" />
            <circle cx="25" cy="29" r="2" fill="white" className={styles.eyeGlint} />
            <circle cx="41" cy="29" r="2" fill="white" className={styles.eyeGlint} />
            {/* Mouth - LED Display */}
            <rect x="22" y="38" width="20" height="4" rx="2" fill="#4ade80" className={styles.mouthLed} />
            {/* Ear panels */}
            <rect x="8" y="24" width="6" height="16" rx="2" fill="currentColor" opacity="0.7" />
            <rect x="50" y="24" width="6" height="16" rx="2" fill="currentColor" opacity="0.7" />
            {/* Body hint */}
            <rect x="24" y="46" width="16" height="8" rx="2" fill="currentColor" opacity="0.6" />
          </svg>
        )}
      </button>

      {/* Chat Container */}
      {isOpen && (
        <div ref={containerRef} className={styles.chatContainer}>
          <div className={styles.chatHeader}>
            <div className={styles.chatHeaderIcon}>
              <svg width="24" height="24" viewBox="0 0 64 64" fill="none" xmlns="http://www.w3.org/2000/svg">
                <rect x="14" y="18" width="36" height="28" rx="6" fill="white" opacity="0.9" />
                <circle cx="24" cy="30" r="4" fill="#1e3a5f" />
                <circle cx="40" cy="30" r="4" fill="#1e3a5f" />
                <rect x="22" y="38" width="20" height="3" rx="1.5" fill="#4ade80" />
              </svg>
            </div>
            <div className={styles.headerText}>
              <span className={styles.headerTitle}>AI Robot Assistant</span>
              <span className={styles.headerSubtitle}>Ask me about robotics!</span>
            </div>
            <button className={styles.closeButton} onClick={() => setIsOpen(false)} aria-label="Close chat">
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <line x1="18" y1="6" x2="6" y2="18"></line>
                <line x1="6" y1="6" x2="18" y2="18"></line>
              </svg>
            </button>
          </div>

          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <div className={styles.welcomeRobot}>
                  <svg width="64" height="64" viewBox="0 0 64 64" fill="none" xmlns="http://www.w3.org/2000/svg">
                    <rect x="14" y="18" width="36" height="28" rx="6" fill="var(--ifm-color-primary)" opacity="0.2" />
                    <rect x="14" y="18" width="36" height="28" rx="6" stroke="var(--ifm-color-primary)" strokeWidth="2" />
                    <circle cx="24" cy="30" r="4" fill="var(--ifm-color-primary)" />
                    <circle cx="40" cy="30" r="4" fill="var(--ifm-color-primary)" />
                    <rect x="22" y="38" width="20" height="3" rx="1.5" fill="var(--ifm-color-primary)" />
                    <line x1="32" y1="18" x2="32" y2="10" stroke="var(--ifm-color-primary)" strokeWidth="2" />
                    <circle cx="32" cy="8" r="3" fill="var(--ifm-color-primary)" />
                  </svg>
                </div>
                <h4>Welcome to the Physical AI Edge Kit Assistant!</h4>
                <p>Ask me anything about robotics, educational content, or the Edge Kit.</p>
              </div>
            )}

            {messages.map((message, index) => (
              <div
                key={index}
                className={`${styles.message} ${message.role === 'user' ? styles.userMessage : styles.assistantMessage
                  }`}
              >
                <div className={styles.messageContent}>{message.content}</div>
                {message.sources && message.sources.length > 0 && (
                  <div className={styles.sources}>
                    <small>Sources:</small>
                    {message.sources.slice(0, 3).map((source, i) => (
                      <span key={i} className={styles.sourceTag}>
                        {source.title}
                      </span>
                    ))}
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className={`${styles.message} ${styles.assistantMessage}`}>
                <div className={styles.loadingDots}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <div className={styles.inputContainer}>
            <textarea
              className={styles.input}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question..."
              rows={1}
              disabled={isLoading}
            />
            <button
              className={styles.sendButton}
              onClick={sendMessage}
              disabled={isLoading || !inputValue.trim()}
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <line x1="22" y1="2" x2="11" y2="13"></line>
                <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
              </svg>
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatInterface;
