/**
 * ChatWidget component - floating chatbot interface.
 *
 * Provides a modal chat interface accessible from all pages with message history,
 * input field, and loading states.
 */

import React, { useState, useRef, useEffect } from 'react';
import { useChat } from '../context/ChatContext';
import { MessageBubble } from './MessageBubble';
import styles from './ChatWidget.module.css';

export function ChatWidget() {
  const {
    messages,
    isOpen,
    isLoading,
    error,
    sendMessage,
    clearHistory,
    toggleChat,
    closeChat,
    resetError,
  } = useChat();

  const [inputValue, setInputValue] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading) return;

    const message = inputValue.trim();
    setInputValue('');

    await sendMessage(message);
  };

  const handleClearHistory = async () => {
    if (window.confirm('Are you sure you want to clear the chat history?')) {
      await clearHistory();
    }
  };

  return (
    <>
      {/* Floating Chat Button */}
      <button
        className={styles.floatingButton}
        onClick={toggleChat}
        aria-label="Toggle chat"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat Modal */}
      {isOpen && (
        <div className={styles.chatModal}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.headerTitle}>
              <span className={styles.headerIcon}>🤖</span>
              <h3>Physical AI Assistant</h3>
            </div>
            <div className={styles.headerActions}>
              <button
                className={styles.clearButton}
                onClick={handleClearHistory}
                disabled={messages.length === 0}
                title="Clear chat history"
              >
                🗑️
              </button>
              <button
                className={styles.closeButton}
                onClick={closeChat}
                aria-label="Close chat"
              >
                ✕
              </button>
            </div>
          </div>

          {/* Error Banner */}
          {error && (
            <div className={styles.errorBanner}>
              <span>{error}</span>
              <button onClick={resetError} className={styles.errorClose}>
                ✕
              </button>
            </div>
          )}

          {/* Messages Container */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 ? (
              <div className={styles.emptyState}>
                <div className={styles.emptyStateIcon}>👋</div>
                <h4>Welcome to the Physical AI Assistant!</h4>
                <p>
                  Ask me anything about ROS 2, Gazebo simulation, NVIDIA Isaac Sim,
                  or Vision-Language-Action systems.
                </p>
                <div className={styles.exampleQuestions}>
                  <p className={styles.exampleLabel}>Try asking:</p>
                  <button
                    className={styles.exampleButton}
                    onClick={() => setInputValue('What is ROS 2?')}
                  >
                    What is ROS 2?
                  </button>
                  <button
                    className={styles.exampleButton}
                    onClick={() => setInputValue('How does Gazebo simulation work?')}
                  >
                    How does Gazebo simulation work?
                  </button>
                  <button
                    className={styles.exampleButton}
                    onClick={() => setInputValue('Explain NVIDIA Isaac Sim')}
                  >
                    Explain NVIDIA Isaac Sim
                  </button>
                </div>
              </div>
            ) : (
              <>
                {messages.map((message) => (
                  <MessageBubble
                    key={message.id}
                    role={message.role}
                    content={message.content}
                    timestamp={message.timestamp}
                    sources={message.sources}
                  />
                ))}

                {/* Loading Indicator */}
                {isLoading && (
                  <div className={styles.loadingIndicator}>
                    <div className={styles.typingDots}>
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                    <span className={styles.loadingText}>Thinking...</span>
                  </div>
                )}

                <div ref={messagesEndRef} />
              </>
            )}
          </div>

          {/* Input Form */}
          <form className={styles.inputForm} onSubmit={handleSubmit}>
            <input
              ref={inputRef}
              type="text"
              className={styles.messageInput}
              placeholder="Ask a question..."
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              disabled={isLoading}
              maxLength={2000}
            />
            <button
              type="submit"
              className={styles.sendButton}
              disabled={isLoading || !inputValue.trim()}
              aria-label="Send message"
            >
              {isLoading ? '⏳' : '➤'}
            </button>
          </form>
        </div>
      )}
    </>
  );
}
