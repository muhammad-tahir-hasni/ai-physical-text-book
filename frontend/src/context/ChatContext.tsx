/**
 * Chat context for managing chatbot state across the application.
 *
 * Provides global state for chat messages, session management, and API interactions.
 */

import React, { createContext, useContext, useState, useCallback, ReactNode } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

interface Source {
  title: string;
  heading: string;
  score: number;
}

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: Source[];
}

interface ChatContextType {
  messages: Message[];
  sessionId: string | null;
  isOpen: boolean;
  isLoading: boolean;
  error: string | null;

  // Actions
  sendMessage: (message: string) => Promise<void>;
  clearHistory: () => Promise<void>;
  toggleChat: () => void;
  openChat: () => void;
  closeChat: () => void;
  resetError: () => void;
}

const ChatContext = createContext<ChatContextType | undefined>(undefined);

interface ChatProviderProps {
  children: ReactNode;
  apiUrl?: string;
}

export function ChatProvider({ children, apiUrl }: ChatProviderProps) {
  const { siteConfig } = useDocusaurusContext();
  const API_BASE_URL = apiUrl || (siteConfig.customFields?.apiBaseUrl as string) || 'http://localhost:8000';
  const [messages, setMessages] = useState<Message[]>([]);
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [isOpen, setIsOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const sendMessage = useCallback(async (message: string) => {
    if (!message.trim()) return;

    // Add user message immediately
    const userMessage: Message = {
      id: `user-${Date.now()}`,
      role: 'user',
      content: message,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);

    try {
      // Call chat API
      const response = await fetch(`${API_BASE_URL}/api/v1/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message,
          session_id: sessionId,
          include_history: true,
          top_k: 5,
        }),
      });

      if (!response.ok) {
        if (response.status === 429) {
          throw new Error('Rate limit exceeded. Please try again in a moment.');
        }
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Update session ID if new
      if (data.session_id && !sessionId) {
        setSessionId(data.session_id);
      }

      // Add assistant message
      const assistantMessage: Message = {
        id: `assistant-${Date.now()}`,
        role: 'assistant',
        content: data.answer,
        timestamp: new Date(data.timestamp),
        sources: data.sources,
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to send message';
      setError(errorMessage);

      // Add error message to chat
      const errorMsg: Message = {
        id: `error-${Date.now()}`,
        role: 'assistant',
        content: `Sorry, I encountered an error: ${errorMessage}`,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMsg]);
    } finally {
      setIsLoading(false);
    }
  }, [sessionId, API_BASE_URL]);

  const clearHistory = useCallback(async () => {
    if (!sessionId) {
      setMessages([]);
      return;
    }

    try {
      const response = await fetch(`${API_BASE_URL}/api/v1/chat/history/${sessionId}`, {
        method: 'DELETE',
      });

      if (!response.ok) {
        throw new Error('Failed to clear history');
      }

      // Clear local state
      setMessages([]);
      setSessionId(null);
      setError(null);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to clear history';
      setError(errorMessage);
    }
  }, [sessionId, API_BASE_URL]);

  const toggleChat = useCallback(() => {
    setIsOpen(prev => !prev);
  }, []);

  const openChat = useCallback(() => {
    setIsOpen(true);
  }, []);

  const closeChat = useCallback(() => {
    setIsOpen(false);
  }, []);

  const resetError = useCallback(() => {
    setError(null);
  }, []);

  const value: ChatContextType = {
    messages,
    sessionId,
    isOpen,
    isLoading,
    error,
    sendMessage,
    clearHistory,
    toggleChat,
    openChat,
    closeChat,
    resetError,
  };

  return (
    <ChatContext.Provider value={value}>
      {children}
    </ChatContext.Provider>
  );
}

export function useChat(): ChatContextType {
  const context = useContext(ChatContext);
  if (context === undefined) {
    throw new Error('useChat must be used within a ChatProvider');
  }
  return context;
}
