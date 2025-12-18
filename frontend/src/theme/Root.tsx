/**
 * Root component for Docusaurus.
 *
 * Wraps the entire app with ErrorBoundary, ChatProvider, AuthProvider and includes
 * ChatWidget and TextSelectionTooltip on all pages.
 */

import React, { ReactNode, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import ErrorBoundary from '../components/ErrorBoundary';
import { ChatProvider } from '../context/ChatContext';
import { AuthProvider } from '../context/AuthContext';
import { ChatWidget } from '../components/ChatWidget';
import { TextSelectionTooltip } from '../components/TextSelectionTooltip';
import '../css/personalize.css';

interface RootProps {
  children: ReactNode;
}

export default function Root({ children }: RootProps): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  // Set API base URL on window object for utility modules
  useEffect(() => {
    if (typeof window !== 'undefined') {
      (window as any).__DOCUSAURUS_API_BASE_URL__ =
        (siteConfig.customFields?.apiBaseUrl as string) || 'http://localhost:8000';
    }
  }, [siteConfig]);

  return (
    <ErrorBoundary>
      <AuthProvider>
        <ChatProvider>
          {children}
          <ChatWidget />
          <TextSelectionTooltip />
        </ChatProvider>
      </AuthProvider>
    </ErrorBoundary>
  );
}
