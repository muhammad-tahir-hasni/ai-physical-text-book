/**
 * Custom DocItem Layout with Authentication and Personalization Support
 *
 * Wraps default Docusaurus DocItem layout to:
 * - Require authentication to view docs
 * - Add chapter personalization for authenticated users
 * - Display PersonalizeButton at the top of each chapter
 */

import React, { useState, useMemo, useRef, useEffect } from 'react';
import OriginalDocItemLayout from '@theme-original/DocItem/Layout';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import { Redirect } from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useAuth } from '../../../context/AuthContext';
import PersonalizeButton from '../../../components/PersonalizeButton';

type ComplexityLevel = 'beginner' | 'intermediate' | 'advanced';

export default function DocItemLayout(props) {
  const { isAuthenticated, isLoading } = useAuth();
  const { metadata } = useDoc();
  const homeUrl = useBaseUrl('/');
  const [personalizedHtml, setPersonalizedHtml] = useState<string | null>(null);
  const contentRef = useRef<HTMLDivElement>(null);
  const originalContentRef = useRef<string | null>(null);

  // Show loading state while checking authentication
  if (isLoading) {
    return (
      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '50vh',
        fontSize: '1.2rem',
        color: 'var(--ifm-color-content-secondary)'
      }}>
        Loading...
      </div>
    );
  }

  // Redirect to homepage if not authenticated
  if (!isAuthenticated) {
    return <Redirect to={homeUrl} />;
  }

  // Extract chapter ID from document path (e.g., "module-1/chapter-1-1" -> "1-1")
  const chapterId = useMemo(() => {
    const match = metadata.id?.match(/chapter-(\d+-\d+)/);
    return match ? match[1] : null;
  }, [metadata.id]);

  // Save original HTML content on first render
  useEffect(() => {
    if (contentRef.current && !originalContentRef.current) {
      const articleElement = contentRef.current.querySelector('article');
      if (articleElement) {
        originalContentRef.current = articleElement.innerHTML;
      }
    }
  }, []);

  // Apply personalized content when it changes
  useEffect(() => {
    if (!contentRef.current) return;

    const articleElement = contentRef.current.querySelector('article');
    if (!articleElement) return;

    if (personalizedHtml) {
      // Convert markdown to HTML and replace content
      articleElement.innerHTML = `<div class="theme-doc-markdown markdown">${convertMarkdownToHtml(personalizedHtml)}</div>`;
    } else if (originalContentRef.current) {
      // Restore original content
      articleElement.innerHTML = originalContentRef.current;
    }
  }, [personalizedHtml]);

  // Handle content change from PersonalizeButton
  const handleContentChange = (content: string, level: ComplexityLevel) => {
    if (level === 'intermediate') {
      setPersonalizedHtml(null); // Restore original content
    } else {
      setPersonalizedHtml(content);
    }
  };

  // Simple markdown to HTML conversion (basic implementation)
  const convertMarkdownToHtml = (markdown: string): string => {
    // This is a simplified converter. In production, you'd use a library.
    let html = markdown;

    // Headers
    html = html.replace(/^### (.*$)/gim, '<h3>$1</h3>');
    html = html.replace(/^## (.*$)/gim, '<h2>$1</h2>');
    html = html.replace(/^# (.*$)/gim, '<h1>$1</h1>');

    // Bold
    html = html.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');

    // Italic
    html = html.replace(/\*(.*?)\*/g, '<em>$1</em>');

    // Code blocks
    html = html.replace(/```(\w+)?\n([\s\S]*?)```/g, '<pre><code>$2</code></pre>');

    // Inline code
    html = html.replace(/`([^`]+)`/g, '<code>$1</code>');

    // Links
    html = html.replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2">$1</a>');

    // Paragraphs
    html = html.replace(/\n\n/g, '</p><p>');
    html = '<p>' + html + '</p>';

    return html;
  };

  // Only show PersonalizeButton on chapter pages
  const isChapterPage = chapterId !== null;

  return (
    <div ref={contentRef}>
      {isChapterPage && (
        <PersonalizeButton
          chapterId={chapterId}
          onContentChange={handleContentChange}
          originalContent=""
        />
      )}
      <OriginalDocItemLayout {...props} />
    </div>
  );
}
