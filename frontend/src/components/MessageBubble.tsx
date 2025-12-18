/**
 * MessageBubble component for displaying chat messages.
 *
 * Renders user and assistant messages with appropriate styling and source citations.
 */

import React from 'react';
import styles from './MessageBubble.module.css';

interface Source {
  title: string;
  heading: string;
  score: number;
}

interface MessageBubbleProps {
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: Source[];
}

export function MessageBubble({ role, content, timestamp, sources }: MessageBubbleProps) {
  const isUser = role === 'user';

  return (
    <div className={`${styles.messageContainer} ${isUser ? styles.userMessage : styles.assistantMessage}`}>
      <div className={styles.messageBubble}>
        <div className={styles.messageContent}>
          {content}
        </div>

        {sources && sources.length > 0 && (
          <div className={styles.sources}>
            <div className={styles.sourcesLabel}>Sources:</div>
            {sources.map((source, index) => (
              <div key={index} className={styles.source}>
                <span className={styles.sourceIcon}>📄</span>
                <span className={styles.sourceTitle}>
                  {source.title}
                  {source.heading && ` - ${source.heading}`}
                </span>
                <span className={styles.sourceScore}>
                  ({Math.round(source.score * 100)}% match)
                </span>
              </div>
            ))}
          </div>
        )}

        <div className={styles.timestamp}>
          {timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
        </div>
      </div>
    </div>
  );
}
