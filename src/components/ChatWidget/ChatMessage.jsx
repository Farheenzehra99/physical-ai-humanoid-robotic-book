/**
 * ChatMessage component - Individual message display
 * Feature: 005-frontend-chat
 */

import React from 'react';
import Citation from './Citation';
import styles from './styles.module.css';

/**
 * Renders an individual chat message (user or assistant)
 * @param {Object} props
 * @param {Object} props.message - Message object with role, content, etc.
 */
export default function ChatMessage({ message }) {
  const { role, content, citations, selectedContext, isError } = message;

  const messageClasses = [
    styles.message,
    styles[role],
    isError ? styles.error : '',
  ].filter(Boolean).join(' ');

  return (
    <div className={messageClasses}>
      <div className={styles.content}>{content}</div>

      {/* Show selected context for user messages */}
      {role === 'user' && selectedContext && (
        <div className={styles.userContext}>
          Context: "{selectedContext.substring(0, 100)}..."
        </div>
      )}

      {/* Show citations for assistant messages - only when sources exist */}
      {role === 'assistant' && citations && citations.length > 0 && (
        <div className={styles.citations}>
          <div className={styles.citationsLabel}>Sources:</div>
          {citations.map((cite, index) => (
            <Citation key={index} citation={cite} />
          ))}
        </div>
      )}
    </div>
  );
}
