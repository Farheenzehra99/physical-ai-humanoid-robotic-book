/**
 * ChatMessages component - Scrollable message container
 * Feature: 005-frontend-chat
 */

import React, { useEffect, useRef } from 'react';
import ChatMessage from './ChatMessage';
import LoadingIndicator from './LoadingIndicator';
import styles from './styles.module.css';

/**
 * Scrollable container for chat messages
 * @param {Object} props
 * @param {Array} props.messages - Array of message objects
 * @param {boolean} props.isLoading - Whether a response is being loaded
 */
export default function ChatMessages({ messages, isLoading }) {
  const messagesEndRef = useRef(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  return (
    <div
      className={styles.messages}
      role="log"
      aria-live="polite"
      aria-label="Chat messages"
    >
      {messages.length === 0 && !isLoading && (
        <div className={styles.emptyState}>
          Ask a question about the Physical AI & Humanoid Robotics book!
        </div>
      )}
      {messages.map((msg) => (
        <ChatMessage key={msg.id} message={msg} />
      ))}
      {isLoading && <LoadingIndicator />}
      <div ref={messagesEndRef} />
    </div>
  );
}
