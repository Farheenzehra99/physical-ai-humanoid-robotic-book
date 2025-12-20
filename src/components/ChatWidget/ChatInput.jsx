/**
 * ChatInput component - Input field and send button for chat
 * Feature: 005-frontend-chat
 */

import React from 'react';
import styles from './styles.module.css';

/**
 * Chat input component with text field and send button
 * @param {Object} props
 * @param {string} props.value - Current input value
 * @param {Function} props.onChange - Input change handler
 * @param {Function} props.onSubmit - Form submit handler
 * @param {boolean} props.isLoading - Whether a message is being sent
 * @param {boolean} props.disabled - Whether input should be disabled
 */
export default function ChatInput({ value, onChange, onSubmit, isLoading, disabled }) {
  const handleSubmit = (e) => {
    e.preventDefault();
    if (!value.trim() || isLoading || disabled) return;
    onSubmit(e);
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      handleSubmit(e);
    }
  };

  const isDisabled = isLoading || disabled;
  const isSendDisabled = isDisabled || !value.trim();

  return (
    <form onSubmit={handleSubmit} className={styles.inputArea}>
      <input
        type="text"
        value={value}
        onChange={(e) => onChange(e.target.value)}
        onKeyDown={handleKeyDown}
        placeholder="Ask about the book..."
        disabled={isDisabled}
        aria-label="Type your question"
      />
      <button
        type="submit"
        disabled={isSendDisabled}
        aria-label="Send message"
      >
        Send
      </button>
    </form>
  );
}
