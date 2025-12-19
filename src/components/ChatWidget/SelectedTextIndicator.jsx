/**
 * SelectedTextIndicator component - Shows when text is captured for context
 * Feature: 005-frontend-chat
 */

import React from 'react';
import styles from './styles.module.css';

/**
 * Indicator showing selected text will be used as context
 * @param {Object} props
 * @param {string} props.selectedText - The selected text
 * @param {Function} props.onClear - Handler to clear the selection
 */
export default function SelectedTextIndicator({ selectedText, onClear }) {
  if (!selectedText) return null;

  // Truncate for display
  const displayText = selectedText.length > 50
    ? `${selectedText.substring(0, 50)}...`
    : selectedText;

  return (
    <div className={styles.selectedContext}>
      <span className={styles.selectedContextText}>
        Context: "{displayText}"
      </span>
      {onClear && (
        <button
          className={styles.clearSelection}
          onClick={onClear}
          aria-label="Clear selected text"
          title="Clear selection"
        >
          ×
        </button>
      )}
    </div>
  );
}
