/**
 * LoadingIndicator component - Shows when waiting for API response
 * Feature: 005-frontend-chat
 */

import React from 'react';
import styles from './styles.module.css';

/**
 * Animated loading indicator for chat responses
 */
export default function LoadingIndicator() {
  return (
    <div className={`${styles.message} ${styles.assistant}`}>
      <div className={styles.loading}>
        Thinking<span className={styles.loadingDots}>...</span>
      </div>
    </div>
  );
}
