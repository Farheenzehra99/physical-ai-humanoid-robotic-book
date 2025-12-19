/**
 * PersonalizeButton component for chapter personalization
 * Feature: 007-chapter-personalization
 */

import React from 'react';
import { useAuth } from '../../hooks/useAuth';
import styles from './PersonalizeButton.module.css';

export default function PersonalizeButton({
  isPersonalized,
  isLoading,
  error,
  adaptationSummary,
  onPersonalize,
  onReset,
  onRetry
}) {
  const { isAuthenticated, user } = useAuth();

  // Unauthenticated state
  if (!isAuthenticated) {
    return (
      <div className={styles.container} role="region" aria-label="Content personalization">
        <a
          href="/auth/signin"
          className={styles.signInButton}
          aria-label="Sign in to personalize content"
        >
          <span className={styles.icon} aria-hidden="true">🔒</span>
          Sign in to personalize
        </a>
        <p className={styles.hint} id="personalize-hint">
          Create an account to get content tailored to your level
        </p>
      </div>
    );
  }

  // Error state
  if (error) {
    return (
      <div className={styles.container} role="region" aria-label="Content personalization">
        <button
          className={styles.errorButton}
          onClick={onRetry}
          onKeyDown={(e) => {
            if (e.key === 'Enter' || e.key === ' ') {
              e.preventDefault();
              onRetry();
            }
          }}
          aria-describedby="personalize-hint"
          aria-label="Retry personalization"
        >
          <span className={styles.icon} aria-hidden="true">⚠️</span>
          Personalization failed · Retry
        </button>
        <p className={styles.errorHint} id="personalize-error">{error}</p>
      </div>
    );
  }

  // Loading state
  if (isLoading) {
    return (
      <div className={styles.container} role="region" aria-label="Content personalization">
        <button
          className={styles.loadingButton}
          disabled
          aria-busy="true"
          aria-label="Personalizing content"
        >
          <span className={styles.spinner} aria-hidden="true"></span>
          Personalizing...
        </button>
        <p className={styles.hint} id="personalize-hint">
          Tailoring content for your {user?.profile?.programming_level || 'experience'} level...
        </p>
      </div>
    );
  }

  // Personalized state
  if (isPersonalized) {
    return (
      <div className={styles.container} role="region" aria-label="Content personalization">
        <div className={styles.personalizedHeader}>
          <button
            className={styles.resetButton}
            onClick={onReset}
            onKeyDown={(e) => {
              if (e.key === 'Enter' || e.key === ' ') {
                e.preventDefault();
                onReset();
              }
            }}
            aria-label="View original content"
          >
            <span className={styles.icon} aria-hidden="true">📖</span>
            View Original
          </button>
          <span
            className={styles.badge}
            role="status"
            aria-live="polite"
          >
            <span className={styles.icon} aria-hidden="true">✓</span>
            Personalized for You
          </span>
        </div>
        {adaptationSummary && (
          <p className={styles.hint} id="personalize-summary">
            Content adapted for: {adaptationSummary.level_adjustment} ·
            {Math.round(adaptationSummary.processing_time_ms / 1000)}s to generate
          </p>
        )}
      </div>
    );
  }

  // Idle state (default)
  return (
    <div className={styles.container} role="region" aria-label="Content personalization">
      <button
        className={styles.personalizeButton}
        onClick={onPersonalize}
        onKeyDown={(e) => {
          if (e.key === 'Enter' || e.key === ' ') {
            e.preventDefault();
            onPersonalize();
          }
        }}
        aria-describedby="personalize-hint"
        aria-label="Personalize content for me"
      >
        <span className={styles.icon} aria-hidden="true">✨</span>
        Personalize for Me
      </button>
      <p className={styles.hint} id="personalize-hint">
        Adapts this chapter to your experience level
      </p>
    </div>
  );
}