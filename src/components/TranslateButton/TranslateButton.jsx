/**
 * TranslateButton component for chapter translation
 * Feature: 008-urdu-translation
 */

import React from 'react';
import { useAuth } from '../../hooks/useAuth';
import styles from './TranslateButton.module.css';

export default function TranslateButton({
  isTranslated,
  isLoading,
  error,
  translationSummary,
  onTranslate,
  onReset,
  onRetry,
  onToggle
}) {
  const { isAuthenticated, user } = useAuth();

  // Unauthenticated state
  if (!isAuthenticated) {
    return (
      <div className={styles.container} role="region" aria-label="Content translation">
        <a
          href="/auth/signin"
          className={styles.signInButton}
          aria-label="Sign in to translate content"
        >
          <span className={styles.icon} aria-hidden="true">🔒</span>
          Sign in to translate
        </a>
        <p className={styles.hint} id="translate-hint">
          Create an account to get content translated to Urdu
        </p>
      </div>
    );
  }

  // Error state
  if (error) {
    return (
      <div className={styles.container} role="region" aria-label="Content translation">
        <button
          className={styles.errorButton}
          onClick={onRetry}
          onKeyDown={(e) => {
            if (e.key === 'Enter' || e.key === ' ') {
              e.preventDefault();
              onRetry();
            }
          }}
          aria-describedby="translate-hint"
          aria-label="Retry translation"
        >
          <span className={styles.icon} aria-hidden="true">⚠️</span>
          Translation failed · Retry
        </button>
        <p className={styles.errorHint} id="translate-error">{error}</p>
      </div>
    );
  }

  // Loading state
  if (isLoading) {
    return (
      <div className={styles.container} role="region" aria-label="Content translation">
        <button
          className={styles.loadingButton}
          disabled
          aria-busy="true"
          aria-label="Translating content"
        >
          <span className={styles.spinner} aria-hidden="true"></span>
          Translating to Urdu...
        </button>
        <p className={styles.hint} id="translate-hint">
          Converting content to Urdu...
        </p>
      </div>
    );
  }

  // Translated state
  if (isTranslated) {
    return (
      <div className={styles.container} role="region" aria-label="Content translation">
        <div className={styles.translatedHeader}>
          <button
            className={styles.resetButton}
            onClick={onToggle}
            onKeyDown={(e) => {
              if (e.key === 'Enter' || e.key === ' ') {
                e.preventDefault();
                onToggle();
              }
            }}
            aria-label="Toggle between original and translated content"
          >
            <span className={styles.icon} aria-hidden="true">🔄</span>
            Toggle Original/Urdu
          </button>
          <span
            className={styles.badge}
            role="status"
            aria-live="polite"
          >
            <span className={styles.icon} aria-hidden="true">✓</span>
            Translated to Urdu
          </span>
        </div>
        {translationSummary && (
          <p className={styles.hint} id="translate-summary">
            Content translated in: {Math.round(translationSummary.processing_time_ms / 1000)}s
          </p>
        )}
      </div>
    );
  }

  // Idle state (default)
  return (
    <div className={styles.container} role="region" aria-label="Content translation">
      <button
        className={styles.translateButton}
        onClick={onToggle}
        onKeyDown={(e) => {
          if (e.key === 'Enter' || e.key === ' ') {
            e.preventDefault();
            onToggle();
          }
        }}
        aria-describedby="translate-hint"
        aria-label="Translate content to Urdu"
      >
        <span className={styles.icon} aria-hidden="true">🌐</span>
        Translate to Urdu
      </button>
      <p className={styles.hint} id="translate-hint">
        Translates this chapter to Urdu while preserving code
      </p>
    </div>
  );
}