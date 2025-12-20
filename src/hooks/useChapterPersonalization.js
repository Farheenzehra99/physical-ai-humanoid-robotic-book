/**
 * Hook for chapter personalization
 * Feature: 007-chapter-personalization
 */

import { useState, useCallback } from 'react';
import { personalizeChapter, PersonalizationError } from '../services/personalizationApi';

const CACHE_PREFIX = 'personalized_';

/**
 * Hook for managing chapter personalization state
 * @param {string} chapterSlug - URL slug of the chapter
 * @returns {Object} Personalization state and methods
 */
export function useChapterPersonalization(chapterSlug) {
  const [state, setState] = useState({
    isPersonalized: false,
    isLoading: false,
    error: null,
    personalizedContent: null,
    adaptationSummary: null
  });

  /**
   * Get cached content from sessionStorage
   */
  const getCachedContent = useCallback(() => {
    if (typeof window === 'undefined') return null;

    try {
      const cached = sessionStorage.getItem(`${CACHE_PREFIX}${chapterSlug}`);
      if (cached) {
        return JSON.parse(cached);
      }
    } catch (e) {
      console.warn('Cache read error:', e);
    }
    return null;
  }, [chapterSlug]);

  /**
   * Save content to sessionStorage
   */
  const setCachedContent = useCallback((content, summary) => {
    if (typeof window === 'undefined') return;

    try {
      sessionStorage.setItem(`${CACHE_PREFIX}${chapterSlug}`, JSON.stringify({
        content,
        summary,
        timestamp: Date.now()
      }));
    } catch (e) {
      console.warn('Cache write error:', e);
    }
  }, [chapterSlug]);

  /**
   * Clear cached content
   */
  const clearCache = useCallback(() => {
    if (typeof window === 'undefined') return;
    sessionStorage.removeItem(`${CACHE_PREFIX}${chapterSlug}`);
  }, [chapterSlug]);

  /**
   * Personalize the chapter content
   */
  const personalize = useCallback(async (originalContent, chapterTitle) => {
    // Check cache first
    const cached = getCachedContent();
    if (cached) {
      setState({
        isPersonalized: true,
        isLoading: false,
        error: null,
        personalizedContent: cached.content,
        adaptationSummary: cached.summary
      });
      return;
    }

    setState(prev => ({ ...prev, isLoading: true, error: null }));

    try {
      const response = await personalizeChapter(
        chapterSlug,
        originalContent,
        chapterTitle
      );

      setCachedContent(response.personalized_content, response.adaptation_summary);

      setState({
        isPersonalized: true,
        isLoading: false,
        error: null,
        personalizedContent: response.personalized_content,
        adaptationSummary: response.adaptation_summary
      });

    } catch (error) {
      setState(prev => ({
        ...prev,
        isLoading: false,
        error: error.message || 'Personalization failed'
      }));
    }
  }, [chapterSlug, getCachedContent, setCachedContent]);

  /**
   * Reset to original content
   */
  const resetToOriginal = useCallback(() => {
    setState({
      isPersonalized: false,
      isLoading: false,
      error: null,
      personalizedContent: null,
      adaptationSummary: null
    });
  }, []);

  return {
    ...state,
    personalize,
    resetToOriginal,
    clearCache
  };
}