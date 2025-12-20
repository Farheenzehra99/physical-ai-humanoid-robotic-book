/**
 * Hook for chapter translation
 * Feature: 008-urdu-translation
 */

import { useState, useCallback } from 'react';
import { translateChapter, TranslationError } from '../services/translationApi';

const CACHE_PREFIX = 'translated_';

/**
 * Hook for managing chapter translation state
 * @param {string} chapterSlug - URL slug of the chapter
 * @returns {Object} Translation state and methods
 */
export function useChapterTranslation(chapterSlug) {
  const [state, setState] = useState({
    isTranslated: false,
    isLoading: false,
    error: null,
    translatedContent: null,
    translationSummary: null
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
   * Translate the chapter content to Urdu
   */
  const translate = useCallback(async (originalContent, chapterTitle) => {
    // Check cache first
    const cached = getCachedContent();
    if (cached) {
      setState({
        isTranslated: true,
        isLoading: false,
        error: null,
        translatedContent: cached.content,
        translationSummary: cached.summary
      });
      return;
    }

    setState(prev => ({ ...prev, isLoading: true, error: null }));

    try {
      const response = await translateChapter(
        chapterSlug,
        originalContent,
        chapterTitle
      );

      setCachedContent(response.translated_content, response.translation_summary);

      setState({
        isTranslated: true,
        isLoading: false,
        error: null,
        translatedContent: response.translated_content,
        translationSummary: response.translation_summary
      });

    } catch (error) {
      setState(prev => ({
        ...prev,
        isLoading: false,
        error: error.message || 'Translation failed'
      }));
    }
  }, [chapterSlug, getCachedContent, setCachedContent]);

  /**
   * Reset to original content
   */
  const resetToOriginal = useCallback(() => {
    setState({
      isTranslated: false,
      isLoading: false,
      error: null,
      translatedContent: null,
      translationSummary: null
    });
  }, []);

  /**
   * Toggle between original and translated content
   */
  const toggleTranslation = useCallback(async (originalContent, chapterTitle) => {
    if (state.isTranslated) {
      // If currently translated, switch back to original
      resetToOriginal();
    } else {
      // If currently in original state, translate
      await translate(originalContent, chapterTitle);
    }
  }, [state.isTranslated, resetToOriginal, translate]);

  return {
    ...state,
    translate,
    resetToOriginal,
    toggleTranslation,
    clearCache
  };
}