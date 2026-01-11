/**
 * Translation API service
 * Feature: 008-urdu-translation
 */

const TIMEOUT_MS = 60000; // 60s for LLM response

/**
 * Get API URL from Docusaurus config
 */
function getApiUrl() {
  // Use configured API URL from Docusaurus config, fallback to Hugging Face Space
  if (typeof window !== 'undefined' && window.chatApiUrl) {
    return window.chatApiUrl;
  }

  // Fallback to environment variable or default Hugging Face Space URL
  const envUrl = process.env.CHAT_API_URL || 'https://farheenzehra99-ai-book.hf.space';
  return envUrl;
}
/**
 * Translate chapter content to Urdu for the current user
 * @param {string} chapterSlug - URL slug of the chapter
 * @param {string} chapterContent - Original chapter content (Markdown)
 * @param {string} chapterTitle - Chapter title
 * @returns {Promise<Object>} Translated content and metadata
 */
export async function translateChapter(chapterSlug, chapterContent, chapterTitle) {
  const apiUrl = getApiUrl();
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), TIMEOUT_MS);

  try {
    const token = typeof window !== 'undefined'
      ? localStorage.getItem('authToken')
      : null;

    if (!token) {
      throw new TranslationError(401, 'Authentication required');
    }

    const response = await fetch(`${apiUrl}/translate/chapter`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${token}`
      },
      body: JSON.stringify({
        chapter_slug: chapterSlug,
        chapter_content: chapterContent,
        chapter_title: chapterTitle
      }),
      signal: controller.signal
    });

    clearTimeout(timeoutId);

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new TranslationError(
        response.status,
        errorData.detail || 'Translation failed'
      );
    }

    return await response.json();

  } catch (error) {
    clearTimeout(timeoutId);

    if (error.name === 'AbortError') {
      throw new TranslationError(504, 'Request timed out');
    }
    if (error instanceof TranslationError) {
      throw error;
    }
    throw new TranslationError(0, 'Network error');
  }
}

/**
 * Custom error class for translation errors
 */
export class TranslationError extends Error {
  constructor(status, message) {
    super(message);
    this.status = status;
    this.name = 'TranslationError';
  }
}