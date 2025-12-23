/**
 * Chat API service for communicating with the RAG backend
 * Feature: 005-frontend-chat
 */

const TIMEOUT_MS = 60000;

/**
 * Send a chat message to the backend API
 * @param {string} question - The user's question
 * @param {string|null} selectedText - Optional selected text for context
 * @param {number} topK - Number of context chunks to retrieve (default: 5)
 * @returns {Promise<Object>} Response with answer and citations
 */
export async function sendChatMessage(question, selectedText = null, topK = 5) {
  const apiUrl = getApiUrl();
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), TIMEOUT_MS);

  try {
    // Backend expects 'message' field, not 'question'
    const body = { message: question };
    if (selectedText) {
      body.selected_text = selectedText;
    }

    // Get auth token from localStorage if available
    const token = typeof window !== 'undefined' ? localStorage.getItem('authToken') : null;

    const headers = { 'Content-Type': 'application/json' };
    if (token) {
      headers['Authorization'] = `Bearer ${token}`;
    }

    const response = await fetch(`${apiUrl}/chat`, {
      method: 'POST',
      headers: headers,
      credentials: 'include',  // Include cookies for cross-origin requests
      body: JSON.stringify(body),
      signal: controller.signal,
    });

    clearTimeout(timeoutId);

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new ApiError(response.status, getErrorMessage(response.status, errorData));
    }

    return await response.json();
  } catch (error) {
    clearTimeout(timeoutId);
    if (error.name === 'AbortError') {
      throw new ApiError(504, 'Request timed out. Please try again.');
    }
    if (error instanceof ApiError) throw error;
    throw new ApiError(0, 'Unable to connect. Please check your internet connection.');
  }
}

/**
 * Get the API URL from Docusaurus config or fallback
 * @returns {string} The API base URL
 */
export function getApiUrl() {
  if (typeof window !== 'undefined' && window.__DOCUSAURUS__) {
    return window.__DOCUSAURUS__.siteConfig.customFields?.chatApiUrl || 'http://localhost:8000';
  }
  return 'http://localhost:8000';
}

/**
 * Get user-friendly error message based on status code
 * @param {number} status - HTTP status code
 * @param {Object} errorData - Error response data
 * @returns {string} User-friendly error message
 */
export function getErrorMessage(status, errorData) {
  const detail = errorData.detail;
  switch (status) {
    case 422: return 'Please enter a valid question.';
    case 429: return 'Please wait a moment before sending another question.';
    case 503: return detail || 'Service temporarily unavailable.';
    case 504: return 'Request timed out. Please try again.';
    default: return detail || 'An error occurred. Please try again.';
  }
}

/**
 * Custom error class for API errors
 */
export class ApiError extends Error {
  constructor(status, message) {
    super(message);
    this.status = status;
    this.name = 'ApiError';
  }
}
