/**
 * Session storage utilities for chat history persistence
 * Feature: 005-frontend-chat
 */

const CHAT_HISTORY_KEY = 'chatHistory';

/**
 * Load chat history from sessionStorage
 * @returns {Array} Array of chat messages or empty array
 */
export function loadChatHistory() {
  try {
    const stored = sessionStorage.getItem(CHAT_HISTORY_KEY);
    return stored ? JSON.parse(stored) : [];
  } catch {
    return [];
  }
}

/**
 * Save chat history to sessionStorage
 * @param {Array} messages - Array of chat messages to save
 */
export function saveChatHistory(messages) {
  try {
    sessionStorage.setItem(CHAT_HISTORY_KEY, JSON.stringify(messages));
  } catch (e) {
    console.warn('Failed to save chat history:', e);
  }
}

/**
 * Clear chat history from sessionStorage
 */
export function clearChatHistory() {
  sessionStorage.removeItem(CHAT_HISTORY_KEY);
}
