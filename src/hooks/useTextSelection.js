/**
 * useTextSelection hook - Captures user-selected text from the page
 * Feature: 005-frontend-chat
 */

import { useState, useEffect, useCallback } from 'react';

const MAX_SELECTION_LENGTH = 5000;

/**
 * Custom hook to track text selection on the page
 * @returns {string} Currently selected text (truncated to 5000 chars)
 */
export function useTextSelection() {
  const [selectedText, setSelectedText] = useState('');

  const handleSelectionChange = useCallback(() => {
    const selection = window.getSelection();
    const text = selection ? selection.toString().trim() : '';
    // Truncate to backend constraint (5000 chars)
    setSelectedText(text.substring(0, MAX_SELECTION_LENGTH));
  }, []);

  useEffect(() => {
    // Listen for mouseup and touchend to capture selection
    document.addEventListener('mouseup', handleSelectionChange);
    document.addEventListener('touchend', handleSelectionChange);

    return () => {
      document.removeEventListener('mouseup', handleSelectionChange);
      document.removeEventListener('touchend', handleSelectionChange);
    };
  }, [handleSelectionChange]);

  return selectedText;
}

export default useTextSelection;
