/**
 * ChatWidget main component - Embedded chat interface for book Q&A
 * Feature: 005-frontend-chat
 */

import React, { useState, useEffect, useRef, useCallback } from 'react';
import { sendChatMessage } from '../../services/chatApi';
import { useTextSelection } from '../../hooks/useTextSelection';
import { loadChatHistory, saveChatHistory, clearChatHistory } from '../../utils/sessionStorage';
import { useAuth } from '../../hooks/useAuth'; // Import auth hook
import ChatInput from './ChatInput';
import ChatMessages from './ChatMessages';
import SelectedTextIndicator from './SelectedTextIndicator';
import styles from './styles.module.css';

/**
 * Main ChatWidget component that provides the chat interface
 */
export default function ChatWidget() {
  // State management
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  // Text selection from page
  const selectedText = useTextSelection();
  const [capturedSelection, setCapturedSelection] = useState('');

  // Auth context
  const { user, profile, isAuthenticated } = useAuth();

  // Refs
  const inputRef = useRef(null);

  // Load chat history from sessionStorage on mount
  useEffect(() => {
    const history = loadChatHistory();
    if (history && history.length > 0) {
      setMessages(history);
    }
  }, []);

  // Save chat history when messages change
  useEffect(() => {
    if (messages.length > 0) {
      saveChatHistory(messages);
    }
  }, [messages]);

  // Focus input when panel opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  // Capture selected text when chat opens
  useEffect(() => {
    if (isOpen && selectedText) {
      setCapturedSelection(selectedText);
    }
  }, [isOpen, selectedText]);

  // Handle keyboard shortcuts
  useEffect(() => {
    const handleKeyDown = (e) => {
      // Escape to close
      if (e.key === 'Escape' && isOpen) {
        setIsOpen(false);
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen]);

  // Handle form submission
  const handleSubmit = useCallback(async (e) => {
    e?.preventDefault();

    const question = input.trim();
    if (!question || isLoading) return;

    // Create user message
    const userMessage = {
      id: `user-${Date.now()}`,
      role: 'user',
      content: question,
      timestamp: Date.now(),
      selectedContext: capturedSelection || null,
    };

    // Add user message and clear input
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      // Call API with question and optional selected text
      const response = await sendChatMessage(question, capturedSelection || null);

      // Create assistant message
      const assistantMessage = {
        id: `assistant-${Date.now()}`,
        role: 'assistant',
        content: response.answer,
        timestamp: Date.now(),
        citations: response.citations || [],
      };

      setMessages(prev => [...prev, assistantMessage]);
      // Clear captured selection after successful submission
      setCapturedSelection('');
    } catch (error) {
      // Create error message
      const errorMessage = {
        id: `error-${Date.now()}`,
        role: 'assistant',
        content: error.message || 'An error occurred. Please try again.',
        timestamp: Date.now(),
        isError: true,
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  }, [input, capturedSelection, isLoading]);

  // Handle clear chat
  const handleClearChat = useCallback(() => {
    setMessages([]);
    clearChatHistory();
  }, []);

  // Handle clear selection
  const handleClearSelection = useCallback(() => {
    setCapturedSelection('');
  }, []);

  // Toggle button (closed state)
  if (!isOpen) {
    return (
      <div className={styles.chatWidget}>
        <button
          className={styles.toggleButton}
          onClick={() => setIsOpen(true)}
          aria-label="Open chat assistant"
          aria-expanded="false"
        >
          <span className={styles.chatIcon}>💬</span>
          <span>Ask a Question</span>
        </button>
      </div>
    );
  }

  // Chat panel (open state)
  return (
    <div className={styles.chatWidget}>
      <div
        className={styles.chatPanel}
        role="dialog"
        aria-label="Book Assistant"
        aria-modal="false"
      >
        {/* Header */}
        <div className={styles.header}>
          <span className={styles.headerTitle}>
            <span>📚</span>
            <span>Book Assistant</span>
          </span>
          <div className={styles.headerButtons}>
            {messages.length > 0 && (
              <button
                onClick={handleClearChat}
                aria-label="Clear chat history"
                title="Clear chat"
              >
                🗑️
              </button>
            )}
            <button
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>
        </div>

        {/* Selected text indicator */}
        <SelectedTextIndicator
          selectedText={capturedSelection}
          onClear={handleClearSelection}
        />

        {/* Messages area */}
        <ChatMessages
          messages={messages}
          isLoading={isLoading}
        />

        {/* Input area */}
        <ChatInput
          value={input}
          onChange={setInput}
          onSubmit={handleSubmit}
          isLoading={isLoading}
          disabled={false}
        />
      </div>
    </div>
  );
}
