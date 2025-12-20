# Quickstart: Frontend Chat Integration

**Feature**: 005-frontend-chat
**Estimated Setup Time**: 15 minutes

## Prerequisites

- Node.js 18+ installed
- Backend API running (Spec-004) at `http://localhost:8000`
- Existing Docusaurus site with `npm install` completed

## Quick Setup

### 1. Configure Environment

Add the API URL to your Docusaurus config:

```javascript
// docusaurus.config.js
module.exports = {
  // ... existing config
  customFields: {
    chatApiUrl: process.env.CHAT_API_URL || 'http://localhost:8000',
  },
};
```

### 2. Create Directory Structure

```bash
mkdir -p src/components/ChatWidget
mkdir -p src/services
mkdir -p src/hooks
mkdir -p src/utils
mkdir -p src/theme
```

### 3. Create the ChatWidget Component

Create `src/components/ChatWidget/index.js`:

```javascript
export { default } from './ChatWidget';
```

Create `src/components/ChatWidget/ChatWidget.jsx`:

```jsx
import React, { useState, useEffect, useRef } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { sendChatMessage } from '@site/src/services/chatApi';
import { useTextSelection } from '@site/src/hooks/useTextSelection';
import { loadChatHistory, saveChatHistory } from '@site/src/utils/sessionStorage';
import styles from './styles.module.css';

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const selectedText = useTextSelection();
  const messagesEndRef = useRef(null);

  // Load chat history on mount
  useEffect(() => {
    setMessages(loadChatHistory());
  }, []);

  // Save chat history when messages change
  useEffect(() => {
    if (messages.length > 0) {
      saveChatHistory(messages);
    }
  }, [messages]);

  // Auto-scroll to bottom
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    const question = input.trim();
    if (!question || isLoading) return;

    // Add user message
    const userMessage = {
      id: `user-${Date.now()}`,
      role: 'user',
      content: question,
      timestamp: Date.now(),
      selectedContext: selectedText || null,
    };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const response = await sendChatMessage(question, selectedText || null);
      const assistantMessage = {
        id: `assistant-${Date.now()}`,
        role: 'assistant',
        content: response.answer,
        timestamp: Date.now(),
        citations: response.citations,
      };
      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
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
  };

  return (
    <div className={styles.chatWidget}>
      {!isOpen ? (
        <button
          className={styles.toggleButton}
          onClick={() => setIsOpen(true)}
          aria-label="Open chat"
        >
          <span className={styles.chatIcon}>💬</span>
          <span>Ask a Question</span>
        </button>
      ) : (
        <div className={styles.chatPanel} role="dialog" aria-label="Chat assistant">
          <div className={styles.header}>
            <span>Book Assistant</span>
            <button onClick={() => setIsOpen(false)} aria-label="Close chat">×</button>
          </div>

          {selectedText && (
            <div className={styles.selectedContext}>
              <small>Selected text will be included as context</small>
            </div>
          )}

          <div className={styles.messages}>
            {messages.map(msg => (
              <div
                key={msg.id}
                className={`${styles.message} ${styles[msg.role]} ${msg.isError ? styles.error : ''}`}
              >
                <div className={styles.content}>{msg.content}</div>
                {msg.citations?.length > 0 && (
                  <div className={styles.citations}>
                    <small>Sources:</small>
                    {msg.citations.map((cite, i) => (
                      <a
                        key={i}
                        href={cite.source_url}
                        target="_blank"
                        rel="noopener noreferrer"
                        className={styles.citation}
                      >
                        {cite.title}
                      </a>
                    ))}
                  </div>
                )}
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.loading}>Thinking...</div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={handleSubmit} className={styles.inputArea}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Ask about the book..."
              disabled={isLoading}
            />
            <button type="submit" disabled={isLoading || !input.trim()}>
              Send
            </button>
          </form>
        </div>
      )}
    </div>
  );
}
```

### 4. Create API Service

Create `src/services/chatApi.js`:

```javascript
const TIMEOUT_MS = 30000;

export async function sendChatMessage(question, selectedText = null, topK = 5) {
  const apiUrl = getApiUrl();
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), TIMEOUT_MS);

  try {
    const body = { question, top_k: topK };
    if (selectedText) {
      body.selected_text = selectedText;
    }

    const response = await fetch(`${apiUrl}/chat`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
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

function getApiUrl() {
  if (typeof window !== 'undefined' && window.__DOCUSAURUS__) {
    return window.__DOCUSAURUS__.siteConfig.customFields?.chatApiUrl || 'http://localhost:8000';
  }
  return 'http://localhost:8000';
}

function getErrorMessage(status, errorData) {
  const detail = errorData.detail;
  switch (status) {
    case 422: return 'Please enter a valid question.';
    case 429: return 'Please wait a moment before sending another question.';
    case 503: return detail || 'Service temporarily unavailable.';
    case 504: return 'Request timed out. Please try again.';
    default: return detail || 'An error occurred. Please try again.';
  }
}

export class ApiError extends Error {
  constructor(status, message) {
    super(message);
    this.status = status;
    this.name = 'ApiError';
  }
}
```

### 5. Create Hooks and Utils

Create `src/hooks/useTextSelection.js`:

```javascript
import { useState, useEffect, useCallback } from 'react';

export function useTextSelection() {
  const [selectedText, setSelectedText] = useState('');

  const handleSelectionChange = useCallback(() => {
    const selection = window.getSelection();
    const text = selection ? selection.toString().trim() : '';
    // Limit to 5000 chars per backend constraint
    setSelectedText(text.substring(0, 5000));
  }, []);

  useEffect(() => {
    document.addEventListener('mouseup', handleSelectionChange);
    document.addEventListener('touchend', handleSelectionChange);
    return () => {
      document.removeEventListener('mouseup', handleSelectionChange);
      document.removeEventListener('touchend', handleSelectionChange);
    };
  }, [handleSelectionChange]);

  return selectedText;
}
```

Create `src/utils/sessionStorage.js`:

```javascript
const CHAT_HISTORY_KEY = 'chatHistory';

export function loadChatHistory() {
  try {
    const stored = sessionStorage.getItem(CHAT_HISTORY_KEY);
    return stored ? JSON.parse(stored) : [];
  } catch {
    return [];
  }
}

export function saveChatHistory(messages) {
  try {
    sessionStorage.setItem(CHAT_HISTORY_KEY, JSON.stringify(messages));
  } catch (e) {
    console.warn('Failed to save chat history:', e);
  }
}

export function clearChatHistory() {
  sessionStorage.removeItem(CHAT_HISTORY_KEY);
}
```

### 6. Create Theme Wrapper

Create `src/theme/Root.js` to inject the chat widget globally:

```javascript
import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
```

### 7. Add Styles

Create `src/components/ChatWidget/styles.module.css`:

```css
.chatWidget {
  position: fixed;
  bottom: 20px;
  right: 20px;
  z-index: 1000;
  font-family: var(--ifm-font-family-base);
}

.toggleButton {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 12px 20px;
  background: var(--ifm-color-primary);
  color: white;
  border: none;
  border-radius: 24px;
  cursor: pointer;
  font-size: 14px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.2);
  transition: transform 0.2s, box-shadow 0.2s;
}

.toggleButton:hover {
  transform: translateY(-2px);
  box-shadow: 0 6px 16px rgba(0, 0, 0, 0.25);
}

.chatIcon {
  font-size: 18px;
}

.chatPanel {
  width: 350px;
  max-height: 500px;
  display: flex;
  flex-direction: column;
  background: var(--ifm-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 12px;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.15);
  overflow: hidden;
}

.header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 16px;
  background: var(--ifm-color-primary);
  color: white;
  font-weight: 600;
}

.header button {
  background: none;
  border: none;
  color: white;
  font-size: 24px;
  cursor: pointer;
  line-height: 1;
}

.selectedContext {
  padding: 8px 16px;
  background: var(--ifm-color-emphasis-100);
  color: var(--ifm-color-emphasis-700);
  border-bottom: 1px solid var(--ifm-color-emphasis-200);
}

.messages {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
  min-height: 200px;
  max-height: 300px;
}

.message {
  margin-bottom: 12px;
  padding: 10px 14px;
  border-radius: 12px;
  max-width: 90%;
}

.user {
  background: var(--ifm-color-primary);
  color: white;
  margin-left: auto;
}

.assistant {
  background: var(--ifm-color-emphasis-200);
  color: var(--ifm-font-color-base);
}

.error {
  background: var(--ifm-color-danger-lightest);
  color: var(--ifm-color-danger-darkest);
}

.content {
  white-space: pre-wrap;
  word-break: break-word;
}

.citations {
  margin-top: 8px;
  padding-top: 8px;
  border-top: 1px solid var(--ifm-color-emphasis-300);
  font-size: 12px;
}

.citation {
  display: block;
  color: var(--ifm-color-primary);
  text-decoration: none;
  margin-top: 4px;
}

.citation:hover {
  text-decoration: underline;
}

.loading {
  color: var(--ifm-color-emphasis-600);
  font-style: italic;
}

.inputArea {
  display: flex;
  gap: 8px;
  padding: 12px;
  border-top: 1px solid var(--ifm-color-emphasis-200);
}

.inputArea input {
  flex: 1;
  padding: 10px 14px;
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 8px;
  font-size: 14px;
  background: var(--ifm-background-color);
  color: var(--ifm-font-color-base);
}

.inputArea input:focus {
  outline: none;
  border-color: var(--ifm-color-primary);
}

.inputArea button {
  padding: 10px 16px;
  background: var(--ifm-color-primary);
  color: white;
  border: none;
  border-radius: 8px;
  cursor: pointer;
  font-weight: 500;
}

.inputArea button:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

@media (max-width: 480px) {
  .chatWidget {
    bottom: 10px;
    right: 10px;
    left: 10px;
  }

  .chatPanel {
    width: 100%;
  }
}
```

### 8. Verify Setup

```bash
# Start the backend (in another terminal)
cd backend && python -m uvicorn src.api.app:app --reload

# Start Docusaurus dev server
npm start
```

Open `http://localhost:3000` and you should see the chat widget in the bottom-right corner.

## Testing Checklist

- [ ] Chat widget toggle button appears on all pages
- [ ] Clicking toggle opens the chat panel
- [ ] Typing a question and pressing Enter sends the message
- [ ] Loading indicator appears while waiting for response
- [ ] Response appears with citations
- [ ] Citation links are clickable
- [ ] Error messages display gracefully when backend is down
- [ ] Chat history persists when navigating between pages
- [ ] Chat history clears on new browser session
- [ ] Selected text is captured when highlighting content

## Troubleshooting

**Chat widget not appearing?**
- Check browser console for errors
- Verify `src/theme/Root.js` exists

**CORS errors?**
- Verify backend CORS config includes `http://localhost:3000`
- Check backend is running

**API connection failed?**
- Verify `customFields.chatApiUrl` in docusaurus.config.js
- Check backend is running at the configured URL
