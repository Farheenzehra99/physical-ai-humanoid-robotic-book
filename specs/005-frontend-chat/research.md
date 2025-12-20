# Research: Frontend Chat Integration

**Feature**: 005-frontend-chat
**Date**: 2025-12-14
**Status**: Complete

## Research Tasks

### 1. Docusaurus Theme Swizzling for Global Components

**Question**: How to inject a React component on every page in Docusaurus?

**Decision**: Use `src/theme/Root.js` wrapper component

**Rationale**:
- Docusaurus supports "swizzling" to override theme components
- `Root.js` wraps the entire application, making it ideal for global components
- No need to modify MDX files or individual pages

**Implementation**:
```javascript
// src/theme/Root.js
import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
```

**Alternatives Considered**:
- MDX layout wrapper: Rejected - only works for docs, not all pages
- Custom plugin: Rejected - overkill for a single component
- Inject via `clientModules`: Rejected - less React-friendly

**Source**: [Docusaurus Swizzling Guide](https://docusaurus.io/docs/swizzling)

---

### 2. Environment Variables in Docusaurus

**Question**: How to configure API endpoint URL for different environments?

**Decision**: Use `docusaurus.config.js` customFields + runtime detection

**Rationale**:
- Docusaurus does not use Create React App's `REACT_APP_*` convention
- `customFields` in config are available at runtime via `useDocusaurusContext()`
- Can combine with `window.location.hostname` for automatic detection

**Implementation**:
```javascript
// docusaurus.config.js
module.exports = {
  customFields: {
    chatApiUrl: process.env.CHAT_API_URL || 'http://localhost:8000',
  },
};

// In component:
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
const { siteConfig } = useDocusaurusContext();
const apiUrl = siteConfig.customFields.chatApiUrl;
```

**Alternatives Considered**:
- Hardcoded URLs: Rejected - not environment-flexible
- `.env` files with webpack DefinePlugin: Complex setup, Docusaurus handles this

**Source**: [Docusaurus Configuration](https://docusaurus.io/docs/configuration)

---

### 3. Text Selection API

**Question**: How to detect and capture user-selected text on a web page?

**Decision**: Use `window.getSelection()` API with event listeners

**Rationale**:
- Standard Web API, no library needed
- Works across all modern browsers
- Can detect selection on mouseup and touchend events

**Implementation**:
```javascript
// src/hooks/useTextSelection.js
import { useState, useEffect, useCallback } from 'react';

export function useTextSelection() {
  const [selectedText, setSelectedText] = useState('');

  const handleSelectionChange = useCallback(() => {
    const selection = window.getSelection();
    const text = selection ? selection.toString().trim() : '';
    setSelectedText(text);
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

**Edge Cases**:
- Selection in input fields: filter using `selection.anchorNode` check
- Very long selections: truncate to 5000 chars per backend constraint

**Source**: [MDN Web Docs - Selection API](https://developer.mozilla.org/en-US/docs/Web/API/Selection)

---

### 4. Session Storage for Chat History

**Question**: How to persist chat messages across page navigation?

**Decision**: Use `sessionStorage` with JSON serialization

**Rationale**:
- sessionStorage persists for the browser tab session
- Automatically clears on new session (spec requirement)
- No server-side storage needed, privacy-friendly

**Implementation**:
```javascript
// src/utils/sessionStorage.js
const CHAT_HISTORY_KEY = 'chatHistory';

export function saveChatHistory(messages) {
  try {
    sessionStorage.setItem(CHAT_HISTORY_KEY, JSON.stringify(messages));
  } catch (e) {
    console.warn('Failed to save chat history:', e);
  }
}

export function loadChatHistory() {
  try {
    const stored = sessionStorage.getItem(CHAT_HISTORY_KEY);
    return stored ? JSON.parse(stored) : [];
  } catch (e) {
    console.warn('Failed to load chat history:', e);
    return [];
  }
}

export function clearChatHistory() {
  sessionStorage.removeItem(CHAT_HISTORY_KEY);
}
```

**Alternatives Considered**:
- localStorage: Rejected - persists beyond session, not per spec
- IndexedDB: Rejected - overkill for simple message list
- React Context only: Rejected - does not persist across navigation

---

### 5. CSS Modules in Docusaurus

**Question**: Best practice for component-scoped CSS in Docusaurus?

**Decision**: Use CSS Modules (`.module.css` files)

**Rationale**:
- Docusaurus natively supports CSS Modules
- Automatic class name scoping prevents style leakage
- Follows existing project patterns (see `homepage.module.css`)

**Implementation**:
```css
/* src/components/ChatWidget/styles.module.css */
.chatWidget {
  position: fixed;
  bottom: 20px;
  right: 20px;
  z-index: 1000;
}

.chatPanel {
  width: 350px;
  max-height: 500px;
  background: var(--ifm-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 8px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
}
```

**Theme Integration**:
- Use Docusaurus CSS custom properties (`--ifm-*`) for colors
- Respects dark/light mode automatically

---

### 6. Fetch API Error Handling

**Question**: How to handle network errors, timeouts, and HTTP errors?

**Decision**: Custom fetch wrapper with timeout and error classification

**Rationale**:
- Native fetch does not timeout by default
- Need to distinguish network errors from HTTP errors
- User-friendly messages for each error type

**Implementation**:
```javascript
// src/services/chatApi.js
const DEFAULT_TIMEOUT = 30000; // 30 seconds

export async function sendChatMessage(question, selectedText = null, topK = 5) {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), DEFAULT_TIMEOUT);

  try {
    const response = await fetch(`${getApiUrl()}/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        question,
        selected_text: selectedText || undefined,
        top_k: topK,
      }),
      signal: controller.signal,
    });

    clearTimeout(timeoutId);

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new ApiError(response.status, errorData.detail || 'Request failed');
    }

    return await response.json();
  } catch (error) {
    clearTimeout(timeoutId);

    if (error.name === 'AbortError') {
      throw new ApiError(504, 'Request timed out. Please try again.');
    }
    if (error instanceof ApiError) {
      throw error;
    }
    throw new ApiError(0, 'Unable to connect. Please check your internet connection.');
  }
}

class ApiError extends Error {
  constructor(status, message) {
    super(message);
    this.status = status;
    this.name = 'ApiError';
  }
}
```

---

### 7. Accessibility (WCAG 2.1 AA)

**Question**: What accessibility requirements apply to the chat widget?

**Decision**: Implement keyboard navigation, ARIA attributes, focus management

**Requirements**:
1. **Keyboard Navigation**: Tab to toggle button, Enter to open/close
2. **Focus Management**: Auto-focus input when panel opens
3. **ARIA Labels**: Role="dialog", aria-label on buttons
4. **Screen Reader**: Live region for new messages
5. **Color Contrast**: Use theme colors (already WCAG compliant)

**Implementation**:
```jsx
<button
  aria-label="Toggle chat"
  aria-expanded={isOpen}
  onClick={() => setIsOpen(!isOpen)}
>
  Chat
</button>

<div
  role="dialog"
  aria-label="Chat with book assistant"
  aria-hidden={!isOpen}
>
  <div role="log" aria-live="polite">
    {messages.map(...)}
  </div>
</div>
```

**Source**: [WCAG 2.1 Guidelines](https://www.w3.org/WAI/WCAG21/quickref/)

---

### 8. CORS Configuration

**Question**: Will the frontend be able to communicate with the backend?

**Decision**: Backend already configured for CORS (verify during integration)

**Verification Required**:
- Check `backend/src/api/app.py` for CORSMiddleware
- Ensure `allow_origins` includes frontend domain
- Test preflight OPTIONS requests

**Expected Backend Config**:
```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://physical-ai-robotics.dev", "http://localhost:3000"],
    allow_methods=["POST", "OPTIONS"],
    allow_headers=["Content-Type"],
)
```

---

## Summary

All research tasks completed. No NEEDS CLARIFICATION items remain.

| Topic | Decision | Ready for Implementation |
|-------|----------|-------------------------|
| Global component injection | Theme swizzling (Root.js) | Yes |
| Environment variables | customFields in config | Yes |
| Text selection | window.getSelection() API | Yes |
| Chat persistence | sessionStorage | Yes |
| CSS styling | CSS Modules with theme vars | Yes |
| API communication | fetch with timeout wrapper | Yes |
| Accessibility | ARIA attributes + keyboard nav | Yes |
| CORS | Backend config (verify) | Yes |
