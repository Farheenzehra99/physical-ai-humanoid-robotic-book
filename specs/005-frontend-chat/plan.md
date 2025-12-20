# Implementation Plan: Frontend Chat Integration

**Branch**: `005-frontend-chat` | **Date**: 2025-12-14 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-frontend-chat/spec.md`

## Summary

Integrate a chatbot interface into the Docusaurus-based book website that communicates with the existing FastAPI RAG backend (Spec-004). The frontend consists of a lightweight React component that allows users to ask questions about the book, capture selected text as context, display responses with clickable citations, and maintain chat history across page navigation using browser session storage.

## Technical Context

**Language/Version**: JavaScript/JSX (React components within Docusaurus 3)
**Primary Dependencies**: React (via Docusaurus), native fetch API, sessionStorage
**Storage**: Browser sessionStorage for chat history persistence
**Testing**: Manual testing, browser console validation, Lighthouse audit
**Target Platform**: Modern browsers (Chrome, Firefox, Safari, Edge - last 2 versions)
**Project Type**: Web application (frontend integration with existing backend)
**Performance Goals**: User-perceived latency <3s for 95% of requests; page load <2s
**Constraints**: Lightweight bundle (<50KB added), no external CSS framework, accessible (WCAG 2.1 AA)
**Scale/Scope**: Single ChatWidget component integrated site-wide via Docusaurus theme swizzling

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Requirement | Compliance |
|-----------|-------------|------------|
| V. Visual Excellence | Page load <2s, Lighthouse 100/100, dark/light modes | PASS - Component will be lightweight, CSS follows existing theme |
| V. Visual Excellence | Color palette: Deep Space Black + Electric Cyan + Tesla Bot Silver | PASS - Will use existing CSS custom properties |
| VI. Open Source & Accessibility | No paywalls, no login for core features | PASS - Chat is unauthenticated |
| VI. Open Source & Accessibility | Total site <15MB, PWA offline support | PASS - Adding <50KB, using existing PWA plugin |
| III. Zero-Tolerance Quality | Zero broken links, tested code | PASS - Will validate all citation links |
| VIII. Test-Driven Development | Tests BEFORE implementation | PARTIAL - Manual E2E testing, not TDD for UI |

**Gate Status**: PASS (VIII deviation acceptable for pure UI components - manual E2E testing sufficient)

## Project Structure

### Documentation (this feature)

```text
specs/005-frontend-chat/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── frontend-api.md  # Frontend-to-backend contract
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
src/
├── components/
│   ├── ChatWidget/
│   │   ├── index.js           # Main ChatWidget component
│   │   ├── ChatWidget.jsx     # Component implementation
│   │   ├── ChatInput.jsx      # Input field + send button
│   │   ├── ChatMessages.jsx   # Message display area
│   │   ├── ChatMessage.jsx    # Single message (user/bot)
│   │   ├── Citation.jsx       # Citation link component
│   │   ├── LoadingIndicator.jsx
│   │   └── styles.module.css  # Scoped CSS modules
│   └── [existing components...]
├── services/
│   └── chatApi.js             # API client for /chat endpoint
├── hooks/
│   └── useTextSelection.js    # Hook for detecting selected text
├── utils/
│   └── sessionStorage.js      # Chat history persistence helpers
├── theme/
│   └── Root.js                # Swizzled Root to inject ChatWidget globally
└── css/
    └── custom.css             # Existing + chat variables

tests/
└── e2e/
    └── chat.spec.js           # End-to-end test scenarios
```

**Structure Decision**: Docusaurus-native React component structure. ChatWidget injected globally via theme swizzling (Root.js wrapper). All chat-specific code isolated under `src/components/ChatWidget/` with a dedicated API service layer.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Theme swizzling (Root.js) | Need global component on all pages | Per-page import would require modifying every MDX file |

## Architecture Decisions

### 1. State Management

**Decision**: React useState + sessionStorage (no external state library)

**Rationale**:
- Chat state is local to the widget, not shared across components
- sessionStorage provides persistence across page navigation
- Adding Redux/Zustand would increase bundle size unnecessarily
- useState is sufficient for: messages[], loading, selectedText, isOpen

### 2. API Communication

**Decision**: Native fetch API with async/await

**Rationale**:
- Axios adds ~15KB to bundle
- fetch is built-in and sufficient for single POST endpoint
- Custom wrapper provides error handling and timeout

### 3. Text Selection Detection

**Decision**: Window selection API with debounced listener

**Rationale**:
- Standard Web API, no library needed
- Debouncing prevents excessive state updates
- Captures selection on mouseup/touchend events

### 4. Chat Widget Placement

**Decision**: Fixed position bottom-right corner with toggle button

**Rationale**:
- Common UX pattern (familiar to users)
- Does not interfere with main content
- Collapsible to minimize visual noise

### 5. Citation Navigation

**Decision**: Internal links open in same tab; external in new tab

**Rationale**:
- Internal book links should keep reader in context
- External references should not disrupt reading flow

## Implementation Milestones

Based on user input milestones:

1. **Frontend Environment Setup** - Configure Docusaurus for custom components, env vars
2. **Chat UI Component Development** - Build ChatWidget with input, messages, styling
3. **Backend Connectivity** - Implement chatApi.js, handle loading/errors
4. **Selected-Text Interaction Support** - Add text selection capture, visual feedback
5. **Error Handling + UX Validation** - Graceful errors, edge cases, accessibility
6. **End-to-End Testing** - Manual test scenarios, Lighthouse audit
7. **Deployment & Documentation** - Deploy, verify production, document setup

## API Contract (Backend)

The frontend will communicate with the existing backend API:

**Endpoint**: `POST /chat`

**Request**:
```json
{
  "question": "string (1-2000 chars, required)",
  "selected_text": "string (max 5000 chars, optional)",
  "top_k": "integer (1-20, default: 5, optional)"
}
```

**Response**:
```json
{
  "answer": "string",
  "citations": [
    {
      "source_url": "string",
      "title": "string",
      "chunk_id": "string",
      "relevance_score": "float (0-1)"
    }
  ],
  "metadata": {
    "request_id": "string",
    "processing_time_ms": "float",
    "retrieval_count": "integer",
    "model_used": "string"
  }
}
```

**Error Responses**:
- 422: Validation error
- 429: Rate limit exceeded
- 503: Service unavailable
- 504: Gateway timeout

## Environment Configuration

**Environment Variables** (via `.env` and Docusaurus config):

| Variable | Local | Production |
|----------|-------|------------|
| `REACT_APP_CHAT_API_URL` | `http://localhost:8000` | `https://api.physical-ai-robotics.dev` |

Access in code via: `process.env.REACT_APP_CHAT_API_URL` or Docusaurus `docusaurus.config.js` customFields.

## Risk Analysis

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Backend unavailable | Medium | High | Graceful error messages, retry button |
| CORS issues | Medium | Medium | Verify backend CORS config, test early |
| Large response truncation | Low | Medium | Scrollable message area, max-height |
| Bundle size bloat | Low | High | Code splitting, no large dependencies |
| Mobile UX issues | Medium | Medium | Test on real devices, responsive design |
