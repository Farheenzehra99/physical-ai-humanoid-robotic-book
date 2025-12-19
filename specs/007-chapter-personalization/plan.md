# Implementation Plan: Chapter-Level Content Personalization

**Feature ID:** 007-chapter-personalization
**Spec Version:** 1.0.0
**Plan Version:** 1.0.0
**Created:** 2025-12-17

---

## 1. Executive Summary

This plan implements **Chapter-Level Content Personalization** - a "Personalize for Me" button that rewrites chapter content based on the authenticated user's profile (experience level, software/hardware background, learning style).

### Key Deliverables
1. Backend personalization endpoint (`POST /api/v1/personalize/chapter`)
2. LLM prompt with strict preservation rules
3. Frontend `PersonalizeButton` component
4. Docusaurus `DocItem` theme wrapper for chapter injection
5. Session-based caching for personalized content

### Architecture Decision

**Chosen Approach:** Swizzle Docusaurus `DocItem` component to inject personalization button at the chapter level, reusing existing Gemini agent patterns.

**Rationale:**
- Existing `Root.js` already wraps with `AuthProvider` - we can extend this pattern
- Agent personalization injection already works in chat (lines 88-100 in `agent.py`)
- Profile retrieval via `/auth/preferences` endpoint is already optimized for this use case

---

## 2. Technical Architecture

### 2.1 System Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        FRONTEND (Docusaurus)                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────────┐     ┌─────────────────────────────────────────┐   │
│  │  Root.js        │     │  DocItem (Swizzled)                     │   │
│  │  - AuthProvider │     │  - Wraps all /docs/* pages              │   │
│  │  - ChatWidget   │     │  - Injects PersonalizeButton            │   │
│  └─────────────────┘     │  - Manages content state (orig/custom)  │   │
│                          └──────────────────┬──────────────────────┘   │
│                                             │                           │
│  ┌──────────────────────────────────────────┼──────────────────────┐   │
│  │                PersonalizeButton         │                      │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌────┴────────┐             │   │
│  │  │ idle state  │  │ loading     │  │ personalized│             │   │
│  │  │ [Button]    │  │ [Spinner]   │  │ [Toggle]    │             │   │
│  │  └─────────────┘  └─────────────┘  └─────────────┘             │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                             │                           │
│  ┌──────────────────────────────────────────┼──────────────────────┐   │
│  │            useChapterPersonalization Hook                       │   │
│  │  - fetchPersonalizedContent(chapterSlug, rawContent)           │   │
│  │  - getFromCache(chapterSlug)                                   │   │
│  │  - saveToCache(chapterSlug, content)                           │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                             │                           │
└─────────────────────────────────────────────┼───────────────────────────┘
                                              │
                                              ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                        BACKEND (FastAPI)                                │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  POST /api/v1/personalize/chapter                               │   │
│  │  - Auth: Bearer token required                                  │   │
│  │  - Rate limit: 10/min per user                                  │   │
│  │  - Input: { chapter_slug, chapter_content, chapter_title }      │   │
│  │  - Output: { personalized_content, adaptation_summary }         │   │
│  └──────────────────────────────┬──────────────────────────────────┘   │
│                                 │                                       │
│           ┌─────────────────────┼─────────────────────┐                │
│           ▼                     ▼                     ▼                │
│  ┌────────────────┐  ┌──────────────────┐  ┌──────────────────┐       │
│  │ User Profile   │  │ Personalization  │  │ Response Cache   │       │
│  │ Repository     │  │ Service          │  │ (Redis/Memory)   │       │
│  │                │  │                  │  │                  │       │
│  │ get_user_      │  │ - build_prompt() │  │ Key: user_id +   │       │
│  │ preferences()  │  │ - call_llm()     │  │      chapter_slug│       │
│  └────────────────┘  │ - validate()     │  │ TTL: 24 hours    │       │
│                      └────────┬─────────┘  └──────────────────┘       │
│                               │                                        │
│                               ▼                                        │
│                      ┌──────────────────┐                              │
│                      │  Gemini API      │                              │
│                      │  (gemini-2.0-    │                              │
│                      │   flash)         │                              │
│                      └──────────────────┘                              │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Data Flow Sequence

```
User clicks "Personalize for Me"
         │
         ▼
┌─────────────────────────────────────┐
│ 1. Check sessionStorage cache       │
│    Key: personalized_{chapter_slug} │
└─────────────────┬───────────────────┘
                  │
        ┌─────────┴─────────┐
        │                   │
   Cache HIT           Cache MISS
        │                   │
        ▼                   ▼
┌───────────────┐   ┌───────────────────────────────┐
│ Display       │   │ 2. Extract chapter content    │
│ cached content│   │    from DOM/props             │
└───────────────┘   └───────────────┬───────────────┘
                                    │
                                    ▼
                    ┌───────────────────────────────┐
                    │ 3. POST /api/v1/personalize/  │
                    │    chapter                    │
                    │    Headers: Authorization     │
                    │    Body: {slug, content}      │
                    └───────────────┬───────────────┘
                                    │
                                    ▼
                    ┌───────────────────────────────┐
                    │ 4. Backend validates session  │
                    │    Fetches user profile       │
                    └───────────────┬───────────────┘
                                    │
                                    ▼
                    ┌───────────────────────────────┐
                    │ 5. Build LLM prompt:          │
                    │    - System: Adaptation rules │
                    │    - User: Profile context    │
                    │    - Content: Original chapter│
                    └───────────────┬───────────────┘
                                    │
                                    ▼
                    ┌───────────────────────────────┐
                    │ 6. Call Gemini API            │
                    │    Model: gemini-2.0-flash    │
                    │    Temperature: 0.3 (low)     │
                    └───────────────┬───────────────┘
                                    │
                                    ▼
                    ┌───────────────────────────────┐
                    │ 7. Validate response:         │
                    │    - Headings preserved?      │
                    │    - Code blocks intact?      │
                    │    - No script injection?     │
                    └───────────────┬───────────────┘
                                    │
                                    ▼
                    ┌───────────────────────────────┐
                    │ 8. Return personalized content│
                    │    Cache in backend (24h)     │
                    └───────────────┬───────────────┘
                                    │
                                    ▼
                    ┌───────────────────────────────┐
                    │ 9. Frontend caches in         │
                    │    sessionStorage             │
                    └───────────────┬───────────────┘
                                    │
                                    ▼
                    ┌───────────────────────────────┐
                    │ 10. Render personalized MD    │
                    │     via ReactMarkdown         │
                    └───────────────────────────────┘
```

---

## 3. Implementation Phases

### Phase 1: Backend Personalization Service (Days 1-3)

#### 1.1 Create Personalization Models

**File:** `backend/src/personalization/models.py`

```python
from pydantic import BaseModel, Field
from typing import Optional
from enum import Enum

class PersonalizationLevel(str, Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class ChapterPersonalizationRequest(BaseModel):
    """Request model for chapter personalization."""
    chapter_slug: str = Field(
        ...,
        description="URL slug of the chapter",
        example="module-01-foundations/chapter-01/page-01"
    )
    chapter_content: str = Field(
        ...,
        max_length=51200,  # 50KB limit
        description="Original chapter content in Markdown"
    )
    chapter_title: Optional[str] = Field(
        None,
        description="Chapter title for context"
    )

class AdaptationSummary(BaseModel):
    """Summary of adaptations made."""
    level_adjustment: str
    processing_time_ms: int
    content_length_original: int
    content_length_personalized: int

class ChapterPersonalizationResponse(BaseModel):
    """Response model for chapter personalization."""
    success: bool
    personalized_content: str
    adaptation_summary: AdaptationSummary
```

#### 1.2 Create Personalization Service

**File:** `backend/src/personalization/service.py`

```python
import time
import logging
from typing import Optional
import google.generativeai as genai
from pathlib import Path

from ..config import get_settings
from ..database.crud import get_user_preferences

logger = logging.getLogger(__name__)

# Load prompt template
PROMPT_TEMPLATE_PATH = Path(__file__).parent / "personalization_prompt.md"

class PersonalizationService:
    """Service for personalizing chapter content using LLM."""

    def __init__(self):
        self.settings = get_settings()
        genai.configure(api_key=self.settings.gemini_api_key)
        self.model = genai.GenerativeModel(
            model_name=self.settings.gemini_model,
            generation_config={
                "temperature": 0.3,  # Low for consistency
                "max_output_tokens": 16384,
            }
        )
        self.system_prompt = self._load_prompt_template()

    def _load_prompt_template(self) -> str:
        """Load the personalization prompt template."""
        try:
            return PROMPT_TEMPLATE_PATH.read_text(encoding="utf-8")
        except FileNotFoundError:
            logger.warning("Prompt template not found, using default")
            return self._default_prompt()

    async def personalize_chapter(
        self,
        chapter_content: str,
        chapter_title: str,
        user_profile: dict
    ) -> tuple[str, dict]:
        """
        Personalize chapter content for user.

        Returns:
            Tuple of (personalized_content, adaptation_summary)
        """
        start_time = time.time()

        # Build the full prompt
        prompt = self._build_prompt(
            chapter_content=chapter_content,
            chapter_title=chapter_title,
            user_profile=user_profile
        )

        # Call Gemini
        try:
            response = await self.model.generate_content_async(prompt)
            personalized_content = response.text
        except Exception as e:
            logger.error(f"LLM call failed: {e}")
            raise

        # Validate response
        self._validate_response(chapter_content, personalized_content)

        processing_time = int((time.time() - start_time) * 1000)

        adaptation_summary = {
            "level_adjustment": user_profile.get("programming_level", "unknown"),
            "processing_time_ms": processing_time,
            "content_length_original": len(chapter_content),
            "content_length_personalized": len(personalized_content)
        }

        return personalized_content, adaptation_summary

    def _build_prompt(
        self,
        chapter_content: str,
        chapter_title: str,
        user_profile: dict
    ) -> str:
        """Build the full LLM prompt with user context."""
        return self.system_prompt.format(
            programming_level=user_profile.get("programming_level", "Not specified"),
            programming_languages=", ".join(user_profile.get("programming_languages", [])) or "Not specified",
            ai_knowledge_level=user_profile.get("ai_knowledge_level", "Not specified"),
            hardware_experience=user_profile.get("hardware_experience", "Not specified"),
            learning_style=user_profile.get("learning_style", "Not specified"),
            chapter_title=chapter_title or "Untitled",
            chapter_content=chapter_content
        )

    def _validate_response(
        self,
        original: str,
        personalized: str
    ) -> None:
        """Validate that response preserves structure."""
        import re

        # Extract headings from original
        original_headings = re.findall(r'^#{1,4}\s+.+$', original, re.MULTILINE)
        personalized_headings = re.findall(r'^#{1,4}\s+.+$', personalized, re.MULTILINE)

        # Check heading preservation (allow some flexibility)
        if len(personalized_headings) < len(original_headings) * 0.8:
            logger.warning(
                f"Heading count mismatch: {len(original_headings)} -> {len(personalized_headings)}"
            )

        # Check code block preservation
        original_code_blocks = len(re.findall(r'```[\s\S]*?```', original))
        personalized_code_blocks = len(re.findall(r'```[\s\S]*?```', personalized))

        if personalized_code_blocks < original_code_blocks:
            logger.warning(
                f"Code block count reduced: {original_code_blocks} -> {personalized_code_blocks}"
            )

    def _default_prompt(self) -> str:
        """Return default prompt if template file not found."""
        return """You are an expert technical writer..."""  # Abbreviated
```

#### 1.3 Create LLM Prompt Template

**File:** `backend/src/personalization/personalization_prompt.md`

```markdown
## System Instructions

You are an expert technical writer for a robotics and AI textbook. Your task is to adapt chapter content for individual learners based on their background.

### CRITICAL RULES (MUST FOLLOW)

1. **PRESERVE ALL HEADINGS** - Keep every H1, H2, H3, H4 heading with exact text
2. **PRESERVE ALL CODE BLOCKS** - Do not modify any code inside ``` blocks
3. **PRESERVE ALL COMMANDS** - Terminal commands must remain unchanged
4. **PRESERVE SECTION ORDER** - Keep sections in the same sequence
5. **PRESERVE ALL LINKS** - Keep all [text](url) links intact
6. **PRESERVE WARNINGS/TIPS** - Keep all admonition blocks (:::tip, :::warning, etc.)
7. **NO CONTENT REMOVAL** - Do not skip or delete any topics

### WHAT YOU MAY ADAPT

- Explanatory paragraphs (expand for beginners, condense for advanced)
- Analogies and examples (use appropriate complexity)
- Transitional sentences
- Introductory context before technical sections

### ADAPTATION BY EXPERIENCE LEVEL

**BEGINNER ({programming_level}):**
- Add intuition BEFORE technical details
- Use everyday analogies (factory workers, mail delivery, etc.)
- Explain ALL acronyms and jargon on first use
- Add "Why this matters" context paragraphs
- Include step-by-step reasoning

**INTERMEDIATE:**
- Balanced explanations with practical focus
- Use domain-specific analogies (other programming concepts)
- Assume basic programming knowledge
- Focus on "how" and "when to use"

**ADVANCED:**
- Concise, direct language
- Skip basic explanations (e.g., "a variable stores data")
- Use expert terminology freely
- Focus on edge cases, optimizations, and gotchas
- Add performance considerations

### ADAPTATION BY LEARNING STYLE

**{learning_style}:**
- Theory: Lead with concepts, explain "why" before "how"
- Code-first: Start with examples, explain after
- Visual: Emphasize diagrams, add ASCII art where helpful
- Mixed: Balanced approach

### USER PROFILE

- **Programming Level:** {programming_level}
- **Known Languages:** {programming_languages}
- **AI Knowledge:** {ai_knowledge_level}
- **Hardware Experience:** {hardware_experience}
- **Learning Style:** {learning_style}

### CHAPTER TO ADAPT

**Title:** {chapter_title}

---

{chapter_content}

---

### OUTPUT INSTRUCTIONS

Return ONLY the adapted chapter content in Markdown format.
- Do not include meta-commentary
- Do not explain your changes
- Start directly with the first heading
```

#### 1.4 Create Personalization Endpoint

**File:** `backend/src/api/routes/personalize.py`

```python
import logging
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from ...personalization.models import (
    ChapterPersonalizationRequest,
    ChapterPersonalizationResponse,
    AdaptationSummary
)
from ...personalization.service import PersonalizationService
from ...database import get_async_db
from ...database.crud import get_user_preferences
from ...auth.dependencies import get_current_active_user
from ...better_auth_mock import User

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/personalize", tags=["personalization"])

# Singleton service instance
_personalization_service = None

def get_personalization_service() -> PersonalizationService:
    global _personalization_service
    if _personalization_service is None:
        _personalization_service = PersonalizationService()
    return _personalization_service

@router.post("/chapter", response_model=ChapterPersonalizationResponse)
async def personalize_chapter(
    request: ChapterPersonalizationRequest,
    current_user: User = Depends(get_current_active_user),
    db: AsyncSession = Depends(get_async_db),
    service: PersonalizationService = Depends(get_personalization_service)
):
    """
    Personalize chapter content for the authenticated user.

    Adapts the chapter content based on the user's:
    - Programming level (beginner/intermediate/advanced)
    - Known programming languages
    - AI knowledge level
    - Hardware experience
    - Learning style preference
    """
    try:
        # Fetch user preferences
        user_preferences = await get_user_preferences(db, current_user.id)

        if not user_preferences:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="User profile not found. Please complete your profile first."
            )

        # Personalize content
        personalized_content, adaptation_summary = await service.personalize_chapter(
            chapter_content=request.chapter_content,
            chapter_title=request.chapter_title or "",
            user_profile=user_preferences
        )

        return ChapterPersonalizationResponse(
            success=True,
            personalized_content=personalized_content,
            adaptation_summary=AdaptationSummary(**adaptation_summary)
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Personalization failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Personalization service temporarily unavailable"
        )
```

#### 1.5 Register Route in App

**Update:** `backend/src/api/app.py`

```python
# Add import
from .routes.personalize import router as personalize_router

# In create_app(), add:
app.include_router(personalize_router, prefix=settings.api_prefix)
```

---

### Phase 2: Frontend Personalization Hook (Days 4-5)

#### 2.1 Create Personalization API Service

**File:** `src/services/personalizationApi.js`

```javascript
/**
 * Personalization API service
 * Feature: 007-chapter-personalization
 */

const TIMEOUT_MS = 60000; // 60s for LLM response

/**
 * Get API URL from Docusaurus config
 */
function getApiUrl() {
  if (typeof window !== 'undefined' && window.__DOCUSAURUS__) {
    return window.__DOCUSAURUS__.siteConfig.customFields?.chatApiUrl || 'http://localhost:8000';
  }
  return 'http://localhost:8000';
}

/**
 * Personalize chapter content for the current user
 * @param {string} chapterSlug - URL slug of the chapter
 * @param {string} chapterContent - Original chapter content (Markdown)
 * @param {string} chapterTitle - Chapter title
 * @returns {Promise<Object>} Personalized content and metadata
 */
export async function personalizeChapter(chapterSlug, chapterContent, chapterTitle) {
  const apiUrl = getApiUrl();
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), TIMEOUT_MS);

  try {
    const token = typeof window !== 'undefined'
      ? localStorage.getItem('authToken')
      : null;

    if (!token) {
      throw new PersonalizationError(401, 'Authentication required');
    }

    const response = await fetch(`${apiUrl}/api/v1/personalize/chapter`, {
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
      throw new PersonalizationError(
        response.status,
        errorData.detail || 'Personalization failed'
      );
    }

    return await response.json();

  } catch (error) {
    clearTimeout(timeoutId);

    if (error.name === 'AbortError') {
      throw new PersonalizationError(504, 'Request timed out');
    }
    if (error instanceof PersonalizationError) {
      throw error;
    }
    throw new PersonalizationError(0, 'Network error');
  }
}

/**
 * Custom error class for personalization errors
 */
export class PersonalizationError extends Error {
  constructor(status, message) {
    super(message);
    this.status = status;
    this.name = 'PersonalizationError';
  }
}
```

#### 2.2 Create Personalization Hook

**File:** `src/hooks/useChapterPersonalization.js`

```javascript
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
```

---

### Phase 3: PersonalizeButton Component (Days 6-7)

#### 3.1 Create Button Component

**File:** `src/components/PersonalizeButton/PersonalizeButton.jsx`

```jsx
/**
 * PersonalizeButton component for chapter personalization
 * Feature: 007-chapter-personalization
 */

import React from 'react';
import { useAuth } from '../../hooks/useAuth';
import styles from './PersonalizeButton.module.css';

export default function PersonalizeButton({
  isPersonalized,
  isLoading,
  error,
  adaptationSummary,
  onPersonalize,
  onReset,
  onRetry
}) {
  const { isAuthenticated, user } = useAuth();

  // Unauthenticated state
  if (!isAuthenticated) {
    return (
      <div className={styles.container}>
        <a href="/auth/signin" className={styles.signInButton}>
          <span className={styles.icon}>🔒</span>
          Sign in to personalize
        </a>
        <p className={styles.hint}>
          Create an account to get content tailored to your level
        </p>
      </div>
    );
  }

  // Error state
  if (error) {
    return (
      <div className={styles.container}>
        <button
          className={styles.errorButton}
          onClick={onRetry}
        >
          <span className={styles.icon}>⚠️</span>
          Personalization failed · Retry
        </button>
        <p className={styles.errorHint}>{error}</p>
      </div>
    );
  }

  // Loading state
  if (isLoading) {
    return (
      <div className={styles.container}>
        <button className={styles.loadingButton} disabled>
          <span className={styles.spinner}></span>
          Personalizing...
        </button>
        <p className={styles.hint}>
          Tailoring content for your {user?.profile?.programming_level || 'experience'} level...
        </p>
      </div>
    );
  }

  // Personalized state
  if (isPersonalized) {
    return (
      <div className={styles.container}>
        <div className={styles.personalizedHeader}>
          <button
            className={styles.resetButton}
            onClick={onReset}
          >
            <span className={styles.icon}>📖</span>
            View Original
          </button>
          <span className={styles.badge}>
            <span className={styles.icon}>✓</span>
            Personalized for You
          </span>
        </div>
        {adaptationSummary && (
          <p className={styles.hint}>
            Content adapted for: {adaptationSummary.level_adjustment} ·
            {Math.round(adaptationSummary.processing_time_ms / 1000)}s to generate
          </p>
        )}
      </div>
    );
  }

  // Idle state (default)
  return (
    <div className={styles.container}>
      <button
        className={styles.personalizeButton}
        onClick={onPersonalize}
      >
        <span className={styles.icon}>✨</span>
        Personalize for Me
      </button>
      <p className={styles.hint}>
        Adapts this chapter to your experience level
      </p>
    </div>
  );
}
```

#### 3.2 Create Button Styles

**File:** `src/components/PersonalizeButton/PersonalizeButton.module.css`

```css
.container {
  margin: 1.5rem 0 2rem 0;
  padding: 1rem;
  background: linear-gradient(135deg, rgba(0, 217, 255, 0.05) 0%, rgba(138, 43, 226, 0.05) 100%);
  border: 1px solid rgba(0, 217, 255, 0.2);
  border-radius: 12px;
}

.personalizeButton,
.resetButton,
.signInButton,
.loadingButton,
.errorButton {
  display: inline-flex;
  align-items: center;
  gap: 0.5rem;
  padding: 0.75rem 1.5rem;
  font-size: 1rem;
  font-weight: 600;
  border-radius: 8px;
  cursor: pointer;
  transition: all 0.2s ease;
  text-decoration: none;
}

.personalizeButton {
  background: linear-gradient(135deg, #00D9FF 0%, #0099cc 100%);
  color: #0a0a1a;
  border: none;
}

.personalizeButton:hover {
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(0, 217, 255, 0.3);
}

.resetButton {
  background: transparent;
  color: var(--ifm-color-primary);
  border: 2px solid var(--ifm-color-primary);
}

.resetButton:hover {
  background: var(--ifm-color-primary);
  color: white;
}

.signInButton {
  background: rgba(0, 217, 255, 0.1);
  color: #00D9FF;
  border: 1px solid rgba(0, 217, 255, 0.3);
}

.signInButton:hover {
  background: rgba(0, 217, 255, 0.2);
}

.loadingButton {
  background: #e0e0e0;
  color: #666;
  border: none;
  cursor: not-allowed;
}

.errorButton {
  background: #fff3cd;
  color: #856404;
  border: 1px solid #ffc107;
}

.errorButton:hover {
  background: #ffc107;
  color: #000;
}

.icon {
  font-size: 1.1rem;
}

.spinner {
  width: 16px;
  height: 16px;
  border: 2px solid #ccc;
  border-top-color: #666;
  border-radius: 50%;
  animation: spin 0.8s linear infinite;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}

.hint {
  margin: 0.5rem 0 0 0;
  font-size: 0.875rem;
  color: var(--ifm-color-emphasis-600);
}

.errorHint {
  margin: 0.5rem 0 0 0;
  font-size: 0.875rem;
  color: #856404;
}

.personalizedHeader {
  display: flex;
  align-items: center;
  gap: 1rem;
  flex-wrap: wrap;
}

.badge {
  display: inline-flex;
  align-items: center;
  gap: 0.25rem;
  padding: 0.5rem 1rem;
  background: rgba(40, 167, 69, 0.1);
  color: #28a745;
  border-radius: 20px;
  font-size: 0.875rem;
  font-weight: 500;
}

/* Dark mode */
[data-theme='dark'] .container {
  background: linear-gradient(135deg, rgba(0, 217, 255, 0.08) 0%, rgba(138, 43, 226, 0.08) 100%);
}

[data-theme='dark'] .hint {
  color: var(--ifm-color-emphasis-500);
}
```

#### 3.3 Create Index Export

**File:** `src/components/PersonalizeButton/index.js`

```javascript
export { default } from './PersonalizeButton';
```

---

### Phase 4: Docusaurus DocItem Integration (Days 8-9)

#### 4.1 Swizzle DocItem Component

**File:** `src/theme/DocItem/index.js`

```jsx
/**
 * Swizzled DocItem component with personalization support
 * Feature: 007-chapter-personalization
 */

import React, { useState, useCallback, useMemo } from 'react';
import DocItem from '@theme-original/DocItem';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import { useLocation } from '@docusaurus/router';
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import { useChapterPersonalization } from '@site/src/hooks/useChapterPersonalization';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import styles from './styles.module.css';

export default function DocItemWrapper(props) {
  const { metadata, contentTitle } = useDoc();
  const location = useLocation();

  // Extract chapter slug from pathname
  const chapterSlug = useMemo(() => {
    return location.pathname.replace(/^\/docs\//, '').replace(/\/$/, '');
  }, [location.pathname]);

  // Personalization state
  const {
    isPersonalized,
    isLoading,
    error,
    personalizedContent,
    adaptationSummary,
    personalize,
    resetToOriginal
  } = useChapterPersonalization(chapterSlug);

  // Track original content
  const [originalContent, setOriginalContent] = useState(null);

  // Extract content from DOM when personalizing
  const handlePersonalize = useCallback(() => {
    // Get the markdown content from the doc
    // Note: This extracts from the rendered content
    const docContent = document.querySelector('.theme-doc-markdown');
    if (docContent) {
      // For now, we'll pass the title and let backend fetch content
      // In production, consider server-side content extraction
      const rawContent = extractMarkdownFromDOM(docContent);
      setOriginalContent(rawContent);
      personalize(rawContent, contentTitle || metadata.title);
    }
  }, [personalize, contentTitle, metadata.title]);

  const handleRetry = useCallback(() => {
    if (originalContent) {
      personalize(originalContent, contentTitle || metadata.title);
    }
  }, [personalize, originalContent, contentTitle, metadata.title]);

  return (
    <>
      {/* Personalization Button - Injected before content */}
      <div className={styles.personalizationWrapper}>
        <PersonalizeButton
          isPersonalized={isPersonalized}
          isLoading={isLoading}
          error={error}
          adaptationSummary={adaptationSummary}
          onPersonalize={handlePersonalize}
          onReset={resetToOriginal}
          onRetry={handleRetry}
        />
      </div>

      {/* Conditional Content Rendering */}
      {isPersonalized && personalizedContent ? (
        <div className={styles.personalizedContent}>
          <ReactMarkdown
            remarkPlugins={[remarkGfm]}
            components={{
              // Custom renderers for code blocks, etc.
              code: ({ node, inline, className, children, ...props }) => {
                const match = /language-(\w+)/.exec(className || '');
                return !inline ? (
                  <pre className={className}>
                    <code className={className} {...props}>
                      {children}
                    </code>
                  </pre>
                ) : (
                  <code className={className} {...props}>
                    {children}
                  </code>
                );
              }
            }}
          >
            {personalizedContent}
          </ReactMarkdown>
        </div>
      ) : (
        <DocItem {...props} />
      )}
    </>
  );
}

/**
 * Extract markdown-like content from DOM
 * This is a simplified extraction - production should use source files
 */
function extractMarkdownFromDOM(element) {
  // Clone to avoid modifying the actual DOM
  const clone = element.cloneNode(true);

  // Remove elements we don't want
  clone.querySelectorAll('script, style, .hash-link').forEach(el => el.remove());

  // Convert to text with basic markdown structure
  let markdown = '';

  clone.querySelectorAll('h1, h2, h3, h4, h5, h6, p, pre, ul, ol, blockquote').forEach(el => {
    const tag = el.tagName.toLowerCase();

    if (tag.startsWith('h')) {
      const level = parseInt(tag[1]);
      markdown += '#'.repeat(level) + ' ' + el.textContent.trim() + '\n\n';
    } else if (tag === 'p') {
      markdown += el.textContent.trim() + '\n\n';
    } else if (tag === 'pre') {
      const code = el.querySelector('code');
      const lang = code?.className?.match(/language-(\w+)/)?.[1] || '';
      markdown += '```' + lang + '\n' + (code?.textContent || el.textContent) + '\n```\n\n';
    } else if (tag === 'ul' || tag === 'ol') {
      el.querySelectorAll('li').forEach((li, i) => {
        const prefix = tag === 'ol' ? `${i + 1}. ` : '- ';
        markdown += prefix + li.textContent.trim() + '\n';
      });
      markdown += '\n';
    } else if (tag === 'blockquote') {
      markdown += '> ' + el.textContent.trim().split('\n').join('\n> ') + '\n\n';
    }
  });

  return markdown.trim();
}
```

#### 4.2 DocItem Styles

**File:** `src/theme/DocItem/styles.module.css`

```css
.personalizationWrapper {
  max-width: var(--ifm-container-width);
  margin: 0 auto;
  padding: 0 var(--ifm-spacing-horizontal);
}

.personalizedContent {
  /* Match Docusaurus markdown styling */
  max-width: var(--ifm-container-width);
  margin: 0 auto;
  padding: 0 var(--ifm-spacing-horizontal);
}

.personalizedContent h1,
.personalizedContent h2,
.personalizedContent h3,
.personalizedContent h4 {
  margin-top: 2rem;
  margin-bottom: 1rem;
}

.personalizedContent p {
  margin-bottom: 1rem;
  line-height: 1.7;
}

.personalizedContent pre {
  margin: 1.5rem 0;
  padding: 1rem;
  background: var(--ifm-pre-background);
  border-radius: var(--ifm-pre-border-radius);
  overflow-x: auto;
}

.personalizedContent code {
  font-family: var(--ifm-font-family-monospace);
  font-size: var(--ifm-code-font-size);
}

.personalizedContent ul,
.personalizedContent ol {
  margin-bottom: 1rem;
  padding-left: 2rem;
}

.personalizedContent li {
  margin-bottom: 0.5rem;
}

.personalizedContent blockquote {
  margin: 1rem 0;
  padding: 0.5rem 1rem;
  border-left: 4px solid var(--ifm-color-primary);
  background: var(--ifm-color-emphasis-100);
}

/* Smooth transition between original and personalized */
.personalizedContent {
  animation: fadeIn 0.3s ease-in-out;
}

@keyframes fadeIn {
  from { opacity: 0; }
  to { opacity: 1; }
}
```

---

### Phase 5: Testing & Validation (Days 10-12)

#### 5.1 Backend Unit Tests

**File:** `backend/tests/test_personalization.py`

```python
import pytest
from unittest.mock import AsyncMock, patch
from src.personalization.service import PersonalizationService
from src.personalization.models import ChapterPersonalizationRequest

@pytest.fixture
def sample_user_profile():
    return {
        "programming_level": "Beginner",
        "programming_languages": ["Python"],
        "ai_knowledge_level": "None",
        "hardware_experience": "None",
        "learning_style": "Code-first"
    }

@pytest.fixture
def sample_chapter_content():
    return """
# Understanding ROS2 Nodes

A ROS2 node is a process that performs computation.

## Creating a Node

```python
import rclpy

def main():
    rclpy.init()
```

## Summary

Nodes are fundamental to ROS2.
"""

class TestPersonalizationService:

    @pytest.mark.asyncio
    async def test_personalize_chapter_preserves_headings(
        self, sample_user_profile, sample_chapter_content
    ):
        """Test that personalization preserves all headings."""
        service = PersonalizationService()

        with patch.object(service.model, 'generate_content_async') as mock_llm:
            # Mock LLM response with headings preserved
            mock_response = AsyncMock()
            mock_response.text = """
# Understanding ROS2 Nodes

Think of a ROS2 node like a worker in a factory...

## Creating a Node

Let's walk through creating your first node step by step...

```python
import rclpy

def main():
    rclpy.init()
```

## Summary

In this section, we learned that nodes are fundamental...
"""
            mock_llm.return_value = mock_response

            content, summary = await service.personalize_chapter(
                sample_chapter_content,
                "ROS2 Nodes",
                sample_user_profile
            )

            # Verify headings preserved
            assert "# Understanding ROS2 Nodes" in content
            assert "## Creating a Node" in content
            assert "## Summary" in content

    @pytest.mark.asyncio
    async def test_personalize_chapter_preserves_code_blocks(
        self, sample_user_profile, sample_chapter_content
    ):
        """Test that personalization preserves code blocks."""
        service = PersonalizationService()

        with patch.object(service.model, 'generate_content_async') as mock_llm:
            mock_response = AsyncMock()
            mock_response.text = sample_chapter_content  # Return unchanged
            mock_llm.return_value = mock_response

            content, _ = await service.personalize_chapter(
                sample_chapter_content,
                "Test",
                sample_user_profile
            )

            assert "```python" in content
            assert "import rclpy" in content
```

#### 5.2 Frontend Integration Tests

**File:** `src/__tests__/PersonalizeButton.test.jsx`

```jsx
import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import PersonalizeButton from '../components/PersonalizeButton';

// Mock useAuth hook
jest.mock('../hooks/useAuth', () => ({
  useAuth: () => ({
    isAuthenticated: true,
    user: { profile: { programming_level: 'Beginner' } }
  })
}));

describe('PersonalizeButton', () => {
  it('renders idle state for authenticated users', () => {
    render(
      <PersonalizeButton
        isPersonalized={false}
        isLoading={false}
        error={null}
        onPersonalize={() => {}}
        onReset={() => {}}
      />
    );

    expect(screen.getByText('Personalize for Me')).toBeInTheDocument();
  });

  it('shows loading state when personalizing', () => {
    render(
      <PersonalizeButton
        isPersonalized={false}
        isLoading={true}
        error={null}
        onPersonalize={() => {}}
        onReset={() => {}}
      />
    );

    expect(screen.getByText('Personalizing...')).toBeInTheDocument();
  });

  it('shows reset button when personalized', () => {
    render(
      <PersonalizeButton
        isPersonalized={true}
        isLoading={false}
        error={null}
        adaptationSummary={{ level_adjustment: 'beginner', processing_time_ms: 2000 }}
        onPersonalize={() => {}}
        onReset={() => {}}
      />
    );

    expect(screen.getByText('View Original')).toBeInTheDocument();
    expect(screen.getByText(/Personalized for You/)).toBeInTheDocument();
  });

  it('calls onPersonalize when button clicked', () => {
    const mockPersonalize = jest.fn();

    render(
      <PersonalizeButton
        isPersonalized={false}
        isLoading={false}
        error={null}
        onPersonalize={mockPersonalize}
        onReset={() => {}}
      />
    );

    fireEvent.click(screen.getByText('Personalize for Me'));
    expect(mockPersonalize).toHaveBeenCalled();
  });
});
```

#### 5.3 E2E Test Scenarios

**File:** `tests/e2e/personalization.spec.js` (Playwright)

```javascript
import { test, expect } from '@playwright/test';

test.describe('Chapter Personalization', () => {

  test.beforeEach(async ({ page }) => {
    // Login before each test
    await page.goto('/auth/signin');
    await page.fill('input[name="email"]', 'test@example.com');
    await page.fill('input[name="password"]', 'TestPass123!');
    await page.click('button[type="submit"]');
    await page.waitForURL('/dashboard');
  });

  test('shows personalize button on chapter pages', async ({ page }) => {
    await page.goto('/docs/module-01-foundations/chapter-01-ros2-nervous-system/page-01-what-problem-does-ros2-solve');

    await expect(page.locator('text=Personalize for Me')).toBeVisible();
  });

  test('personalizes content and shows toggle', async ({ page }) => {
    await page.goto('/docs/module-01-foundations/chapter-01-ros2-nervous-system/page-01-what-problem-does-ros2-solve');

    await page.click('text=Personalize for Me');

    // Wait for loading
    await expect(page.locator('text=Personalizing...')).toBeVisible();

    // Wait for completion (60s timeout for LLM)
    await expect(page.locator('text=View Original')).toBeVisible({ timeout: 60000 });
    await expect(page.locator('text=Personalized for You')).toBeVisible();
  });

  test('toggles back to original content', async ({ page }) => {
    await page.goto('/docs/module-01-foundations/chapter-01-ros2-nervous-system/page-01-what-problem-does-ros2-solve');

    // Personalize first
    await page.click('text=Personalize for Me');
    await expect(page.locator('text=View Original')).toBeVisible({ timeout: 60000 });

    // Toggle back
    await page.click('text=View Original');
    await expect(page.locator('text=Personalize for Me')).toBeVisible();
  });

  test('shows sign-in prompt for unauthenticated users', async ({ page, context }) => {
    // Clear auth
    await context.clearCookies();
    await page.evaluate(() => localStorage.clear());

    await page.goto('/docs/module-01-foundations/chapter-01-ros2-nervous-system/page-01-what-problem-does-ros2-solve');

    await expect(page.locator('text=Sign in to personalize')).toBeVisible();
  });
});
```

---

### Phase 6: Documentation & Deployment (Days 13-14)

#### 6.1 API Documentation Update

**Add to:** `docs/module-05-integrated-rag-chatbot/api-usage.md`

```markdown
## Chapter Personalization API

### POST /api/v1/personalize/chapter

Personalizes chapter content based on the authenticated user's profile.

**Authentication:** Required (Bearer token)

**Request Body:**
```json
{
  "chapter_slug": "module-01-foundations/chapter-01/page-01",
  "chapter_content": "# Chapter Title\n\nContent here...",
  "chapter_title": "Chapter Title"
}
```

**Response:**
```json
{
  "success": true,
  "personalized_content": "# Chapter Title\n\nPersonalized content...",
  "adaptation_summary": {
    "level_adjustment": "beginner",
    "processing_time_ms": 2340,
    "content_length_original": 5000,
    "content_length_personalized": 7500
  }
}
```

**Rate Limit:** 10 requests per minute per user

**Errors:**
- `401`: Authentication required
- `400`: User profile not found
- `422`: Invalid request (content too large)
- `429`: Rate limit exceeded
- `503`: LLM service unavailable
```

#### 6.2 User Guide

**File:** `docs/module-05-integrated-rag-chatbot/personalization.md`

```markdown
---
title: Content Personalization
sidebar_position: 4
---

# Personalizing Your Learning Experience

Our textbook adapts to YOUR background! Each chapter can be personalized based on your:

- **Programming Level** - Beginner, Intermediate, or Advanced
- **Known Languages** - Python, C++, JavaScript, etc.
- **AI Knowledge** - From none to advanced
- **Hardware Experience** - Basic, Robotics, Embedded Systems
- **Learning Style** - Theory-first, Code-first, Visual, or Mixed

## How to Use

1. **Sign in** to your account (or create one)
2. **Navigate** to any chapter
3. Click **"Personalize for Me"** at the top
4. Wait a few seconds while AI adapts the content
5. Click **"View Original"** anytime to toggle back

## What Changes?

| Your Level | Adaptation |
|------------|------------|
| Beginner | More explanations, everyday analogies, step-by-step reasoning |
| Intermediate | Balanced depth, practical examples |
| Advanced | Concise language, focus on edge cases and optimizations |

## What Stays the Same?

- All headings and structure
- All code blocks and commands
- All technical terminology
- All links and references

## Tips

- Complete your profile for best results
- Personalization is cached - revisiting is instant
- Works best on longer chapters with dense content
```

---

## 4. File Structure Summary

```
Physical_AI_Humanoid_Robotics/
├── backend/
│   └── src/
│       ├── personalization/           # NEW
│       │   ├── __init__.py
│       │   ├── models.py              # Request/Response schemas
│       │   ├── service.py             # LLM integration
│       │   └── personalization_prompt.md  # Prompt template
│       └── api/
│           └── routes/
│               └── personalize.py     # NEW endpoint
├── src/
│   ├── components/
│   │   └── PersonalizeButton/         # NEW
│   │       ├── PersonalizeButton.jsx
│   │       ├── PersonalizeButton.module.css
│   │       └── index.js
│   ├── hooks/
│   │   └── useChapterPersonalization.js  # NEW
│   ├── services/
│   │   └── personalizationApi.js      # NEW
│   └── theme/
│       └── DocItem/                   # NEW (swizzled)
│           ├── index.js
│           └── styles.module.css
├── specs/
│   └── 007-chapter-personalization/
│       ├── spec.md
│       └── plan.md                    # THIS FILE
└── docs/
    └── module-05-integrated-rag-chatbot/
        └── personalization.md         # NEW user guide
```

---

## 5. Dependencies to Install

### Backend
```bash
# Already installed via existing requirements
# No new dependencies needed
```

### Frontend
```bash
npm install react-markdown remark-gfm
```

---

## 6. Configuration Updates

### docusaurus.config.js
```javascript
// No changes needed - chatApiUrl already configured
```

### backend/.env
```bash
# No changes needed - Gemini API key already configured
```

---

## 7. Risk Mitigation

| Risk | Mitigation |
|------|------------|
| LLM removes headings | Post-validation with regex; reject and retry |
| LLM modifies code | Extract code blocks before, stitch after |
| High latency | Loading indicator with progress hint |
| Gemini quota exceeded | Rate limiting + fallback to original |
| MDX component breakage | Use plain Markdown rendering for personalized |

---

## 8. Success Criteria

- [ ] Beginner users see expanded explanations with analogies
- [ ] Advanced users see concise content
- [ ] All headings preserved in personalized output
- [ ] All code blocks unchanged
- [ ] Toggle between original/personalized works
- [ ] Session caching prevents re-fetching
- [ ] Unauthenticated users see sign-in prompt
- [ ] Error states handled gracefully
- [ ] < 10s response time for personalization

---

## 9. Timeline Summary

| Phase | Days | Deliverables |
|-------|------|--------------|
| 1. Backend Service | 1-3 | Endpoint, service, prompt |
| 2. Frontend Hook | 4-5 | API service, hook |
| 3. Button Component | 6-7 | React component, styles |
| 4. DocItem Integration | 8-9 | Swizzled theme, content swap |
| 5. Testing | 10-12 | Unit, integration, E2E tests |
| 6. Documentation | 13-14 | API docs, user guide |

**Total: 14 days**

---

*Plan created: 2025-12-17*
*Ready for task generation via `/sp.tasks`*
