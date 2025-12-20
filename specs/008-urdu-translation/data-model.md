# Data Model: Chapter-Level Urdu Translation

## Overview
This document defines the data structures and models for the chapter-level Urdu translation feature.

## API Request/Response Models

### TranslationRequest
```python
class TranslationRequest:
    chapter_slug: str          # URL slug of the chapter
    chapter_content: str       # Original chapter content (Markdown/MDX)
    chapter_title: str         # Chapter title (optional)
```

### TranslationResponse
```python
class TranslationResponse:
    success: bool                           # Whether translation was successful
    translated_content: str                 # Translated chapter content
    translation_summary: TranslationSummary # Metadata about the translation
```

### TranslationSummary
```python
class TranslationSummary:
    processing_time_ms: int              # Time taken for translation in milliseconds
    content_length_original: int         # Original content length
    content_length_translated: int       # Translated content length
    preserved_elements_count: int        # Number of technical elements preserved
```

## Frontend State Models

### TranslationState
```javascript
interface TranslationState {
  isTranslated: boolean;           // Whether content is currently translated
  isLoading: boolean;              // Whether translation is in progress
  error: string | null;            // Error message if translation failed
  translatedContent: string | null; // Cached translated content
  translationSummary: TranslationSummary | null; // Metadata about translation
}
```

## Cache Model

### CachedTranslation
```javascript
interface CachedTranslation {
  content: string;                // The translated content
  summary: TranslationSummary;    // Metadata about the translation
  timestamp: number;              // When the translation was cached
}
```

## Validation Rules

### Content Preservation
- Code blocks (```...```) must remain unchanged in the translated content
- Technical identifiers (ROS topics, CLI commands, APIs, file paths) must remain in English
- Document structure (headings, lists, links) must be preserved
- Markdown/MDX formatting must be maintained

### Translation Quality
- All prose text must be translated to Urdu
- Mixed-script content should be properly handled (Urdu text with English technical terms)
- Content length should be reasonable (not significantly expanded or compressed)