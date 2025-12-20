# Quickstart: Chapter-Level Urdu Translation

## Overview
This guide explains how to implement and use the chapter-level Urdu translation feature in the Docusaurus-based technical book.

## Prerequisites

- Python 3.10+ (for backend)
- Node.js 18+ (for frontend)
- Google Gemini API key
- Existing personalization feature set up (this feature reuses the personalization architecture)

## Backend Setup

### 1. Create Translation Service
```bash
# Create the translation service in the backend
mkdir -p backend/src/translation
touch backend/src/translation/__init__.py
touch backend/src/translation/service.py
touch backend/src/translation/models.py
```

### 2. Create Translation API Endpoint
```bash
# Create the translation API route
touch backend/src/api/routes/translate.py
```

### 3. Add Translation Prompt Template
```bash
# Create the translation prompt template
touch backend/src/translation/translation_prompt.md
```

## Frontend Setup

### 1. Create Translation Button Component
```bash
# Create the translation button component
mkdir -p src/components/TranslateButton
touch src/components/TranslateButton/TranslateButton.jsx
touch src/components/TranslateButton/TranslateButton.module.css
```

### 2. Create Translation Hook
```bash
# Create the translation hook
touch src/hooks/useChapterTranslation.js
```

### 3. Create Translation API Service
```bash
# Create the translation API service
touch src/services/translationApi.js
```

## Implementation Steps

### 1. Backend Implementation
1. Implement the `TranslationService` class following the same pattern as `PersonalizationService`
2. Create API endpoint at `/api/v1/translate/chapter` similar to the personalization endpoint
3. Create a translation-specific prompt template that preserves technical content

### 2. Frontend Implementation
1. Create the `TranslateButton` component similar to `PersonalizeButton`
2. Implement the `useChapterTranslation` hook similar to `useChapterPersonalization`
3. Create the `translationApi` service similar to `personalizationApi`

### 3. Integration
1. Add the translation button to the `DocItem` theme component
2. Connect the button to the translation hook
3. Implement caching using sessionStorage
4. Add toggle functionality between English and Urdu versions

## Testing

### Backend Tests
```bash
# Run backend translation tests
cd backend
pytest tests/test_translation.py
```

### Frontend Tests
```bash
# Run frontend translation tests
npm test -- --testPathPattern=TranslateButton
```

## Configuration

### Environment Variables
Ensure the following environment variable is set:
- `GEMINI_API_KEY`: Your Google Gemini API key for translation

### Docusaurus Configuration
The translation feature integrates with the existing Docusaurus setup and requires no additional configuration.

## Usage

Once implemented, users will see a "Translate to Urdu" button at the beginning of each chapter. Clicking this button will:
1. Send the chapter content to the backend translation service
2. Preserve all code blocks, technical identifiers, and document structure
3. Return the translated content in Urdu
4. Cache the translation for future use
5. Provide a toggle to switch back to English