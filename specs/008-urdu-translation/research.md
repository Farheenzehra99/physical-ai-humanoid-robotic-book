# Research: Chapter-Level Urdu Translation

## Overview
This research document captures the investigation and decisions made during the planning phase for implementing chapter-level Urdu translation functionality for the Docusaurus-based technical book.

## Translation API Options

### Decision: Use Google Gemini API for translation
**Rationale**: The existing personalization system already uses Google Gemini API, so reusing it for translation maintains consistency and leverages existing infrastructure. Gemini is also well-suited for preserving document structure and handling technical content.

**Alternatives considered**:
- OpenAI GPT: Would require additional API key management and potentially different integration
- Azure Translator: Different architecture and potentially different cost structure
- Open-source translation models: Would require significant infrastructure setup and maintenance

## Technical Approach

### Decision: Reuse Personalization Architecture
**Rationale**: The existing personalization feature provides a proven architecture for content transformation that can be adapted for translation. This includes:
- Backend service structure
- Frontend hook pattern
- Caching mechanism
- Error handling
- Authentication integration

**Alternatives considered**:
- Building from scratch: Higher development time and risk
- Third-party translation widget: Less control over preserving formatting and technical content

## Preservation of Technical Content

### Decision: Use Prompt Engineering to Preserve Technical Elements
**Rationale**: Using prompt engineering with the LLM to specifically exclude code blocks, ROS topics, CLI commands, APIs, and file paths from translation while translating prose content is the most effective approach given our existing architecture.

**Implementation approach**:
- Create a translation-specific prompt template
- Use regex patterns to identify and protect technical elements
- Preserve document structure (headings, lists, etc.)

## Frontend Implementation

### Decision: Create TranslationButton Component Similar to PersonalizeButton
**Rationale**: Following the same UI/UX pattern as the existing personalization feature ensures consistency for users and reuses proven patterns.

**Key features needed**:
- Toggle between English and Urdu versions
- Loading states
- Error handling
- Caching of translated content
- Preservation of scroll position during toggle

## Backend Implementation

### Decision: Create Translation Service Following Personalization Service Pattern
**Rationale**: Reusing the same architectural pattern ensures consistency and reduces development time.

**Key components**:
- TranslationService class
- Translation API endpoint
- Translation prompt template
- Content validation to ensure structure preservation