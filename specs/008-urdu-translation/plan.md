# Implementation Plan: Chapter-Level Urdu Translation

**Branch**: `008-urdu-translation` | **Date**: 2025-12-18 | **Spec**: [specs/008-urdu-translation/spec.md](specs/008-urdu-translation/spec.md)
**Input**: Feature specification from `/specs/008-urdu-translation/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement chapter-level Urdu translation functionality for Docusaurus-based technical book. Reuse existing personalization architecture to provide "Translate to Urdu" button that translates current chapter content while preserving document structure, code blocks, and technical identifiers. The system will use Google Gemini API for translation with special handling to exclude code blocks, ROS topics, CLI commands, APIs, and file paths from translation.

## Technical Context

**Language/Version**: Python 3.10+ (backend), JavaScript/React (frontend)
**Primary Dependencies**: FastAPI (backend), Docusaurus/React (frontend), Google Gemini API (translation)
**Storage**: N/A (uses browser sessionStorage for caching, no persistent storage needed)
**Testing**: pytest (backend), Jest (frontend)
**Target Platform**: Web browser (Docusaurus documentation site)
**Project Type**: Web application with backend API services
**Performance Goals**: < 10 seconds for chapter translation, < 0.5 seconds for toggle between languages
**Constraints**: Must preserve Markdown/MDX formatting, code blocks, and technical identifiers; translation should maintain document structure
**Scale/Scope**: Individual chapter-level translation for educational content

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the constitution file, this feature:
- Follows the Execution-First Philosophy by reusing existing personalization architecture
- Maintains Zero-Tolerance Quality Standards with proper error handling and validation
- Follows Test-Driven Development practices with frontend and backend tests
- Supports Open Source & Accessibility by making educational content available in Urdu
- Uses the existing technology stack (Python, FastAPI, React, Docusaurus)

## Project Structure

### Documentation (this feature)

```text
specs/008-urdu-translation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   ├── services/
│   │   └── translation/           # New translation service
│   ├── api/
│   │   └── routes/
│   │       └── translate.py       # New translation API endpoint
│   └── config/
└── tests/

src/
├── components/
│   └── TranslateButton/           # New translation button component
├── hooks/
│   └── useChapterTranslation.js   # New translation hook
├── services/
│   └── translationApi.js          # New translation API service
└── theme/
    └── DocItem/
        └── TranslationWrapper.js  # New translation wrapper
```

**Structure Decision**: Web application with backend API services, following the same architecture as the existing personalization feature but for translation functionality.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |