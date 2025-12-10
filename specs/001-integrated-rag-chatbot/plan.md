# Implementation Plan: Integrated RAG Chatbot for Interactive Technical Book

**Branch**: `001-integrated-rag-chatbot` | **Date**: 2025-12-10 | **Spec**: [link to spec](./spec.md)
**Input**: Feature specification from `/specs/[001-integrated-rag-chatbot]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of an Integrated RAG Chatbot embedded in an interactive technical book that answers user queries strictly from book content using two modes: Normal Mode (with Qdrant + Neon RAG pipeline) and Selected Text Only Mode. The system provides citations and prevents hallucinations while supporting flexible deployment options.

## Technical Context

**Language/Version**: Python 3.11, JavaScript/TypeScript for frontend integration
**Primary Dependencies**: FastAPI, Qdrant, Neon, Claude MCP, Docusaurus
**Storage**: Qdrant vector database for embeddings, Neon PostgreSQL for metadata
**Testing**: pytest for backend, Jest for frontend components
**Target Platform**: Web application (Docusaurus-based documentation site)
**Project Type**: Web - determines source structure
**Performance Goals**: <5 seconds response time for queries, 95% availability
**Constraints**: Must handle external service failures gracefully, support hybrid deployment
**Scale/Scope**: Support variable concurrent users based on deployment environment

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The implementation will follow constitutional principles by:
- Using open-source technologies where possible
- Maintaining clear separation of concerns
- Ensuring proper error handling and logging
- Following security best practices with encryption and audit logging
- Supporting flexible deployment options

## Project Structure

### Documentation (this feature)

```text
specs/001-integrated-rag-chatbot/
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
│   │   ├── query.py
│   │   ├── response.py
│   │   └── citation.py
│   ├── services/
│   │   ├── rag_service.py
│   │   ├── qdrant_client.py
│   │   ├── neon_client.py
│   │   └── security_service.py
│   ├── api/
│   │   ├── main.py
│   │   ├── routes/
│   │   │   ├── query.py
│   │   │   └── health.py
│   │   └── middleware/
│   │       ├── auth.py
│   │       └── logging.py
│   └── utils/
│       ├── chunker.py
│       ├── embedding.py
│       └── validator.py
└── tests/
    ├── unit/
    ├── integration/
    └── contract/

frontend/
├── src/
│   ├── components/
│   │   ├── RagChatbot.jsx
│   │   ├── CitationDisplay.jsx
│   │   └── QueryInput.jsx
│   ├── services/
│   │   └── apiClient.js
│   └── hooks/
│       └── useRagChat.js
└── tests/
    ├── unit/
    └── integration/
```

**Structure Decision**: Selected web application structure with separate backend and frontend to maintain clear separation of concerns while enabling flexible deployment. The backend handles RAG operations and external service integration, while the frontend manages UI components for the chatbot interface.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |