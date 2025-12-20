# Feature Specification: Chapter-Level Content Personalization

**Feature ID:** 007-chapter-personalization
**Version:** 1.0.0
**Status:** Draft
**Author:** Claude Code
**Created:** 2025-12-17
**Last Updated:** 2025-12-17

---

## 1. Executive Summary

### 1.1 Problem Statement

Technical books face a fundamental challenge: the same content must serve readers with vastly different backgrounds. A beginner in robotics needs more foundational explanations and intuition-building, while an advanced practitioner wants concise, direct information. Currently, our Physical AI & Humanoid Robotics book delivers identical content to all readers, regardless of their:

- Programming experience level (beginner/intermediate/advanced)
- Software background (Python, C++, ROS familiarity)
- Hardware experience (embedded systems, robotics, GPUs)
- AI/ML knowledge level
- Preferred learning style (theory-first, code-first, visual)

### 1.2 Proposed Solution

Implement a **"Personalize for Me"** button at the start of each chapter that:

1. Retrieves the logged-in user's profile data
2. Sends the original chapter content + user profile to an LLM
3. Returns a dynamically rewritten version tailored to the user's background
4. Displays the personalized content inline, replacing the original

### 1.3 Key Constraints

- **Content Integrity:** Do NOT remove, shorten, or skip any topics
- **Structure Preservation:** Maintain all headings, section order, code blocks, commands, and technical terms
- **Adaptation Only:** Only adapt depth, tone, explanations, and analogies
- **Reversibility:** Users can toggle back to original content at any time

---

## 2. User Stories

### 2.1 Primary User Stories

| ID | As a... | I want to... | So that... | Priority |
|----|---------|--------------|------------|----------|
| US-01 | Beginner reader | click "Personalize for Me" on a chapter | I get more explanations and intuition-building | P0 |
| US-02 | Advanced reader | personalize content for my level | I skip basic explanations and get concise info | P0 |
| US-03 | Hardware-focused reader | see content adapted to my embedded systems background | analogies relate to my experience | P1 |
| US-04 | Any reader | toggle back to original content | I can compare or share the standard version | P1 |
| US-05 | Non-authenticated user | see a prompt to sign in for personalization | I understand I need an account | P2 |

### 2.2 Acceptance Criteria

**US-01: Beginner Personalization**
```gherkin
Given I am logged in as a user with programming_level="beginner"
And I am viewing a chapter page
When I click the "Personalize for Me" button
Then the chapter content is replaced with a beginner-friendly version
And all original headings and sections are preserved
And code blocks remain unchanged
And additional explanations are provided for technical concepts
And the button changes to "View Original"
```

**US-02: Advanced Personalization**
```gherkin
Given I am logged in as a user with programming_level="advanced"
And ai_knowledge_level="advanced"
When I click the "Personalize for Me" button
Then the chapter content becomes more concise
And advanced terminology is used without over-explanation
And "obvious" setup steps are condensed
And code remains fully intact
```

**US-04: Toggle Back to Original**
```gherkin
Given I am viewing personalized content
When I click "View Original"
Then the original chapter content is restored
And the button changes back to "Personalize for Me"
And no page reload occurs
```

---

## 3. Functional Requirements

### 3.1 Frontend Requirements

#### FR-01: Personalize Button Component
- **Location:** Top of each chapter page, below the title
- **States:**
  - `idle`: "Personalize for Me" button visible (authenticated users only)
  - `loading`: Spinner with "Personalizing..." text
  - `personalized`: "View Original" button visible
  - `error`: Error message with retry option
  - `unauthenticated`: "Sign in to personalize" link

#### FR-02: Content Display
- **Original Content:** Rendered from MDX via Docusaurus
- **Personalized Content:** Rendered from LLM response (Markdown)
- **Transition:** Smooth fade animation between states
- **Preservation:** Code blocks must retain syntax highlighting

#### FR-03: State Persistence
- **Session Storage:** Personalized content cached per chapter per session
- **Key Format:** `personalized_content_{chapter_slug}`
- **Invalidation:** Clear on logout or profile update

### 3.2 Backend Requirements

#### FR-04: Personalization Endpoint
```
POST /api/v1/personalize/chapter
Authorization: Bearer <token>
Content-Type: application/json

Request:
{
  "chapter_slug": "module-01-foundations/chapter-01-ros2-nervous-system/page-01-what-problem-does-ros2-solve",
  "chapter_content": "# What Problem Does ROS2 Solve?\n\n...",
  "chapter_title": "What Problem Does ROS2 Solve?"
}

Response:
{
  "success": true,
  "personalized_content": "# What Problem Does ROS2 Solve?\n\n[Personalized content...]",
  "adaptation_summary": {
    "level_adjustment": "beginner",
    "added_explanations": 12,
    "analogies_added": 5,
    "processing_time_ms": 2340
  }
}
```

#### FR-05: LLM Prompt Engineering
The system prompt must include:
1. User profile context (all 5 fields)
2. Strict preservation rules
3. Adaptation guidelines per level
4. Output format requirements

#### FR-06: Content Validation
- **Input:** Validate chapter_content is non-empty, < 50KB
- **Output:** Validate response maintains all original headings
- **Error Handling:** Return original content on LLM failure

### 3.3 Content Adaptation Rules

#### FR-07: Adaptation by Experience Level

| Level | Explanation Depth | Analogies | Code Comments | Assumed Knowledge |
|-------|-------------------|-----------|---------------|-------------------|
| Beginner | Maximum - explain every concept | Many, everyday objects | Verbose, step-by-step | None assumed |
| Intermediate | Moderate - explain new concepts | Domain-specific | Key points only | Basic programming |
| Advanced | Minimal - reference only | Expert-level | Minimal | Full stack familiarity |

#### FR-08: Adaptation by Learning Style

| Style | Adaptation |
|-------|------------|
| Theory | Lead with concepts, then examples |
| Code-first | Lead with examples, explain after |
| Visual | Add ASCII diagrams, emphasize visual elements |
| Mixed | Balanced approach |

#### FR-09: Strict Preservation Rules

**MUST Preserve:**
- All H1, H2, H3, H4 headings (exact text)
- Section ordering
- All code blocks (exact content)
- All terminal commands
- All technical terms and definitions
- All links and references
- All warnings, tips, and callouts

**MAY Adapt:**
- Explanatory paragraphs (expand/condense)
- Analogies and examples
- Transitional sentences
- Introductory context

---

## 4. Non-Functional Requirements

### 4.1 Performance

| Metric | Target | Maximum |
|--------|--------|---------|
| Time to first byte | < 500ms | 1000ms |
| Full response time | < 5s | 10s |
| Content size increase | < 50% | 100% |

### 4.2 Reliability

- **Availability:** 99.5% uptime for personalization endpoint
- **Fallback:** Return original content on any error
- **Retry:** Automatic retry once on 5xx errors
- **Circuit Breaker:** Disable personalization after 5 consecutive failures

### 4.3 Security

- **Authentication:** Required for personalization
- **Authorization:** Users can only personalize with their own profile
- **Rate Limiting:** 10 requests per minute per user
- **Content Sanitization:** Strip any injected scripts from LLM output

### 4.4 Scalability

- **Caching:** Cache personalized content for 24 hours (invalidate on profile change)
- **Queue:** Process requests via async queue if > 100 concurrent
- **Cost:** Estimate ~$0.02 per personalization (Gemini API)

---

## 5. Technical Architecture

### 5.1 System Components

```
┌─────────────────────────────────────────────────────────────────┐
│                     FRONTEND (Docusaurus)                       │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────────────┐    ┌──────────────────┐                  │
│  │  ChapterPage     │    │  PersonalizeBtn  │                  │
│  │  (MDX Renderer)  │◄───│  Component       │                  │
│  └────────┬─────────┘    └────────┬─────────┘                  │
│           │                       │                             │
│           │    ┌──────────────────┴──────────────────┐         │
│           │    │  useChapterPersonalization Hook     │         │
│           │    │  - fetchPersonalizedContent()       │         │
│           │    │  - cacheInSessionStorage()          │         │
│           │    └──────────────────┬──────────────────┘         │
└───────────┼───────────────────────┼─────────────────────────────┘
            │                       │
            ▼                       ▼
┌─────────────────────────────────────────────────────────────────┐
│                     BACKEND (FastAPI)                           │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────────────┐    ┌──────────────────┐                  │
│  │  /api/v1/        │    │  Personalization │                  │
│  │  personalize/    │───►│  Service         │                  │
│  │  chapter         │    │                  │                  │
│  └──────────────────┘    └────────┬─────────┘                  │
│                                   │                             │
│           ┌───────────────────────┼───────────────────────┐    │
│           ▼                       ▼                       ▼    │
│  ┌────────────────┐    ┌──────────────────┐    ┌────────────┐ │
│  │  User Profile  │    │  LLM Client      │    │  Content   │ │
│  │  Repository    │    │  (Gemini)        │    │  Cache     │ │
│  └────────────────┘    └──────────────────┘    └────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

### 5.2 Data Flow

```
1. User clicks "Personalize for Me"
   │
2. Frontend checks session storage cache
   │
   ├─► Cache HIT: Display cached content
   │
   └─► Cache MISS:
       │
3.     Extract current chapter content from DOM
       │
4.     POST /api/v1/personalize/chapter
       │
5.     Backend fetches user profile from database
       │
6.     Build LLM prompt with:
       │   - System prompt (adaptation rules)
       │   - User profile context
       │   - Original chapter content
       │
7.     Call Gemini API
       │
8.     Validate response (headings preserved)
       │
9.     Return personalized content
       │
10.    Frontend caches in session storage
       │
11.    Render personalized Markdown
```

### 5.3 LLM Prompt Structure

```markdown
## System Prompt

You are an expert technical writer adapting educational content for individual learners.

### Your Task
Rewrite the following chapter content to match the user's background and learning preferences.

### Strict Rules (MUST FOLLOW)
1. PRESERVE all headings exactly (H1, H2, H3, H4)
2. PRESERVE all code blocks exactly (do not modify code)
3. PRESERVE all terminal commands exactly
4. PRESERVE section ordering
5. PRESERVE all technical terms
6. PRESERVE all links and references
7. DO NOT remove or skip any topics
8. DO NOT shorten the content significantly

### Adaptation Guidelines

**For Beginners:**
- Add intuition before technical details
- Use everyday analogies
- Explain acronyms and jargon
- Add "Why this matters" context
- Include more transitional sentences

**For Intermediate:**
- Balanced explanations
- Use domain-specific analogies
- Assume basic programming knowledge
- Focus on practical applications

**For Advanced:**
- Concise, direct language
- Skip basic explanations
- Use expert terminology freely
- Focus on edge cases and optimizations

### User Profile
- Programming Level: {programming_level}
- Known Languages: {programming_languages}
- AI Knowledge: {ai_knowledge_level}
- Hardware Experience: {hardware_experience}
- Learning Style: {learning_style}

### Original Content
{chapter_content}

### Output Format
Return ONLY the adapted chapter content in Markdown format.
Do not include any meta-commentary or explanations of changes.
```

---

## 6. API Specification

### 6.1 Personalize Chapter Endpoint

```yaml
openapi: 3.0.3
paths:
  /api/v1/personalize/chapter:
    post:
      summary: Personalize chapter content for authenticated user
      security:
        - bearerAuth: []
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required:
                - chapter_slug
                - chapter_content
              properties:
                chapter_slug:
                  type: string
                  description: URL slug of the chapter
                  example: "module-01-foundations/chapter-01/page-01"
                chapter_content:
                  type: string
                  description: Original chapter content in Markdown
                  maxLength: 51200
                chapter_title:
                  type: string
                  description: Chapter title for context
      responses:
        '200':
          description: Personalized content
          content:
            application/json:
              schema:
                type: object
                properties:
                  success:
                    type: boolean
                  personalized_content:
                    type: string
                  adaptation_summary:
                    type: object
                    properties:
                      level_adjustment:
                        type: string
                      processing_time_ms:
                        type: integer
        '401':
          description: Unauthorized
        '422':
          description: Validation error
        '429':
          description: Rate limit exceeded
        '503':
          description: LLM service unavailable
```

### 6.2 Error Responses

```json
// 401 Unauthorized
{
  "error": "unauthorized",
  "message": "Authentication required for personalization"
}

// 422 Validation Error
{
  "error": "validation_error",
  "message": "Chapter content exceeds maximum size",
  "details": {
    "field": "chapter_content",
    "max_size": 51200,
    "actual_size": 65000
  }
}

// 429 Rate Limited
{
  "error": "rate_limit_exceeded",
  "message": "Too many personalization requests",
  "retry_after": 60
}

// 503 Service Unavailable
{
  "error": "service_unavailable",
  "message": "Personalization service temporarily unavailable",
  "fallback": "original_content"
}
```

---

## 7. User Interface Design

### 7.1 Button States

```
┌─────────────────────────────────────────────────────────────┐
│  STATE: IDLE (Authenticated)                                │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  ✨ Personalize for Me                              │   │
│  └─────────────────────────────────────────────────────┘   │
│  Adapts this chapter to your experience level              │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  STATE: LOADING                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  ◌ Personalizing...                                 │   │
│  └─────────────────────────────────────────────────────┘   │
│  Tailoring content for your beginner level...              │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  STATE: PERSONALIZED                                        │
│  ┌───────────────────────┐  ┌────────────────────────────┐ │
│  │  📖 View Original     │  │  ✓ Personalized for You   │ │
│  └───────────────────────┘  └────────────────────────────┘ │
│  Content adapted for: Beginner · Code-first learner        │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  STATE: UNAUTHENTICATED                                     │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  🔒 Sign in to personalize                          │   │
│  └─────────────────────────────────────────────────────┘   │
│  Create an account to get content tailored to your level   │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  STATE: ERROR                                               │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  ⚠️ Personalization failed · Retry                  │   │
│  └─────────────────────────────────────────────────────┘   │
│  Could not connect to personalization service              │
└─────────────────────────────────────────────────────────────┘
```

### 7.2 Component Placement

```
┌─────────────────────────────────────────────────────────────┐
│  Navigation Bar                                    [Profile]│
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  # Chapter Title                                            │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  ✨ Personalize for Me                              │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  ## Section 1                                               │
│                                                             │
│  Lorem ipsum dolor sit amet...                              │
│                                                             │
│  ```python                                                  │
│  # Code block                                               │
│  ```                                                        │
│                                                             │
│  ## Section 2                                               │
│  ...                                                        │
│                                                             │
├─────────────────────────────────────────────────────────────┤
│  [Previous Page]                           [Next Page]      │
└─────────────────────────────────────────────────────────────┘
```

---

## 8. Implementation Phases

### Phase 1: Backend Foundation (3-4 days)
- [ ] Create `/api/v1/personalize/chapter` endpoint
- [ ] Implement personalization service with Gemini integration
- [ ] Add rate limiting middleware
- [ ] Create content validation logic
- [ ] Write unit tests for service layer

### Phase 2: LLM Prompt Engineering (2-3 days)
- [ ] Design and test system prompt
- [ ] Create adaptation templates per experience level
- [ ] Implement output validation (heading preservation)
- [ ] Test with various chapter types
- [ ] Tune for quality and consistency

### Phase 3: Frontend Component (2-3 days)
- [ ] Create `PersonalizeButton` component
- [ ] Implement `useChapterPersonalization` hook
- [ ] Add session storage caching
- [ ] Handle all button states (idle, loading, personalized, error)
- [ ] Style with CSS Modules

### Phase 4: Docusaurus Integration (1-2 days)
- [ ] Create custom MDX component wrapper
- [ ] Inject personalization button into chapter layout
- [ ] Handle content extraction from DOM
- [ ] Implement Markdown rendering for personalized content

### Phase 5: Testing & QA (2-3 days)
- [ ] End-to-end testing all user flows
- [ ] Test with real user profiles (beginner, intermediate, advanced)
- [ ] Performance testing (response times)
- [ ] Error handling verification
- [ ] Cross-browser testing

### Phase 6: Documentation & Launch (1 day)
- [ ] Update API documentation
- [ ] Write user guide for personalization feature
- [ ] Deploy to staging
- [ ] Monitor initial usage

**Total Estimated Time:** 11-16 days

---

## 9. Dependencies

### 9.1 External Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| Google Gemini API | 2.0 | LLM for content adaptation |
| FastAPI | 0.100+ | Backend framework |
| Docusaurus | 3.0+ | Frontend framework |
| React | 18+ | UI components |

### 9.2 Internal Dependencies

| Dependency | Location | Purpose |
|------------|----------|---------|
| Auth System | `/backend/src/auth/` | User authentication |
| User Profile | `/backend/src/models/user_profile.py` | Profile data |
| Chat API patterns | `/backend/src/agent/` | LLM integration |
| Auth Context | `/src/components/Auth/AuthProvider.jsx` | Frontend auth |

---

## 10. Risks & Mitigations

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| LLM removes headings | High | Medium | Post-process validation, reject invalid responses |
| LLM modifies code blocks | High | Low | Regex extraction before/after, stitch back |
| High latency (>10s) | Medium | Medium | Streaming response, progress indicators |
| Gemini quota exceeded | High | Medium | Fallback to original content, rate limiting |
| Inconsistent quality | Medium | Medium | Prompt engineering, A/B testing |
| Cost overruns | Medium | Low | Usage monitoring, daily caps |

---

## 11. Success Metrics

### 11.1 Quantitative Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Personalization usage rate | > 30% of authenticated users | Analytics |
| Average response time | < 5 seconds | APM |
| Error rate | < 2% | Logging |
| Content quality score | > 4/5 user rating | Feedback widget |
| Heading preservation rate | 100% | Automated validation |

### 11.2 Qualitative Metrics

- User feedback on content relevance
- A/B test: Time-on-page for personalized vs original
- User retention comparison

---

## 12. Future Enhancements

### 12.1 Near-term (v1.1)
- Persist personalized content to database (not just session)
- "Personalize All Chapters" batch option
- Export personalized content as PDF

### 12.2 Long-term (v2.0)
- Real-time streaming of personalized content
- Collaborative filtering (recommend based on similar users)
- Personalized quizzes and exercises
- Progress tracking with adaptive difficulty

---

## 13. Glossary

| Term | Definition |
|------|------------|
| Personalization | Adapting content based on user profile |
| Experience Level | User's self-reported programming expertise |
| Learning Style | User's preferred way of learning (theory/code/visual) |
| Content Integrity | Preserving all essential elements during adaptation |
| LLM | Large Language Model (Gemini in this case) |

---

## 14. Appendix

### 14.1 Sample Adaptation - Before/After

**Original (for all users):**
```markdown
## Understanding ROS2 Nodes

A ROS2 node is a process that performs computation. Nodes communicate
with each other using topics, services, and actions.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
```
```

**Personalized (Beginner):**
```markdown
## Understanding ROS2 Nodes

Think of a ROS2 node like a worker in a factory. Each worker (node) has
a specific job to do, and they communicate with other workers through
messages. Just like factory workers might pass notes or use walkies-talkies,
nodes use "topics" to share information.

**Why does this matter?** In robotics, we break down complex tasks into
smaller pieces. Each node handles one piece, making the system easier to
understand, test, and maintain.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
```

In this code, we're creating a new worker (node) named 'my_node'. The
`super().__init__()` line is like filling out the paperwork to officially
register this worker in our ROS2 factory.
```

**Personalized (Advanced):**
```markdown
## Understanding ROS2 Nodes

ROS2 nodes are executables managed by the ROS2 runtime. Communication
primitives: topics (pub/sub), services (RPC), actions (long-running RPC
with feedback).

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
```

Note: Node names must be unique within a namespace. Consider using
`automatically_declare_parameters_from_overrides=True` for parameter
flexibility.
```

---

## 15. Sign-off

| Role | Name | Date | Signature |
|------|------|------|-----------|
| Product Owner | | | |
| Tech Lead | | | |
| QA Lead | | | |

---

*This specification is a living document and will be updated as requirements evolve.*
