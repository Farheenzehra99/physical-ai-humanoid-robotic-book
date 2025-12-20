# User Schema and Background Data Collection

## 1. User Profile Schema Definition

### Programming Level Enum
```python
from enum import Enum

class ProgrammingLevel(str, Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"
```

Valid values: `beginner`, `intermediate`, `advanced`
Description: Represents the user's current programming skill level to tailor code complexity in content.

### Programming Languages
```python
from typing import List

programming_languages_options = [
    "Python",
    "JavaScript",
    "TypeScript",
    "C++",
    "Java",
    "C#",
    "Rust",
    "Go",
    "Other"
]
```

Data Type: Array of strings
Description: Multiple selection of programming languages the user knows, to customize examples and tutorials.

### AI Knowledge Level Enum
```python
class AIKnowledgeLevel(str, Enum):
    NONE = "none"
    BASIC = "basic"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"
```

Valid values: `none`, `basic`, `intermediate`, `advanced`
Description: User's current understanding of AI concepts to adjust content depth.

### Hardware Experience Enum
```python
class HardwareExperience(str, Enum):
    NONE = "none"
    BASIC = "basic"
    ROBOTICS = "robotics"
    EMBEDDED_SYSTEMS = "embedded_systems"
```

Valid values: `none`, `basic`, `robotics`, `embedded_systems`
Description: User's hardware experience level to focus content on relevant platforms.

### Learning Style Enum
```python
class LearningStyle(str, Enum):
    THEORY = "theory"
    CODE_FIRST = "code_first"
    VISUAL = "visual"
    MIXED = "mixed"
```

Valid values: `theory`, `code_first`, `visual`, `mixed`
Description: User's preferred learning approach to customize content presentation.

## 2. Pydantic Models

### User Registration Request Model
```python
from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime

class UserRegistrationRequest(BaseModel):
    email: str = Field(..., description="User's email address")
    password: str = Field(..., min_length=8, description="User's password (min 8 chars)")
    first_name: str = Field(..., description="User's first name")
    last_name: str = Field(..., description="User's last name")
    programming_level: ProgrammingLevel = Field(..., description="User's programming skill level")
    programming_languages: List[str] = Field(
        ...,
        description="List of programming languages the user knows",
        max_items=10
    )
    ai_knowledge_level: AIKnowledgeLevel = Field(..., description="User's AI knowledge level")
    hardware_experience: HardwareExperience = Field(..., description="User's hardware experience level")
    learning_style: LearningStyle = Field(..., description="User's preferred learning style")
    agree_to_terms: bool = Field(True, description="User agrees to terms of service")
```

### User Profile Response Model
```python
class UserProfileResponse(BaseModel):
    id: str
    email: str
    first_name: str
    last_name: str
    programming_level: ProgrammingLevel
    programming_languages: List[str]
    ai_knowledge_level: AIKnowledgeLevel
    hardware_experience: HardwareExperience
    learning_style: LearningStyle
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

### User Profile Update Request Model
```python
class UserProfileUpdateRequest(BaseModel):
    programming_level: Optional[ProgrammingLevel] = None
    programming_languages: Optional[List[str]] = None
    ai_knowledge_level: Optional[AIKnowledgeLevel] = None
    hardware_experience: Optional[HardwareExperience] = None
    learning_style: Optional[LearningStyle] = None
```

## 3. Form Validation Rules

### Signup Form Validation
- Email: Must be a valid email format
- Password: Minimum 8 characters, at least one uppercase, one lowercase, one number
- First Name: 1-50 characters, alphabetic only
- Last Name: 1-50 characters, alphabetic only
- Programming Level: Required, one of the enum values
- Programming Languages: Required, at least one selected, maximum 10 selections
- AI Knowledge Level: Required, one of the enum values
- Hardware Experience: Required, one of the enum values
- Learning Style: Required, one of the enum values
- Terms Agreement: Must be true

### Profile Update Validation
- Same validation rules as signup, but all fields optional
- At least one field must be provided for update

## 4. User Interface Components

### Signup Form Structure
```
┌─────────────────────────────────────┐
│              Sign Up                │
├─────────────────────────────────────┤
│ Email: [__________________________]  │
│ Password: [_______________________]  │
│ First Name: [____________________]   │
│ Last Name: [_____________________]   │
├─────────────────────────────────────┤
│ Programming Level: [▼____________]   │
│ Programming Languages: [▼________]   │
│   □ Python                         │
│   □ JavaScript                     │
│   □ TypeScript                     │
│   ...                              │
│ AI Knowledge Level: [▼__________]   │
│ Hardware Experience: [▼_________]   │
│ Learning Style: [▼______________]   │
├─────────────────────────────────────┤
│ [ ] I agree to the Terms of Service │
│                                     │
│           [Sign Up Button]          │
└─────────────────────────────────────┘
```

### Profile Editing Interface
Similar to signup but with saved values pre-filled and individual field updates allowed.

## 5. Data Privacy and Consent

### GDPR Compliance
- User data stored securely in Neon Postgres
- Right to access, modify, or delete personal data
- Data retention policies applied
- Consent for data processing obtained during signup

### Data Usage Policy
- Profile data used solely for content personalization
- No sharing of personal information with third parties
- Analytics may use anonymized data for improving service
- Users can modify profile data at any time

## 6. Error Messages

### Validation Error Messages
- Email: "Please enter a valid email address"
- Password: "Password must be at least 8 characters with uppercase, lowercase, and number"
- Required fields: "{field_name} is required"
- Invalid selection: "Please select a valid {field_name}"
- Programming languages: "Please select at least one programming language"

### Server Error Messages
- Duplicate email: "An account with this email already exists"
- Database error: "Unable to create account, please try again"
- Authentication error: "Invalid credentials, please check your email and password"