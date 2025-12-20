# API Endpoints for Authentication and User Profiling

## 1. Overview
This document defines all API endpoints for the authentication and user profiling system. The endpoints follow RESTful conventions and are designed to work with the Better Auth integration and extended user profile data.

## 2. Base URL
All authentication endpoints are prefixed with `/api/v1/auth` unless otherwise specified.

## 3. Authentication Endpoints

### 3.1 User Registration (Signup)
- **Endpoint**: `POST /api/v1/auth/signup`
- **Description**: Register a new user with profile data
- **Authentication**: None (public endpoint)
- **Request Body**:
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123!",
  "first_name": "John",
  "last_name": "Doe",
  "programming_level": "intermediate",
  "programming_languages": ["Python", "JavaScript"],
  "ai_knowledge_level": "basic",
  "hardware_experience": "robotics",
  "learning_style": "code_first",
  "agree_to_terms": true
}
```
- **Response (Success)**:
```json
{
  "success": true,
  "user_id": "user-uuid-string",
  "email": "user@example.com",
  "first_name": "John",
  "last_name": "Doe",
  "requires_email_verification": true
}
```
- **Response (Error)**:
```json
{
  "success": false,
  "error": "Validation failed",
  "details": {
    "email": ["Email format invalid"],
    "password": ["Password must contain uppercase, lowercase, number, and symbol"],
    "programming_level": ["Field required"]
  }
}
```
- **HTTP Status Codes**:
  - 200: Successful registration
  - 400: Validation error
  - 409: User already exists

### 3.2 User Login (Signin)
- **Endpoint**: `POST /api/v1/auth/signin`
- **Description**: Authenticate existing user
- **Authentication**: None (public endpoint)
- **Request Body**:
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123!"
}
```
- **Response (Success)**:
```json
{
  "success": true,
  "user": {
    "id": "user-uuid-string",
    "email": "user@example.com",
    "first_name": "John",
    "last_name": "Doe"
  },
  "session_token": "jwt-token-string",
  "expires_at": "2023-12-31T23:59:59Z"
}
```
- **Response (Error)**:
```json
{
  "success": false,
  "error": "Invalid credentials"
}
```
- **HTTP Status Codes**:
  - 200: Successful login
  - 400: Invalid request format
  - 401: Invalid credentials
  - 429: Too many attempts (rate limited)

### 3.3 User Logout
- **Endpoint**: `POST /api/v1/auth/signout`
- **Description**: Log out current user
- **Authentication**: Bearer token or session cookie
- **Headers**:
  - Authorization: `Bearer <session_token>` OR include session cookie
- **Request Body**: Empty
- **Response (Success)**:
```json
{
  "success": true,
  "message": "Successfully logged out"
}
```
- **HTTP Status Codes**:
  - 200: Successfully logged out
  - 401: Not authenticated

### 3.4 Get Current User Profile
- **Endpoint**: `GET /api/v1/auth/me`
- **Description**: Get current user's profile including extended profile data
- **Authentication**: Bearer token or session cookie
- **Headers**:
  - Authorization: `Bearer <session_token>` OR include session cookie
- **Response (Success)**:
```json
{
  "id": "user-uuid-string",
  "email": "user@example.com",
  "first_name": "John",
  "last_name": "Doe",
  "profile": {
    "id": "profile-uuid-string",
    "user_id": "user-uuid-string",
    "programming_level": "intermediate",
    "programming_languages": ["Python", "JavaScript"],
    "ai_knowledge_level": "basic",
    "hardware_experience": "robotics",
    "learning_style": "code_first",
    "created_at": "2023-01-01T00:00:00Z",
    "updated_at": "2023-01-02T00:00:00Z"
  }
}
```
- **Response (Error)**:
```json
{
  "error": "Not authenticated"
}
```
- **HTTP Status Codes**:
  - 200: Successfully retrieved profile
  - 401: Not authenticated

## 4. Profile Management Endpoints

### 4.1 Get User Profile by ID
- **Endpoint**: `GET /api/v1/profile/{user_id}`
- **Description**: Get specific user's public profile information
- **Authentication**: Bearer token (authenticated user required)
- **Parameters**:
  - `user_id` (path): UUID of the user whose profile to retrieve
- **Response (Success)**:
```json
{
  "id": "user-uuid-string",
  "first_name": "John",
  "last_name": "Doe",
  "profile": {
    "programming_level": "intermediate",
    "programming_languages": ["Python", "JavaScript"],
    "ai_knowledge_level": "basic",
    "hardware_experience": "robotics",
    "learning_style": "code_first"
  }
}
```
- **HTTP Status Codes**:
  - 200: Successfully retrieved profile
  - 401: Not authenticated
  - 403: Insufficient permissions (implementation-dependent)
  - 404: User not found

### 4.2 Update User Profile
- **Endpoint**: `PUT /api/v1/auth/profile`
- **Description**: Update current user's profile information
- **Authentication**: Bearer token or session cookie
- **Headers**:
  - Authorization: `Bearer <session_token>` OR include session cookie
- **Request Body** (all fields optional):
```json
{
  "programming_level": "advanced",
  "programming_languages": ["Python", "JavaScript", "C++"],
  "ai_knowledge_level": "intermediate",
  "hardware_experience": "embedded_systems",
  "learning_style": "mixed"
}
```
- **Response (Success)**:
```json
{
  "success": true,
  "profile": {
    "id": "profile-uuid-string",
    "user_id": "user-uuid-string",
    "programming_level": "advanced",
    "programming_languages": ["Python", "JavaScript", "C++"],
    "ai_knowledge_level": "intermediate",
    "hardware_experience": "embedded_systems",
    "learning_style": "mixed",
    "updated_at": "2023-01-03T00:00:00Z"
  }
}
```
- **HTTP Status Codes**:
  - 200: Successfully updated profile
  - 400: Validation error
  - 401: Not authenticated

### 4.3 Get User Preferences for Content Personalization
- **Endpoint**: `GET /api/v1/auth/preferences`
- **Description**: Get user preferences specifically for content personalization
- **Authentication**: Bearer token or session cookie
- **Headers**:
  - Authorization: `Bearer <session_token>` OR include session cookie
- **Response (Success)**:
```json
{
  "programming_level": "intermediate",
  "programming_languages": ["Python", "JavaScript"],
  "ai_knowledge_level": "basic",
  "hardware_experience": "robotics",
  "learning_style": "code_first",
  "personalization_score": 0.85  // Calculated score for content matching
}
```
- **HTTP Status Codes**:
  - 200: Successfully retrieved preferences
  - 401: Not authenticated

## 5. Account Management Endpoints

### 5.1 Verify Email
- **Endpoint**: `POST /api/v1/auth/verify-email`
- **Description**: Verify user's email using verification token
- **Authentication**: None (public endpoint)
- **Request Body**:
```json
{
  "token": "verification-token-string"
}
```
- **Response (Success)**:
```json
{
  "success": true,
  "message": "Email verified successfully"
}
```
- **HTTP Status Codes**:
  - 200: Email verified
  - 400: Invalid or expired token

### 5.2 Resend Verification Email
- **Endpoint**: `POST /api/v1/auth/resend-verification`
- **Description**: Resend email verification link
- **Authentication**: Bearer token (user must be logged in)
- **Headers**:
  - Authorization: `Bearer <session_token>`
- **Request Body**: Empty
- **Response (Success)**:
```json
{
  "success": true,
  "message": "Verification email sent"
}
```
- **HTTP Status Codes**:
  - 200: Verification email sent
  - 401: Not authenticated

## 6. Rate Limiting and Security Headers

### 6.1 Rate Limiting
- Authentication endpoints limited to 5 attempts per minute per IP
- Profile update endpoints limited to 10 requests per minute per user
- Include `X-RateLimit-Limit`, `X-RateLimit-Remaining`, and `X-RateLimit-Reset` headers

### 6.2 Security Headers
All endpoints should include:
- `X-Content-Type-Options: nosniff`
- `X-Frame-Options: DENY`
- `X-XSS-Protection: 1; mode=block`
- Strict transport security headers in production

## 7. Error Response Format

### 7.1 Standard Error Response
```json
{
  "error": "Error message",
  "error_code": "ERROR_CODE",
  "timestamp": "2023-01-01T00:00:00Z",
  "request_id": "unique-request-id"
}
```

### 7.2 Validation Error Response
```json
{
  "error": "Validation failed",
  "error_code": "VALIDATION_ERROR",
  "details": {
    "field_name": ["Error message 1", "Error message 2"]
  },
  "timestamp": "2023-01-01T00:00:00Z",
  "request_id": "unique-request-id"
}
```

## 8. Example FastAPI Implementation

### 8.1 Main Router Definition
```python
# routers/auth.py
from fastapi import APIRouter, Depends, HTTPException, Request
from sqlalchemy.orm import Session
from typing import List, Optional
from .. import schemas, crud, database
from ..auth import get_current_user
from ..models import ProgrammingLevel, AIKnowledgeLevel, HardwareExperience, LearningStyle

router = APIRouter(prefix="/api/v1/auth", tags=["authentication"])

@router.post("/signup", response_model=schemas.UserResponse)
async def register_user(user_data: schemas.UserRegistrationRequest):
    # Implementation here
    pass

@router.post("/signin", response_model=schemas.SignInResponse)
async def login_user(credentials: schemas.LoginRequest):
    # Implementation here
    pass

@router.get("/me", response_model=schemas.UserProfileResponse)
async def get_current_user_profile(
    current_user = Depends(get_current_user),
    db: Session = Depends(database.get_db)
):
    # Implementation here
    pass

@router.put("/profile", response_model=schemas.ProfileUpdateResponse)
async def update_user_profile(
    profile_update: schemas.UserProfileUpdateRequest,
    current_user = Depends(get_current_user),
    db: Session = Depends(database.get_db)
):
    # Implementation here
    pass
```

### 8.2 Profile Router
```python
# routers/profile.py
from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from .. import schemas, crud, database
from ..auth import get_current_user

router = APIRouter(prefix="/api/v1/profile", tags=["profiles"])

@router.get("/{user_id}", response_model=schemas.PublicProfileResponse)
async def get_user_profile(
    user_id: str,
    current_user = Depends(get_current_user),
    db: Session = Depends(database.get_db)
):
    # Implementation here
    pass
```

## 9. Testing Endpoints

### 9.1 Test Cases
- Test successful user registration with profile data
- Test duplicate email registration
- Test login with valid/invalid credentials
- Test profile retrieval for authenticated users
- Test profile update with various field combinations
- Test unauthorized access to protected endpoints
- Test rate limiting behavior
- Test email verification flow

### 9.2 Mock Data for Testing
```python
TEST_USER_DATA = {
    "email": "test@example.com",
    "password": "SecurePassword123!",
    "first_name": "Test",
    "last_name": "User",
    "programming_level": "intermediate",
    "programming_languages": ["Python", "JavaScript"],
    "ai_knowledge_level": "basic",
    "hardware_experience": "robotics",
    "learning_style": "code_first",
    "agree_to_terms": True
}
```