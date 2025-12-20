"""
Custom exceptions for authentication system.

This module defines custom exception classes for authentication
and authorization operations.
"""


class AuthException(Exception):
    """
    Base authentication exception class.
    """
    def __init__(self, message: str, error_code: str = "AUTH_ERROR"):
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)


class UserRegistrationError(AuthException):
    """
    Exception raised when user registration fails.
    """
    def __init__(self, message: str):
        super().__init__(message, "USER_REGISTRATION_ERROR")


class UserAuthenticationError(AuthException):
    """
    Exception raised when user authentication fails.
    """
    def __init__(self, message: str):
        super().__init__(message, "USER_AUTHENTICATION_ERROR")


class UserNotFoundError(AuthException):
    """
    Exception raised when a user is not found.
    """
    def __init__(self, message: str = "User not found"):
        super().__init__(message, "USER_NOT_FOUND")


class InvalidCredentialsError(UserAuthenticationError):
    """
    Exception raised when provided credentials are invalid.
    """
    def __init__(self, message: str = "Invalid email or password"):
        super().__init__(message)


class EmailAlreadyRegisteredError(UserRegistrationError):
    """
    Exception raised when trying to register with an already registered email.
    """
    def __init__(self, message: str = "Email already registered"):
        super().__init__(message)


class InsufficientProfileDataError(AuthException):
    """
    Exception raised when required profile data is missing.
    """
    def __init__(self, message: str = "Insufficient profile data provided"):
        super().__init__(message, "INSUFFICIENT_PROFILE_DATA")


class ProfileUpdateError(AuthException):
    """
    Exception raised when profile update fails.
    """
    def __init__(self, message: str):
        super().__init__(message, "PROFILE_UPDATE_ERROR")


class SessionExpiredError(AuthException):
    """
    Exception raised when a session has expired.
    """
    def __init__(self, message: str = "Session has expired"):
        super().__init__(message, "SESSION_EXPIRED")


class UnauthorizedError(AuthException):
    """
    Exception raised when a user is not authorized to access a resource.
    """
    def __init__(self, message: str = "Unauthorized access"):
        super().__init__(message, "UNAUTHORIZED")


class TokenValidationError(AuthException):
    """
    Exception raised when token validation fails.
    """
    def __init__(self, message: str = "Token validation failed"):
        super().__init__(message, "TOKEN_VALIDATION_ERROR")


class RateLimitExceededError(AuthException):
    """
    Exception raised when rate limit is exceeded.
    """
    def __init__(self, message: str = "Rate limit exceeded"):
        super().__init__(message, "RATE_LIMIT_EXCEEDED")


# HTTP Exception Response Format
def create_error_response(error: AuthException, request_id: str = None) -> dict:
    """
    Create a standardized error response.

    Args:
        error: The authentication exception
        request_id: Optional request ID for tracking

    Returns:
        Dictionary with standardized error response format
    """
    response = {
        "error": error.message,
        "error_code": error.error_code,
        "timestamp": __import__('datetime').datetime.utcnow().isoformat()
    }

    if request_id:
        response["request_id"] = request_id

    return response


# Validation error response
def create_validation_error_response(errors: dict, request_id: str = None) -> dict:
    """
    Create a standardized validation error response.

    Args:
        errors: Dictionary of validation errors
        request_id: Optional request ID for tracking

    Returns:
        Dictionary with standardized validation error response format
    """
    response = {
        "error": "Validation failed",
        "error_code": "VALIDATION_ERROR",
        "details": errors,
        "timestamp": __import__('datetime').datetime.utcnow().isoformat()
    }

    if request_id:
        response["request_id"] = request_id

    return response