"""
Validators for authentication data.

This module contains validation functions for authentication
and profile data to ensure data integrity.
"""

from typing import List, Dict, Any
from .schemas import ProgrammingLevel, AIKnowledgeLevel, HardwareExperience, LearningStyle


def validate_programming_languages(languages: List[str]) -> Dict[str, Any]:
    """
    Validate programming languages selection.

    Args:
        languages: List of programming languages to validate

    Returns:
        Dictionary with validation result and any error messages
    """
    valid_languages = [
        "Python", "JavaScript", "TypeScript", "C++", "Java",
        "C#", "Rust", "Go", "Other"
    ]

    result = {
        "is_valid": True,
        "errors": []
    }

    if not languages or len(languages) == 0:
        result["is_valid"] = False
        result["errors"].append("At least one programming language must be selected")
        return result

    if len(languages) > 10:
        result["is_valid"] = False
        result["errors"].append("Maximum 10 programming languages can be selected")
        return result

    for lang in languages:
        if lang not in valid_languages:
            result["is_valid"] = False
            result["errors"].append(f"Invalid programming language: {lang}")

    return result


def validate_profile_data(
    programming_level: str,
    programming_languages: List[str],
    ai_knowledge_level: str,
    hardware_experience: str,
    learning_style: str
) -> Dict[str, Any]:
    """
    Validate complete profile data.

    Args:
        programming_level: User's programming level
        programming_languages: List of programming languages
        ai_knowledge_level: User's AI knowledge level
        hardware_experience: User's hardware experience
        learning_style: User's learning style

    Returns:
        Dictionary with validation result and any error messages
    """
    result = {
        "is_valid": True,
        "errors": {}
    }

    # Validate programming level
    try:
        ProgrammingLevel(programming_level)
    except ValueError:
        result["is_valid"] = False
        result["errors"]["programming_level"] = ["Invalid programming level"]

    # Validate programming languages
    lang_validation = validate_programming_languages(programming_languages)
    if not lang_validation["is_valid"]:
        result["is_valid"] = False
        result["errors"]["programming_languages"] = lang_validation["errors"]

    # Validate AI knowledge level
    try:
        AIKnowledgeLevel(ai_knowledge_level)
    except ValueError:
        result["is_valid"] = False
        result["errors"]["ai_knowledge_level"] = ["Invalid AI knowledge level"]

    # Validate hardware experience
    try:
        HardwareExperience(hardware_experience)
    except ValueError:
        result["is_valid"] = False
        result["errors"]["hardware_experience"] = ["Invalid hardware experience"]

    # Validate learning style
    try:
        LearningStyle(learning_style)
    except ValueError:
        result["is_valid"] = False
        result["errors"]["learning_style"] = ["Invalid learning style"]

    return result


def validate_email_format(email: str) -> bool:
    """
    Validate email format.

    Args:
        email: Email address to validate

    Returns:
        True if email format is valid, False otherwise
    """
    import re
    pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
    return re.match(pattern, email) is not None


def validate_password_strength(password: str) -> Dict[str, Any]:
    """
    Validate password strength.

    Args:
        password: Password to validate

    Returns:
        Dictionary with validation result and any error messages
    """
    result = {
        "is_valid": True,
        "errors": []
    }

    if len(password) < 8:
        result["is_valid"] = False
        result["errors"].append("Password must be at least 8 characters long")

    if not any(c.isupper() for c in password):
        result["is_valid"] = False
        result["errors"].append("Password must contain at least one uppercase letter")

    if not any(c.islower() for c in password):
        result["is_valid"] = False
        result["errors"].append("Password must contain at least one lowercase letter")

    if not any(c.isdigit() for c in password):
        result["is_valid"] = False
        result["errors"].append("Password must contain at least one digit")

    if not any(c in "!@#$%^&*()_+-=[]{}|;:,.<>?" for c in password):
        result["is_valid"] = False
        result["errors"].append("Password must contain at least one special character")

    return result


def validate_user_name(name: str) -> Dict[str, Any]:
    """
    Validate user name (first name or last name).

    Args:
        name: Name to validate

    Returns:
        Dictionary with validation result and any error messages
    """
    result = {
        "is_valid": True,
        "errors": []
    }

    if not name or len(name.strip()) == 0:
        result["is_valid"] = False
        result["errors"].append("Name is required")
        return result

    if len(name) > 50:
        result["is_valid"] = False
        result["errors"].append("Name must be 50 characters or less")

    # Check if name contains only allowed characters (letters, spaces, hyphens, apostrophes)
    import re
    if not re.match(r"^[a-zA-Z\s\-']+$", name):
        result["is_valid"] = False
        result["errors"].append("Name can only contain letters, spaces, hyphens, and apostrophes")

    return result