"""
Error response models for the RAG Agent API.

This module defines standardized error response formats.
"""

from typing import Any

from pydantic import BaseModel, Field


class ErrorResponse(BaseModel):
    """
    Standardized error response format.

    Attributes:
        error: Error code/type (e.g., validation_error, service_unavailable)
        message: Human-readable error message
        details: Additional error context
        retry_after: Seconds to wait before retry (for rate limits)
    """

    error: str = Field(..., description="Error code/type")
    message: str = Field(..., description="Human-readable error message")
    details: dict[str, Any] | None = Field(
        default=None,
        description="Additional error context"
    )
    retry_after: int | None = Field(
        default=None,
        ge=0,
        description="Seconds to wait before retry"
    )

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "error": "validation_error",
                    "message": "Question cannot be empty or whitespace-only",
                    "details": {"field": "question", "type": "value_error"}
                },
                {
                    "error": "service_unavailable",
                    "message": "AI service temporarily unavailable. Please try again later.",
                    "retry_after": 30
                },
                {
                    "error": "rate_limit_exceeded",
                    "message": "Too many requests. Please wait before retrying.",
                    "retry_after": 60
                },
                {
                    "error": "timeout",
                    "message": "Request timed out. Please try again with a shorter question."
                }
            ]
        }
    }
