"""
Request models for the RAG Agent API.

This module defines Pydantic models for incoming API requests.
"""

from pydantic import BaseModel, Field, field_validator


class ChatRequest(BaseModel):
    """
    Incoming chat request containing user question and optional context.

    Attributes:
        question: User question about the book (1-2000 chars)
        selected_text: Optional selected text from book for focused answers
        top_k: Number of retrieval results to use (1-20, default: 5)
    """

    question: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User question about the book"
    )
    selected_text: str | None = Field(
        default=None,
        max_length=5000,
        description="Optional selected text from book for focused answers"
    )
    top_k: int = Field(
        default=5,
        ge=1,
        le=20,
        description="Number of retrieval results to use"
    )

    @field_validator("question")
    @classmethod
    def validate_question_not_whitespace(cls, v: str) -> str:
        """Validate that question is not empty or whitespace-only."""
        if not v.strip():
            raise ValueError("Question cannot be empty or whitespace-only")
        return v.strip()

    model_config = {
        "json_schema_extra": {
            "examples": [
                {
                    "question": "What is Physical AI?",
                },
                {
                    "question": "Can you explain this further?",
                    "selected_text": "Physical AI refers to artificial intelligence systems that interact with the physical world through sensors and actuators.",
                    "top_k": 8
                }
            ]
        }
    }
