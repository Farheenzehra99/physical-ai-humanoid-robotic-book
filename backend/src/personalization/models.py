from pydantic import BaseModel, Field
from typing import Optional
from enum import Enum


class PersonalizationLevel(str, Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class ChapterPersonalizationRequest(BaseModel):
    """Request model for chapter personalization."""
    chapter_slug: str = Field(
        ...,
        description="URL slug of the chapter",
        example="module-01-foundations/chapter-01/page-01"
    )
    chapter_content: str = Field(
        ...,
        max_length=51200,  # 50KB limit
        description="Original chapter content in Markdown"
    )
    chapter_title: Optional[str] = Field(
        None,
        description="Chapter title for context"
    )


class AdaptationSummary(BaseModel):
    """Summary of adaptations made."""
    level_adjustment: str
    processing_time_ms: int
    content_length_original: int
    content_length_personalized: int


class ChapterPersonalizationResponse(BaseModel):
    """Response model for chapter personalization."""
    success: bool
    personalized_content: str
    adaptation_summary: AdaptationSummary