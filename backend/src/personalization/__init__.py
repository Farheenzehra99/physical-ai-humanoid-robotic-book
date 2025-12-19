"""
Personalization module for chapter content adaptation.

Feature: 007-chapter-personalization
"""

from .service import PersonalizationService
from .models import (
    ChapterPersonalizationRequest,
    ChapterPersonalizationResponse,
    AdaptationSummary,
    PersonalizationLevel
)

__all__ = [
    "PersonalizationService",
    "ChapterPersonalizationRequest",
    "ChapterPersonalizationResponse",
    "AdaptationSummary",
    "PersonalizationLevel"
]
