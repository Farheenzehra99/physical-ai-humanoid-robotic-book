from pydantic import BaseModel
from typing import Optional, Dict, Any


class TranslationRequest(BaseModel):
    """Request model for chapter translation"""
    chapter_slug: str
    chapter_content: str
    chapter_title: Optional[str] = ""


class TranslationSummary(BaseModel):
    """Summary of translation results"""
    processing_time_ms: int
    content_length_original: int
    content_length_translated: int
    preserved_elements_count: int
    preserved_technical_elements_count: Optional[int] = 0


class TranslationResponse(BaseModel):
    """Response model for chapter translation"""
    success: bool
    translated_content: str
    translation_summary: TranslationSummary