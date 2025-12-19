import logging
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from ...translation.models import (
    TranslationRequest,
    TranslationResponse,
    TranslationSummary
)
from ...translation.service import TranslationService
from ...database import get_async_db
from ...auth.dependencies import get_current_active_user
from ...better_auth_mock import User

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/translate", tags=["translation"])

# Singleton service instance
_translation_service = None


def get_translation_service() -> TranslationService:
    global _translation_service
    if _translation_service is None:
        _translation_service = TranslationService()
    return _translation_service


@router.post("/chapter", response_model=TranslationResponse)
async def translate_chapter(
    request: TranslationRequest,
    current_user: User = Depends(get_current_active_user),
    db: AsyncSession = Depends(get_async_db),
    service: TranslationService = Depends(get_translation_service)
):
    """
    Translate chapter content to Urdu for the authenticated user.

    Translates the chapter content to Urdu while preserving:
    - Document structure (headings, paragraphs, lists)
    - Code blocks (in English)
    - Technical identifiers (ROS topics, CLI commands, APIs, file paths)
    """
    try:
        # Validate input
        if not request.chapter_content.strip():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Chapter content cannot be empty"
            )

        # Translate content
        translated_content, translation_summary = await service.translate_chapter(
            chapter_content=request.chapter_content,
            chapter_title=request.chapter_title or "",
        )

        return TranslationResponse(
            success=True,
            translated_content=translated_content,
            translation_summary=TranslationSummary(**translation_summary)
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Translation failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Translation service temporarily unavailable"
        )