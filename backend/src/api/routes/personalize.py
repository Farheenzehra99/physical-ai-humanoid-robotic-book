import logging
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from ...personalization.models import (
    ChapterPersonalizationRequest,
    ChapterPersonalizationResponse,
    AdaptationSummary
)
from ...personalization.service import PersonalizationService
from ...database import get_async_db
from ...database.crud import get_user_preferences
from ...auth.dependencies import get_current_active_user
from ...better_auth_mock import User

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/personalize", tags=["personalization"])

# Singleton service instance
_personalization_service = None

def get_personalization_service() -> PersonalizationService:
    global _personalization_service
    if _personalization_service is None:
        _personalization_service = PersonalizationService()
    return _personalization_service

@router.post("/chapter", response_model=ChapterPersonalizationResponse)
async def personalize_chapter(
    request: ChapterPersonalizationRequest,
    current_user: User = Depends(get_current_active_user),
    db: AsyncSession = Depends(get_async_db),
    service: PersonalizationService = Depends(get_personalization_service)
):
    """
    Personalize chapter content for the authenticated user.

    Adapts the chapter content based on the user's:
    - Programming level (beginner/intermediate/advanced)
    - Known programming languages
    - AI knowledge level
    - Hardware experience
    - Learning style preference
    """
    try:
        # Fetch user preferences
        user_preferences = await get_user_preferences(db, current_user.id)

        if not user_preferences:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="User profile not found. Please complete your profile first."
            )

        # Personalize content
        personalized_content, adaptation_summary = await service.personalize_chapter(
            chapter_content=request.chapter_content,
            chapter_title=request.chapter_title or "",
            user_profile=user_preferences
        )

        return ChapterPersonalizationResponse(
            success=True,
            personalized_content=personalized_content,
            adaptation_summary=AdaptationSummary(**adaptation_summary)
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Personalization failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Personalization service temporarily unavailable"
        )