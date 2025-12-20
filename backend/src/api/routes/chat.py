"""
Chat endpoint for the RAG Agent API.

This module provides the POST /chat endpoint for question answering.
"""

import logging
import time
import uuid

from fastapi import APIRouter, Depends

from src.models import ChatRequest, ChatResponse, Citation, ResponseMetadata
from ...config import get_settings, Settings
from ...models.user import User
from ...database import AsyncSessionLocal
from ...auth.dependencies import get_current_user_optional
from ..middleware.error_handler import ServiceUnavailableError


logger = logging.getLogger(__name__)

router = APIRouter(tags=["Chat"])


@router.post(
    "/chat",
    response_model=ChatResponse,
    summary="Ask a question about the book",
    description="""
    Submit a question about the Physical AI & Humanoid Robotics book.
    The system retrieves relevant context and generates a grounded answer
    with citations to the source material.

    Optionally include selected text from the book to focus the answer
    on a specific passage.
    """,
    responses={
        200: {"description": "Successful response with grounded answer"},
        422: {"description": "Validation error"},
        429: {"description": "Rate limit exceeded"},
        503: {"description": "Service temporarily unavailable"},
        504: {"description": "Gateway timeout"}
    }
)
async def chat(
    request: ChatRequest,
    current_user: User = Depends(get_current_user_optional),
    settings: Settings = Depends(get_settings)
) -> ChatResponse:
    """
    Process a chat request and return a grounded answer.

    Args:
        request: ChatRequest containing question and optional context
        current_user: Currently authenticated user (optional for personalization)
        settings: Application settings

    Returns:
        ChatResponse with answer, citations, and metadata

    Raises:
        ServiceUnavailableError: If external services are unavailable
    """
    start_time = time.time()
    request_id = str(uuid.uuid4())

    logger.info(f"[{request_id}] Processing chat request: {request.question[:100]}...")

    try:
        # Import agent dependencies here to avoid circular imports
        from ...agent.dependencies import get_agent_runner
        from ...models.user_profile import UserProfile
        from sqlalchemy import select

        # Get the agent runner
        agent_runner = get_agent_runner()

        # Get user profile if user is authenticated
        user_profile_data = None
        if current_user:
            # Query the user profile from the database
            async with AsyncSessionLocal() as db:
                profile_query = select(UserProfile).where(UserProfile.user_id == current_user.id)
                profile_result = await db.execute(profile_query)
                profile = profile_result.scalar_one_or_none()

                if profile:
                    user_profile_data = {
                        "programming_level": profile.programming_level,
                        "programming_languages": profile.programming_languages or [],
                        "ai_knowledge_level": profile.ai_knowledge_level,
                        "hardware_experience": profile.hardware_experience,
                        "learning_style": profile.learning_style
                    }

        # Validate that required API keys are available
        settings = get_settings()
        if not settings.is_configured:
            missing_keys = []
            if not settings.openrouter_api_key and not settings.gemini_api_key:
                missing_keys.append("OPENROUTER_API_KEY or GEMINI_API_KEY")
            if not settings.cohere_api_key:
                missing_keys.append("COHERE_API_KEY")
            if not settings.qdrant_url:
                missing_keys.append("QDRANT_URL")
            if not settings.qdrant_api_key:
                missing_keys.append("QDRANT_API_KEY")

            raise ServiceUnavailableError(
                message=f"Missing required API keys: {', '.join(missing_keys)}. Please set them in environment variables.",
                service="configuration",
                retry_after=0
            )

        # Run the agent with the question and optional selected text
        result = await agent_runner.run(
            question=request.question,
            selected_text=request.selected_text,
            top_k=request.top_k,
            user_profile=user_profile_data
        )

        # Calculate processing time
        processing_time_ms = (time.time() - start_time) * 1000

        # Build response
        response = ChatResponse(
            answer=result.answer,
            citations=result.citations,
            metadata=ResponseMetadata(
                request_id=request_id,
                processing_time_ms=processing_time_ms,
                retrieval_count=result.retrieval_count,
                model_used=settings.ai_model_name
            )
        )

        logger.info(
            f"[{request_id}] Request completed in {processing_time_ms:.2f}ms "
            f"with {result.retrieval_count} citations"
        )

        return response

    except ImportError as e:
        logger.error(f"[{request_id}] Agent import failed: {e}")
        raise ServiceUnavailableError(
            message="AI service temporarily unavailable. Please try again later.",
            service="openai",
            retry_after=30
        )
    except Exception as e:
        logger.error(f"[{request_id}] Chat processing failed: {e}")
        # Check for specific error types
        error_msg = str(e).lower()
        if ("openai" in error_msg or "api" in error_msg or "openrouter" in error_msg) or ("gemini" in error_msg or "google" in error_msg):
            service_name = "openrouter" if "openrouter" in error_msg or ("openai" in error_msg and "api" in error_msg) else "gemini"
            raise ServiceUnavailableError(
                message="AI service temporarily unavailable. Please try again later.",
                service=service_name,
                retry_after=30
            )
        elif "qdrant" in error_msg or "vector" in error_msg:
            raise ServiceUnavailableError(
                message="Knowledge base temporarily unavailable. Please try again later.",
                service="qdrant",
                retry_after=30
            )
        elif "cohere" in error_msg or "embedding" in error_msg:
            raise ServiceUnavailableError(
                message="Search service temporarily unavailable. Please try again later.",
                service="cohere",
                retry_after=30
            )
        else:
            raise
