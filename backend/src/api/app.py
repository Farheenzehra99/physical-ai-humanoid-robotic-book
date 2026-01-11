"""
FastAPI application factory.

This module provides the create_app factory function for building
the FastAPI application with all routes and middleware configured.

Key features:
- Proper CORS configuration for cross-domain requests
- Lazy router loading with graceful fallback
- Comprehensive error handling
- RAG-powered chat endpoint
"""

import logging
from contextlib import asynccontextmanager
from typing import AsyncGenerator, Optional
from pydantic import BaseModel
from fastapi import FastAPI, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware

from .middleware import register_error_handlers, setup_rate_limiting
from ..config import get_settings
from ..models.user import User
from ..database import AsyncSessionLocal
from ..auth.dependencies import get_current_user_optional

logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """
    Application lifespan context manager.

    Handles startup and shutdown events.
    """
    # Startup
    logger.info("Starting RAG Agent API...")
    settings = get_settings()
    logger.info(f"API Version: {settings.api_version}")
    logger.info(f"Debug Mode: {settings.debug}")

    yield

    # Shutdown
    logger.info("Shutting down RAG Agent API...")


def create_app() -> FastAPI:
    """
    Create and configure the FastAPI application.

    Returns:
        FastAPI: Configured application instance
    """
    settings = get_settings()

    app = FastAPI(
        title=settings.api_title,
        version=settings.api_version,
        description="""
RAG-powered question answering API for the Physical AI & Humanoid Robotics book.

This API uses retrieval-augmented generation to provide grounded, cited answers
to questions about the book content. All responses are strictly based on
retrieved context from the knowledge base.
        """,
        lifespan=lifespan,
        docs_url="/docs",
        redoc_url="/redoc",
        openapi_url="/openapi.json"
    )

    # Configure CORS
    import os
    env_origins = os.getenv("ALLOWED_ORIGINS", "")
    env_origins_list = [o.strip() for o in env_origins.split(",") if o.strip()]
    ALLOWED_ORIGINS = list(set(env_origins_list + [
        "https://physical-ai-humanoid-robotic-book-kmg2eqtlm.vercel.app",
        "https://physical-ai-humanoid-robotic-book-ten.vercel.app",
        "https://physical-ai-humanoid-robotic-book-bpivm88wi.vercel.app",
        "https://physical-ai-robotics.dev",
        "https://*.hf.space",  # Allow all Hugging Face Spaces
        "*.hf.space",         # Alternative format for Hugging Face Spaces
        "https://farheenzehra99-ai-book.hf.space",  # Specific Hugging Face Space
        "http://localhost:3000",
        "http://localhost:3001",
        "http://localhost:8000",
        "http://localhost:8080",
    ]))
    app.add_middleware(
        CORSMiddleware,
        allow_origins=ALLOWED_ORIGINS,
        allow_credentials=True,
        allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS", "PATCH"],
        allow_headers=["*"],
        expose_headers=["Set-Cookie", "X-Request-Id"],
    )

    # Register error handlers
    register_error_handlers(app)

    # Setup rate limiting (optional, graceful fallback if slowapi not installed)
    try:
        setup_rate_limiting(app)
        logger.info("Rate limiting enabled")
    except ImportError:
        logger.warning("slowapi not installed, rate limiting disabled")
    except Exception as e:
        logger.warning(f"Failed to setup rate limiting: {e}")

    # Lazy-load routers with try-except for graceful degradation
    try:
        from .routes import health_router
        app.include_router(health_router)
    except Exception as e:
        logger.warning(f"Failed to include health_router: {e}")

    try:
        from .routes import chat_router
        app.include_router(chat_router)
    except Exception as e:
        logger.warning(f"Failed to include chat_router: {e}")

    try:
        from .routes import personalize_router
        app.include_router(personalize_router)
    except Exception as e:
        logger.warning(f"Failed to include personalize_router: {e}")

    try:
        from .routes import translate_router
        app.include_router(translate_router)
    except Exception as e:
        logger.warning(f"Failed to include translate_router: {e}")


    from ..auth.client import auth_client
    from ..auth.endpoints import router as auth_router
    app.include_router(auth_router)

    # Root endpoint for health checks
    @app.get("/")
    async def root():
        """Root endpoint for health checks and API info."""
        return {
            "status": "ok",
            "service": "Physical AI & Humanoid Robotics RAG API",
            "version": settings.api_version,
            "docs": "/docs"
        }

    # Frontend chat endpoint
    class FrontendChatRequest(BaseModel):
        """Request model for frontend chat endpoint."""
        message: str

    @app.post("/chat")
    async def frontend_chat_endpoint(
        req: FrontendChatRequest,
        current_user: Optional[User] = Depends(get_current_user_optional)
    ):
        """
        Frontend-compatible chat endpoint that routes to the RAG pipeline.

        Accepts 'message' field from frontend and converts to internal format.

        Returns:
            200: Successful response with 'reply' field
            503: Service unavailable (quota exceeded, etc.)
        """
        logger.info(f"Received chat request: {req.message[:50]}...")
        try:
            from ..agent.dependencies import get_agent_runner
            from ..config import get_settings
            from ..models.user_profile import UserProfile
            from sqlalchemy import select

            settings = get_settings()
            user_profile_data = None

            # Get user profile for personalization
            if current_user:
                try:
                    async with AsyncSessionLocal() as db:
                        profile_query = select(UserProfile).where(
                            UserProfile.user_id == current_user.id
                        )
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
                except Exception as profile_error:
                    logger.warning(f"Failed to fetch user profile: {profile_error}")

            # Check configuration
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

                return {
                    "error": f"Missing required API keys: {', '.join(missing_keys)}",
                    "reply": "Configuration error: Required API keys are missing."
                }

            # Run the RAG agent
            agent_runner = get_agent_runner()
            result = await agent_runner.run(
                question=req.message,
                top_k=5,
                user_profile=user_profile_data
            )
            return {"reply": result.answer}

        except Exception as e:
            import traceback
            error_str = str(e)
            logger.error(f"Chat endpoint error: {e}")
            logger.error(f"Traceback: {traceback.format_exc()}")

            # Handle quota/rate limit errors
            if "RESOURCE_EXHAUSTED" in error_str or "429" in error_str:
                return {
                    "error": "quota_exceeded",
                    "reply": "The AI service is temporarily at capacity. Please try again later."
                }

            return {
                "error": "Chat service temporarily unavailable",
                "reply": "Sorry, I'm having trouble responding right now. Please try again later."
            }

    return app


# Create default app instance
app = create_app()

