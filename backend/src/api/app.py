"""
FastAPI application factory.

This module provides the create_app factory function for building
the FastAPI application with all routes and middleware configured.
"""

import logging
from contextlib import asynccontextmanager
from typing import AsyncGenerator
from pydantic import BaseModel 
from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware

from .routes import health_router, chat_router, personalize_router, translate_router
from ..auth.endpoints import router as auth_router
from .middleware import register_error_handlers
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
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  # Configure appropriately for production
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Register error handlers
    register_error_handlers(app)

    # Include routers
    app.include_router(health_router, prefix=settings.api_prefix)
    app.include_router(chat_router, prefix=settings.api_prefix)
    app.include_router(auth_router, prefix=settings.api_prefix)
    app.include_router(personalize_router, prefix=settings.api_prefix)
    app.include_router(translate_router, prefix=settings.api_prefix)

    # Define a model for frontend compatibility
    class FrontendChatRequest(BaseModel):
        message: str

    # Create a new /chat endpoint that's compatible with frontend requests
    # This endpoint accepts 'message' field and routes through the RAG pipeline
    @app.post("/chat")
    async def frontend_chat_endpoint(
        req: FrontendChatRequest,
        current_user: User = Depends(get_current_user_optional)
    ):
        """
        Frontend-compatible chat endpoint that routes to the RAG pipeline.
        Accepts 'message' field from frontend and converts it to the internal format.
        """
        logger.info(f"Received chat request: {req.message[:50]}...")
        try:
            # Import here to avoid circular imports
            from ..agent.dependencies import get_agent_runner
            from ..config import get_settings
            from ..models.user_profile import UserProfile
            from sqlalchemy import select

            settings = get_settings()

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
            if not settings.is_configured:
                missing_keys = []
                if not settings.gemini_api_key:
                    missing_keys.append("GEMINI_API_KEY")
                if not settings.cohere_api_key:
                    missing_keys.append("COHERE_API_KEY")
                if not settings.qdrant_url:
                    missing_keys.append("QDRANT_URL")
                if not settings.qdrant_api_key:
                    missing_keys.append("QDRANT_API_KEY")

                return {"error": f"Missing required API keys: {', '.join(missing_keys)}", "reply": "Configuration error: Required API keys are missing."}

            # Get the agent runner
            agent_runner = get_agent_runner()

            # Run the agent with the message as the question
            result = await agent_runner.run(
                question=req.message,  # Convert 'message' to 'question'
                top_k=5,
                user_profile=user_profile_data
            )

            # Return the response in a simple format for the frontend
            return {"reply": result.answer}

        except Exception as e:
            import traceback
            error_str = str(e)
            logger.error(f"Chat endpoint error: {e}")
            logger.error(f"Traceback: {traceback.format_exc()}")

            # Handle quota exceeded errors gracefully
            if "RESOURCE_EXHAUSTED" in error_str or "429" in error_str:
                return {
                    "error": "quota_exceeded",
                    "reply": "The AI service is temporarily at capacity. Please try again in a few minutes or contact the administrator to upgrade the API quota."
                }

            return {"error": "Chat service temporarily unavailable", "reply": "Sorry, I'm having trouble responding right now. Please try again later."}

    return app


# Create default app instance
app = create_app()
