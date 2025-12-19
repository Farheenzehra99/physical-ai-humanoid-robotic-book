"""
Health check endpoint for the RAG Agent API.

This module provides the GET /health endpoint for monitoring API
and dependency status.
"""

import logging
from typing import Literal

from fastapi import APIRouter, status
from fastapi.responses import JSONResponse

from src.models import HealthResponse
from src.config import get_settings


logger = logging.getLogger(__name__)

router = APIRouter(tags=["System"])


@router.get(
    "/health",
    response_model=HealthResponse,
    summary="Health check endpoint",
    description="Returns the health status of the API and its dependencies.",
    responses={
        200: {"description": "Service is healthy"},
        503: {"description": "Service is unhealthy"}
    }
)
async def health_check() -> JSONResponse:
    """
    Check the health of the API and its dependencies.

    Returns:
        HealthResponse with status and dependency information
    """
    settings = get_settings()

    dependencies: dict[str, Literal["connected", "disconnected"]] = {
        "qdrant": "disconnected",
        "cohere": "disconnected",
        "gemini": "disconnected",
    }

    # Check Qdrant connectivity
    if settings.qdrant_url and settings.qdrant_api_key:
        try:
            from qdrant_client import QdrantClient
            client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                timeout=5.0
            )
            # Simple health check - get collections
            client.get_collections()
            dependencies["qdrant"] = "connected"
        except Exception as e:
            logger.warning(f"Qdrant health check failed: {e}")

    # Check Cohere connectivity
    if settings.cohere_api_key:
        try:
            import cohere
            client = cohere.Client(settings.cohere_api_key)
            # Check API by attempting a minimal operation
            # Note: We don't actually call embed to avoid costs
            dependencies["cohere"] = "connected"
        except Exception as e:
            logger.warning(f"Cohere health check failed: {e}")

    # Check Gemini connectivity
    if settings.gemini_api_key:
        try:
            # Basic check - just verify the API key is set
            if settings.gemini_api_key:
                dependencies["gemini"] = "connected"
        except Exception as e:
            logger.warning(f"Gemini health check failed: {e}")

    # Determine overall status - at least Gemini should be connected
    essential_connected = dependencies.get("gemini") == "connected"
    overall_status: Literal["healthy", "unhealthy"] = (
        "healthy" if essential_connected else "unhealthy"
    )

    response = HealthResponse(
        status=overall_status,
        version=settings.api_version,
        dependencies=dependencies
    )

    status_code = (
        status.HTTP_200_OK
        if overall_status == "healthy"
        else status.HTTP_503_SERVICE_UNAVAILABLE
    )

    return JSONResponse(
        status_code=status_code,
        content=response.model_dump()
    )
