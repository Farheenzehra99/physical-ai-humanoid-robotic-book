"""
Global error handlers for the RAG Agent API.

This module provides exception handlers for various error types
per the OpenAPI specification.
"""

import logging
from typing import Any

from fastapi import FastAPI, Request, status
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from pydantic import ValidationError

from ...models import ErrorResponse


logger = logging.getLogger(__name__)


class ServiceUnavailableError(Exception):
    """Raised when an external service is unavailable."""

    def __init__(self, message: str, service: str = "unknown", retry_after: int = 30):
        self.message = message
        self.service = service
        self.retry_after = retry_after
        super().__init__(message)


class RateLimitExceededError(Exception):
    """Raised when rate limit is exceeded."""

    def __init__(self, message: str = "Too many requests", retry_after: int = 60):
        self.message = message
        self.retry_after = retry_after
        super().__init__(message)


class RequestTimeoutError(Exception):
    """Raised when a request times out."""

    def __init__(self, message: str = "Request timed out"):
        self.message = message
        super().__init__(message)


def register_error_handlers(app: FastAPI) -> None:
    """
    Register global exception handlers on the FastAPI application.

    Args:
        app: FastAPI application instance
    """

    @app.exception_handler(RequestValidationError)
    async def validation_exception_handler(
        request: Request, exc: RequestValidationError
    ) -> JSONResponse:
        """Handle Pydantic validation errors (422)."""
        errors = exc.errors()
        first_error = errors[0] if errors else {}

        error_response = ErrorResponse(
            error="validation_error",
            message=first_error.get("msg", "Validation error"),
            details={
                "field": ".".join(str(loc) for loc in first_error.get("loc", [])),
                "type": first_error.get("type", "unknown"),
            }
        )

        logger.warning(f"Validation error: {error_response.message}")

        return JSONResponse(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            content=error_response.model_dump(exclude_none=True)
        )

    @app.exception_handler(ValidationError)
    async def pydantic_validation_handler(
        request: Request, exc: ValidationError
    ) -> JSONResponse:
        """Handle Pydantic model validation errors (422)."""
        errors = exc.errors()
        first_error = errors[0] if errors else {}

        error_response = ErrorResponse(
            error="validation_error",
            message=first_error.get("msg", "Validation error"),
            details={
                "field": ".".join(str(loc) for loc in first_error.get("loc", [])),
                "type": first_error.get("type", "unknown"),
            }
        )

        logger.warning(f"Validation error: {error_response.message}")

        return JSONResponse(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            content=error_response.model_dump(exclude_none=True)
        )

    @app.exception_handler(ServiceUnavailableError)
    async def service_unavailable_handler(
        request: Request, exc: ServiceUnavailableError
    ) -> JSONResponse:
        """Handle external service unavailability (503)."""
        error_response = ErrorResponse(
            error="service_unavailable",
            message=exc.message,
            details={"service": exc.service},
            retry_after=exc.retry_after
        )

        logger.error(f"Service unavailable: {exc.service} - {exc.message}")

        return JSONResponse(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            content=error_response.model_dump(exclude_none=True)
        )

    @app.exception_handler(RateLimitExceededError)
    async def rate_limit_handler(
        request: Request, exc: RateLimitExceededError
    ) -> JSONResponse:
        """Handle rate limit exceeded errors (429)."""
        error_response = ErrorResponse(
            error="rate_limit_exceeded",
            message=exc.message,
            retry_after=exc.retry_after
        )

        logger.warning(f"Rate limit exceeded: {exc.message}")

        return JSONResponse(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            content=error_response.model_dump(exclude_none=True),
            headers={"Retry-After": str(exc.retry_after)}
        )

    @app.exception_handler(RequestTimeoutError)
    async def timeout_handler(
        request: Request, exc: RequestTimeoutError
    ) -> JSONResponse:
        """Handle request timeout errors (504)."""
        error_response = ErrorResponse(
            error="timeout",
            message=exc.message
        )

        logger.error(f"Request timeout: {exc.message}")

        return JSONResponse(
            status_code=status.HTTP_504_GATEWAY_TIMEOUT,
            content=error_response.model_dump(exclude_none=True)
        )

    @app.exception_handler(Exception)
    async def general_exception_handler(
        request: Request, exc: Exception
    ) -> JSONResponse:
        """Handle unexpected exceptions (500)."""
        import traceback
        import os

        error_detail = f"{type(exc).__name__}: {str(exc)}"
        logger.exception(f"Unexpected error: {error_detail}")

        # Only expose full error details in debug mode
        debug_mode = os.getenv("DEBUG", "false").lower() == "true"

        if debug_mode:
            error_response = ErrorResponse(
                error="internal_error",
                message=error_detail,
                details={"traceback": traceback.format_exc()}
            )
        else:
            # Production: don't expose internal details
            error_response = ErrorResponse(
                error="internal_error",
                message="An unexpected error occurred. Please try again later."
            )

        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content=error_response.model_dump(exclude_none=True)
        )
