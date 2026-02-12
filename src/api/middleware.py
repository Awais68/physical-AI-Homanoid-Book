"""API middleware for error handling, logging, and rate limiting."""

import asyncio
import logging
import time
import uuid
from collections.abc import Callable
from typing import Any

from fastapi import FastAPI, Request, Response
from fastapi.responses import JSONResponse
from pydantic import ValidationError

from src.config import get_settings
from src.text_processor.models import ErrorResponse, ErrorType

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# Rate limiting semaphore (initialized lazily)
_rate_limiter: asyncio.Semaphore | None = None


def get_rate_limiter() -> asyncio.Semaphore:
    """Get or create the rate limiting semaphore.

    Returns:
        Semaphore for rate limiting concurrent requests
    """
    global _rate_limiter
    if _rate_limiter is None:
        settings = get_settings()
        _rate_limiter = asyncio.Semaphore(settings.max_concurrent_requests)
    return _rate_limiter


async def add_request_id(request: Request, call_next: Callable[..., Any]) -> Response:
    """Middleware to add unique request ID to each request.

    Args:
        request: FastAPI request object
        call_next: Next middleware or route handler

    Returns:
        Response with X-Request-ID header
    """
    request_id = str(uuid.uuid4())
    request.state.request_id = request_id

    # Log request start
    logger.info(
        f"Request started: {request.method} {request.url.path}",
        extra={"request_id": request_id},
    )

    start_time = time.time()

    response = await call_next(request)

    # Calculate processing time
    process_time = (time.time() - start_time) * 1000  # Convert to ms

    # Add headers
    response.headers["X-Request-ID"] = request_id
    response.headers["X-Process-Time-Ms"] = f"{process_time:.2f}"

    # Log request completion
    logger.info(
        f"Request completed: {request.method} {request.url.path} "
        f"status={response.status_code} time={process_time:.2f}ms",
        extra={"request_id": request_id},
    )

    return response


def create_error_response(
    error_type: ErrorType,
    message: str,
    details: dict[str, Any] | None = None,
    status_code: int = 400,
) -> JSONResponse:
    """Create a standardized error response.

    Args:
        error_type: Type of error
        message: Human-readable error message
        details: Additional error context
        status_code: HTTP status code

    Returns:
        JSONResponse with error details
    """
    error = ErrorResponse(
        error_type=error_type,
        message=message,
        details=details,
    )
    return JSONResponse(
        status_code=status_code,
        content=error.model_dump(),
    )


def setup_exception_handlers(app: FastAPI) -> None:
    """Set up global exception handlers for the application.

    Args:
        app: FastAPI application instance
    """

    @app.exception_handler(ValidationError)
    async def validation_error_handler(
        request: Request, exc: ValidationError
    ) -> JSONResponse:
        """Handle Pydantic validation errors."""
        logger.warning(
            f"Validation error: {exc.errors()}",
            extra={"request_id": getattr(request.state, "request_id", "unknown")},
        )
        return create_error_response(
            error_type=ErrorType.VALIDATION_ERROR,
            message="Invalid request data",
            details={"errors": exc.errors()},
            status_code=400,
        )

    @app.exception_handler(ValueError)
    async def value_error_handler(request: Request, exc: ValueError) -> JSONResponse:
        """Handle value errors (validation failures)."""
        logger.warning(
            f"Value error: {str(exc)}",
            extra={"request_id": getattr(request.state, "request_id", "unknown")},
        )
        return create_error_response(
            error_type=ErrorType.VALIDATION_ERROR,
            message=str(exc),
            status_code=400,
        )

    @app.exception_handler(Exception)
    async def general_exception_handler(
        request: Request, exc: Exception
    ) -> JSONResponse:
        """Handle unexpected exceptions."""
        request_id = getattr(request.state, "request_id", "unknown")
        logger.error(
            f"Unexpected error: {type(exc).__name__}: {str(exc)}",
            extra={"request_id": request_id},
            exc_info=True,
        )
        return create_error_response(
            error_type=ErrorType.PROCESSING_ERROR,
            message="An unexpected error occurred. Please try again.",
            details={"request_id": request_id},
            status_code=500,
        )


class RateLimitExceeded(Exception):
    """Exception raised when rate limit is exceeded."""

    pass


async def rate_limit_middleware(
    request: Request, call_next: Callable[..., Any]
) -> Response:
    """Middleware to apply rate limiting to requests.

    Args:
        request: FastAPI request object
        call_next: Next middleware or route handler

    Returns:
        Response or 503 if rate limited
    """
    semaphore = get_rate_limiter()

    # Try to acquire semaphore without blocking
    acquired = semaphore.locked() is False

    if not acquired and semaphore._value == 0:  # type: ignore[attr-defined]
        # All slots are taken, return 503
        logger.warning(
            "Rate limit exceeded",
            extra={"request_id": getattr(request.state, "request_id", "unknown")},
        )
        return create_error_response(
            error_type=ErrorType.SERVICE_UNAVAILABLE,
            message="Service is temporarily unavailable. Please retry after 30 seconds.",
            details={"retry_after": 30},
            status_code=503,
        )

    async with semaphore:
        return await call_next(request)
