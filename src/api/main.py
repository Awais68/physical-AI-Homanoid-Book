"""FastAPI application entry point."""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from src.api.middleware import (
    add_request_id,
    setup_exception_handlers,
)
from src.config import get_settings

# Get application settings
settings = get_settings()

# Create FastAPI application
app = FastAPI(
    title=settings.app_name,
    description=(
        "AI-powered text processing service that clarifies unclear text and translates "
        "it to target languages. Returns structured JSON output with both original and "
        "processed text."
    ),
    version=settings.app_version,
    docs_url="/docs",
    redoc_url="/redoc",
    openapi_url="/openapi.json",
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add request ID middleware
app.middleware("http")(add_request_id)

# Setup exception handlers
setup_exception_handlers(app)

# Import and include routes (imported here to avoid circular imports)
from src.api.routes import router  # noqa: E402

app.include_router(router)


@app.on_event("startup")
async def startup_event() -> None:
    """Application startup event handler."""
    import logging

    logger = logging.getLogger(__name__)
    logger.info(f"Starting {settings.app_name} v{settings.app_version}")
    logger.info(f"Server running on {settings.host}:{settings.port}")


@app.on_event("shutdown")
async def shutdown_event() -> None:
    """Application shutdown event handler."""
    import logging

    logger = logging.getLogger(__name__)
    logger.info("Shutting down application")


# Entry point for uvicorn
if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "src.api.main:app",
        host=settings.host,
        port=settings.port,
        reload=True,
    )
