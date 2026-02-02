"""FastAPI application entry point."""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse

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


@app.get("/", response_class=HTMLResponse, include_in_schema=False)
async def root():
    """Root endpoint with welcome page."""
    return """
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Physical AI Edge Kit API</title>
        <style>
            * { margin: 0; padding: 0; box-sizing: border-box; }
            body {
                font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, sans-serif;
                background: linear-gradient(135deg, #1a1a2e 0%, #16213e 50%, #0f3460 100%);
                min-height: 100vh;
                display: flex;
                justify-content: center;
                align-items: center;
                color: #fff;
            }
            .container {
                text-align: center;
                padding: 40px;
                background: rgba(255, 255, 255, 0.1);
                border-radius: 20px;
                backdrop-filter: blur(10px);
                box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3);
                max-width: 600px;
            }
            h1 {
                font-size: 2.5rem;
                margin-bottom: 10px;
                background: linear-gradient(90deg, #00d4ff, #7c3aed);
                -webkit-background-clip: text;
                -webkit-text-fill-color: transparent;
            }
            .emoji { font-size: 4rem; margin-bottom: 20px; }
            p { color: #a0aec0; margin-bottom: 30px; font-size: 1.1rem; }
            .links { display: flex; gap: 20px; justify-content: center; flex-wrap: wrap; }
            a {
                display: inline-block;
                padding: 12px 24px;
                background: linear-gradient(90deg, #7c3aed, #00d4ff);
                color: white;
                text-decoration: none;
                border-radius: 8px;
                font-weight: 600;
                transition: transform 0.2s, box-shadow 0.2s;
            }
            a:hover {
                transform: translateY(-2px);
                box-shadow: 0 4px 20px rgba(124, 58, 237, 0.4);
            }
            .version { margin-top: 30px; color: #64748b; font-size: 0.9rem; }
        </style>
    </head>
    <body>
        <div class="container">
            <div class="emoji">🤖</div>
            <h1>Physical AI Edge Kit API</h1>
            <p>AI-powered text processing service for clarification and translation</p>
            <div class="links">
                <a href="/docs">📚 Swagger Docs</a>
                <a href="/redoc">📖 ReDoc</a>
                <a href="/health">💚 Health Check</a>
                <a href="/languages">🌍 Languages</a>
            </div>
            <p class="version">v1.0.0</p>
        </div>
    </body>
    </html>
    """


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
