from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from src.api.routers import devices, safety, users, chat, personalization, i18n, translations, text_processing
from src.api.translation_middleware import TranslationMiddleware

# Try to initialize database, but don't fail if psycopg2 is missing
try:
    from src.config.database import engine
    from src.models import Base
    Base.metadata.create_all(bind=engine)
    print("✓ Database initialized")
except ImportError as e:
    print(f"Database connection failed: {e}. Running without database.")
except Exception as e:
    print(f"Database setup error: {e}. Running without database.")

app = FastAPI(
    title="Physical AI Edge Kit API",
    description="API for managing physical AI devices and educational robotics in educational environments",
    version="1.0.0"
)

# Add Translation middleware
app.add_middleware(TranslationMiddleware)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routers
app.include_router(devices.router, prefix="/api", tags=["devices"])
app.include_router(safety.router, prefix="/api", tags=["safety"])
app.include_router(users.router, prefix="/api", tags=["users"])
app.include_router(chat.router, prefix="/api", tags=["chat"])
app.include_router(personalization.router, prefix="/api", tags=["personalization"])
app.include_router(i18n.router, prefix="/api", tags=["i18n"])
app.include_router(translations.router, prefix="/api", tags=["translations"])
app.include_router(text_processing.router, prefix="/api", tags=["text-processing"])

@app.get("/")
def read_root():
    return {"message": "Physical AI Edge Kit API is running"}

@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "backend"}

@app.get("/diagnostics")
def diagnostics():
    """Diagnostic endpoint to check service connections."""
    from src.clients.qdrant_client import qdrant_client
    from src.clients.gemini_client import gemini_client
    from src.clients.openai_client import openai_client
    from src.config.settings import settings
    
    return {
        "status": "running",
        "services": {
            "qdrant": {
                "connected": qdrant_client is not None,
                "url": settings.QDRANT_URL,
                "collection": settings.QDRANT_COLLECTION,
                "has_api_key": bool(settings.QDRANT_API_KEY)
            },
            "gemini": {
                "connected": gemini_client is not None,
                "model": settings.GEMINI_CHAT_MODEL,
                "has_api_key": bool(settings.GEMINI_API_KEY)
            },
            "openai": {
                "connected": openai_client is not None,
                "model": settings.OPENAI_CHAT_MODEL,
                "has_api_key": bool(settings.OPENAI_API_KEY)
            },
            "cohere": {
                "has_api_key": bool(settings.COHERE_API_KEY),
                "model": settings.COHERE_EMBEDDING_MODEL
            }
        }
    }

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)