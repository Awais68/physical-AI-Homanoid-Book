"""FastAPI backend for RAG chatbot - runs on port 8000 internally."""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.api.routers import chat

app = FastAPI(
    title="Humainoid Robotics RAG API",
    description="RAG-powered chatbot API for Physical AI & Humanoid Robotics",
    version="1.0.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(chat.router, prefix="/api", tags=["chat"])


@app.get("/")
def read_root():
    return {"message": "Humainoid Robotics RAG API is running"}


@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "rag-backend"}


@app.get("/api/health")
def api_health():
    return {"status": "healthy", "service": "rag-backend"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
