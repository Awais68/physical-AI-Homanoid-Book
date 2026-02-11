from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv(os.path.join(os.path.dirname(__file__), '..', '.env'))

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI Backend",
    description="Backend API for Physical AI Chatbot",
    version="1.0.0"
)

# Configure CORS
origins = [
    "http://localhost:3000",
    "http://localhost:8000",
    "https://awais68.github.io",
    "https://huggingface.co",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models
class ChatMessage(BaseModel):
    message: str
    history: list = []

class ChatResponse(BaseModel):
    response: str
    sources: list = []

# Routes
@app.get("/")
async def root():
    """Root endpoint"""
    return {"message": "Physical AI Backend API", "status": "ok"}

@app.get("/health")
async def health():
    """Health check endpoint"""
    return {"status": "healthy"}

@app.post("/chat")
async def chat(message: ChatMessage):
    """Chat endpoint for querying the AI assistant"""
    try:
        # Import agent here to avoid circular imports
        from agent import agent, Runner
        
        result = Runner.run_sync(
            agent,
            input=message.message,
        )
        
        return ChatResponse(
            response=result.final_output,
            sources=[]
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/ingest")
async def ingest(force: bool = False):
    """Ingest documents endpoint"""
    try:
        from ingest_main import ingest_book
        ingest_book(force_recreate=force)
        return {"message": "Ingestion completed successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000, reload=True)
