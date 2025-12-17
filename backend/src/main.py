from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from src.api.routers import devices, safety, users, chat, personalization, i18n
from src.database import engine
from src.models import Base

# Create database tables
Base.metadata.create_all(bind=engine)

app = FastAPI(
    title="Physical AI Edge Kit API",
    description="API for managing physical AI devices and educational robotics in educational environments",
    version="1.0.0"
)

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

@app.get("/")
def read_root():
    return {"message": "Physical AI Edge Kit API is running"}

@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "backend"}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)