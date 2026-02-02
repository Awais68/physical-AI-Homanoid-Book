#!/usr/bin/env python3
"""
FastAPI ASGI Application Entry Point

This file provides multiple ways to run the backend:

OPTION 1: Run from backend directory (Recommended for development)
    cd backend
    python -m uvicorn src.main:app --reload

OPTION 2: Run from root using app.py wrapper
    python -m uvicorn app:app --reload

OPTION 3: Run using this file
    python main.py  # or python -m uvicorn main:app

OPTION 4: Docker container (production)
    Uses app.py and runs from /app directory
    CMD: python -m uvicorn src.main:app --host 0.0.0.0 --port 8000

Environment Variables:
    HOST: Server host (default: 0.0.0.0)
    PORT: Server port (default: 8000)
    All settings from backend/src/config/settings.py apply
"""

import sys
import os

# Import FastAPI app from app.py which handles backend module imports
from app import app

if __name__ == "__main__":
    import uvicorn
    
    # Read environment variables for host/port
    host = os.getenv("HOST", "0.0.0.0")
    port = int(os.getenv("PORT", "8000"))
    
    # Run the server
    uvicorn.run(
        app,
        host=host,
        port=port,
        log_level="info"
    )



if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
