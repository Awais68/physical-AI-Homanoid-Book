# ASGI app wrapper - imports FastAPI app from backend
# Run with: python -m uvicorn app:app --reload

import sys
import os

# Get absolute paths
root_dir = os.path.dirname(os.path.abspath(__file__))
backend_path = os.path.join(root_dir, 'backend')
backend_src_path = os.path.join(backend_path, 'src')

# Add backend to path for relative imports
sys.path.insert(0, backend_path)

# Import the FastAPI app from backend/src/main.py
# Since backend is in sys.path, "src.main" will resolve correctly
from src.main import app

# Expose for uvicorn
__all__ = ['app']


