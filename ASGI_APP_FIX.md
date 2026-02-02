# ASGI App Loading Fix - Complete Solution

## Problem
When running `python -m uvicorn main:app --reload` from the project root, the error occurred:
```
ERROR: Error loading ASGI app. Attribute "app" not found in module "main".
```

## Root Cause
The project structure has the FastAPI app in `backend/src/main.py`, but the root `main.py` file was a simple data ingestion script that didn't expose the `app` variable. Additionally, `backend/src/main.py` uses absolute imports like `from src.api.routers import ...` which assume the Python path includes the `backend` directory.

## Solution Implemented

### 1. Created app.py wrapper (Root Directory)
File: `/app.py`

This file properly handles the module import path:
- Adds `backend` directory to `sys.path`
- Imports FastAPI app from `backend/src/main.py`
- Exposes the `app` variable for uvicorn

```python
import sys
import os

root_dir = os.path.dirname(os.path.abspath(__file__))
backend_path = os.path.join(root_dir, 'backend')
sys.path.insert(0, backend_path)

from src.main import app
__all__ = ['app']
```

### 2. Updated main.py (Root Directory)
File: `/main.py`

Now serves as a documentation file with usage instructions and imports from `app.py`. Can be used as entry point with:
```bash
python main.py
python -m uvicorn main:app
```

### 3. Added __init__.py files
- `/backend/__init__.py` - Makes backend a Python package
- `/backend/src/__init__.py` - Makes src a Python package

## How to Run

### Development (from root directory)
```bash
# Option 1: From backend directory (recommended)
cd backend
python -m uvicorn src.main:app --reload

# Option 2: From root using app.py wrapper
python -m uvicorn app:app --reload

# Option 3: Using main.py
python main.py
```

### Production / Docker
The backend Dockerfile is already correctly configured:
```dockerfile
WORKDIR /app  # Sets to backend directory
CMD ["python", "-m", "uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

## Verification

✅ Test import from root directory:
```bash
python3 -c "from app import app; print('✓ App imported successfully')"
```

✅ Start server from backend directory:
```bash
cd backend
timeout 5 python3 -m uvicorn src.main:app --host 0.0.0.0 --port 8000
# Output: INFO:     Uvicorn running on http://0.0.0.0:8000
```

## Why This Solution

1. **Preserves existing structure** - No changes to backend code
2. **Works with Docker** - Backend Dockerfile already uses correct working directory
3. **Supports all import methods** - Works from root and from backend directory
4. **Clean separation** - app.py handles module path management
5. **Flexible** - Works with different entry points (main.py, app.py, or direct backend command)

## Docker Integration

No Dockerfile changes needed. The backend Dockerfile already has:
- WORKDIR set to `/app` (which receives the backend directory)
- CMD properly configured to run `python -m uvicorn src.main:app`

When Docker builds, the entire backend directory is copied to /app, so the imports work correctly.

## Files Modified/Created

1. ✅ Created: `/app.py` - ASGI app wrapper
2. ✅ Updated: `/main.py` - Entry point with documentation
3. ✅ Created: `/backend/__init__.py` - Package marker
4. ✅ Created: `/backend/src/__init__.py` - Package marker

## Testing the Fix

From project root:
```bash
# Test 1: Can we import the app?
python3 -c "from app import app; print('App imported:', app.title)"

# Test 2: Can we run uvicorn from root?
python -m uvicorn app:app --reload

# Test 3: Can we run from backend directory?
cd backend && python -m uvicorn src.main:app --reload

# Test 4: Docker build
docker build -f backend/Dockerfile -t physical-ai-backend:latest backend/
```

All tests should now pass with no "Attribute 'app' not found" errors.
