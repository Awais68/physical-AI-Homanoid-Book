#!/bin/bash
set -e

echo "=== Starting Humainoid Robotics Platform ==="
echo "Starting FastAPI backend on port 8000..."

# Start FastAPI backend in background
python -m uvicorn backend:app --host 0.0.0.0 --port 8000 &
BACKEND_PID=$!

# Wait for backend to start
echo "Waiting for backend to start..."
for i in $(seq 1 30); do
    if curl -s http://localhost:8000/health > /dev/null 2>&1; then
        echo "Backend is ready!"
        break
    fi
    sleep 1
done

echo "Starting Streamlit chatbot on port 7860..."

# Start Streamlit on port 7860 (HF Spaces expected port)
streamlit run app.py \
    --server.port=7860 \
    --server.address=0.0.0.0 \
    --server.headless=true \
    --browser.gatherUsageStats=false \
    --server.fileWatcherType=none

# If Streamlit exits, clean up backend
kill $BACKEND_PID 2>/dev/null
