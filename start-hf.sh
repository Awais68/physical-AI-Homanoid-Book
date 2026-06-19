#!/bin/bash
set -e

echo "🚀 Starting Physical AI Backend with Qdrant..."

# Start Qdrant in background
echo "📦 Starting Qdrant vector database..."
qdrant &
QDRANT_PID=$!

# Wait for Qdrant to be ready
echo "⏳ Waiting for Qdrant to start..."
sleep 5

# Check if Qdrant is running
if curl -s http://localhost:6333/ > /dev/null; then
    echo "✅ Qdrant is running"
else
    echo "⚠️ Qdrant failed to start, continuing anyway..."
fi

# Check if documents need to be ingested (non-blocking)
echo "📚 Checking document collection..."
python3 backend/check_and_ingest.py || echo "⚠️ Document check failed, continuing anyway..."

# Start the FastAPI backend
echo "🚀 Starting FastAPI backend..."
cd backend
exec uvicorn src.main:app --host 0.0.0.0 --port 7860
