FROM python:3.11-slim

WORKDIR /app

# Install curl for health checks
RUN apt-get update && apt-get install -y --no-install-recommends curl && \
    rm -rf /var/lib/apt/lists/*

# Copy and install requirements
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Expose HF Spaces expected port
EXPOSE 7860

# Run unified FastAPI app on port 7860
CMD ["python", "-m", "uvicorn", "app_fastapi:app", "--host", "0.0.0.0", "--port", "7860"]
