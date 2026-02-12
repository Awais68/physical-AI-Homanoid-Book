FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install Qdrant
RUN wget https://github.com/qdrant/qdrant/releases/download/v1.7.4/qdrant-x86_64-unknown-linux-gnu.tar.gz \
    && tar -xzf qdrant-x86_64-unknown-linux-gnu.tar.gz \
    && mv qdrant /usr/local/bin/ \
    && rm qdrant-x86_64-unknown-linux-gnu.tar.gz

# Copy backend code
COPY backend/ /app/backend/
COPY frontend/docs/ /app/frontend/docs/

# Install Python dependencies
COPY backend/requirements.txt /app/backend/
RUN pip install --no-cache-dir -r /app/backend/requirements.txt

# Copy startup script
COPY start-hf.sh /app/
RUN chmod +x /app/start-hf.sh

# Environment variables (will be overridden by HF Secrets)
ENV QDRANT_URL=http://localhost:6333
ENV QDRANT_COLLECTION=rag_chatbot
ENV PYTHONUNBUFFERED=1

EXPOSE 7860

CMD ["/app/start-hf.sh"]
