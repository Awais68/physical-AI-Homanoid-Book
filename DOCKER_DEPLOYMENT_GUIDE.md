# Docker Deployment Guide

This guide covers containerizing and deploying the Physical AI Humanoid Book application using Docker.

## Table of Contents
1. [Local Development](#local-development)
2. [Production Build](#production-build)
3. [Hugging Face Spaces Deployment](#hugging-face-spaces-deployment)
4. [Docker Commands Reference](#docker-commands-reference)
5. [Troubleshooting](#troubleshooting)

---

## Local Development

### Prerequisites
- Docker (version 20.10+)
- Docker Compose (version 2.0+)
- Git

### Setup

1. **Clone the repository**
```bash
git clone <your-repo-url>
cd physical-AI-Homanoid-Book-main
```

2. **Configure environment variables**
```bash
cp .env.example .env
```

Edit `.env` with your API keys:
```bash
OPENAI_API_KEY=sk-your-key-here
GEMINI_API_KEY=your-gemini-key-here
COHERE_API_KEY=your-cohere-key-here
QDRANT_API_KEY=your-qdrant-key-here
```

3. **Build and start containers**
```bash
# Development environment with live reload
docker-compose -f docker-compose.dev.yml up --build

# Or production environment
docker-compose -f docker-compose.prod.yml up --build
```

4. **Access services**
- Frontend: http://localhost:3000
- Backend API: http://localhost:8000
- API Docs: http://localhost:8000/docs
- Qdrant Dashboard: http://localhost:6333/dashboard
- PostgreSQL: localhost:5432

### Useful Development Commands

```bash
# View logs
docker-compose -f docker-compose.dev.yml logs -f backend

# Run backend tests
docker-compose -f docker-compose.dev.yml exec backend pytest

# Access backend shell
docker-compose -f docker-compose.dev.yml exec backend bash

# Access database
docker-compose -f docker-compose.dev.yml exec postgres psql -U postgres -d edgekit_db

# Stop all services
docker-compose -f docker-compose.dev.yml down

# Remove all data (fresh start)
docker-compose -f docker-compose.dev.yml down -v
```

---

## Production Build

### Building Docker Images

1. **Build backend image**
```bash
docker build -t physical-ai-backend:latest ./backend
```

2. **Build frontend image**
```bash
docker build -t physical-ai-frontend:latest ./frontend
```

3. **Tag for registry**
```bash
# For Docker Hub
docker tag physical-ai-backend:latest yourusername/physical-ai-backend:latest
docker tag physical-ai-frontend:latest yourusername/physical-ai-frontend:latest

# Push to registry
docker push yourusername/physical-ai-backend:latest
docker push yourusername/physical-ai-frontend:latest
```

### Production Deployment

1. **Prepare environment**
```bash
cp .env.example .env.production
# Edit with production values
```

2. **Deploy using docker-compose**
```bash
docker-compose -f docker-compose.prod.yml \
  --env-file .env.production \
  up -d
```

3. **Verify deployment**
```bash
# Check all services are running
docker-compose -f docker-compose.prod.yml ps

# Check backend health
curl http://localhost:8000/health

# Check diagnostics
curl http://localhost:8000/diagnostics
```

---

## Hugging Face Spaces Deployment

### Prerequisites
- Hugging Face account
- Hugging Face CLI installed: `pip install huggingface-hub`

### Option 1: Using docker-compose.yml

1. **Create Hugging Face Space**
   - Go to https://huggingface.co/spaces
   - Click "Create new Space"
   - Choose "Docker" as the space runtime
   - Select a name and make it public/private as needed

2. **Set up local repository**
```bash
# Clone your HF space
git clone https://huggingface.co/spaces/YOUR_USERNAME/YOUR_SPACE_NAME
cd YOUR_SPACE_NAME

# Copy Dockerfiles and compose files
cp ../physical-AI-Homanoid-Book-main/backend/Dockerfile ./Dockerfile.backend
cp ../physical-AI-Homanoid-Book-main/frontend/Dockerfile ./Dockerfile.frontend
cp ../physical-AI-Homanoid-Book-main/docker-compose.prod.yml ./docker-compose.yml
```

3. **Create .env file in Space**
```bash
# Create .env in the space directory
cat > .env << EOF
POSTGRES_DB=edgekit_db
POSTGRES_USER=postgres
POSTGRES_PASSWORD=$(openssl rand -base64 32)
QDRANT_API_KEY=${QDRANT_API_KEY}
OPENAI_API_KEY=${OPENAI_API_KEY}
GEMINI_API_KEY=${GEMINI_API_KEY}
COHERE_API_KEY=${COHERE_API_KEY}
EOF
```

4. **Set Secrets in HF Space**
   - Go to Space Settings
   - Add Secrets:
     - `QDRANT_API_KEY`
     - `OPENAI_API_KEY`
     - `GEMINI_API_KEY`
     - `COHERE_API_KEY`

5. **Push to Hugging Face**
```bash
git add .
git commit -m "Deploy to Hugging Face Spaces"
git push
```

### Option 2: Custom Dockerfile for HF Spaces

Create a single `Dockerfile` that combines both services:

```dockerfile
FROM python:3.11-slim AS backend-builder
# ... backend build stage

FROM node:18-alpine AS frontend-builder
# ... frontend build stage

FROM ubuntu:22.04
# Install supervisord to run both services
RUN apt-get update && apt-get install -y supervisord
# Copy both built apps
# Configure to run on port 7860 (HF default)
```

### Option 3: Streamlit/Gradio Wrapper (Recommended for HF)

Create a `app.py` using Streamlit or Gradio:

```python
import streamlit as st
import requests
from datetime import datetime

st.set_page_config(
    page_title="Physical AI Humanoid Book",
    page_icon="🤖",
    layout="wide"
)

st.title("🤖 Physical AI Humanoid Book")

# Initialize session state
if "backend_url" not in st.session_state:
    st.session_state.backend_url = "http://localhost:8000"

# Sidebar configuration
with st.sidebar:
    st.header("⚙️ Configuration")
    backend_url = st.text_input(
        "Backend URL",
        value=st.session_state.backend_url
    )
    
    if st.button("Test Connection"):
        try:
            response = requests.get(f"{backend_url}/health", timeout=5)
            if response.status_code == 200:
                st.success("✅ Backend connected!")
            else:
                st.error(f"❌ Backend error: {response.status_code}")
        except Exception as e:
            st.error(f"❌ Connection failed: {str(e)}")

# Main content
tabs = st.tabs(["Chat", "Documentation", "Diagnostics"])

with tabs[0]:
    st.header("Chat with RAG Chatbot")
    query = st.text_area("Enter your question:")
    if st.button("Send"):
        if query:
            try:
                response = requests.post(
                    f"{backend_url}/api/chat/query",
                    json={"query": query}
                )
                if response.status_code == 200:
                    result = response.json()
                    st.write(result.get("response"))
                else:
                    st.error(f"Error: {response.status_code}")
            except Exception as e:
                st.error(f"Failed to get response: {str(e)}")

with tabs[1]:
    st.header("Documentation")
    st.write("Documentation content here...")

with tabs[2]:
    st.header("System Diagnostics")
    if st.button("Refresh Diagnostics"):
        try:
            response = requests.get(f"{backend_url}/diagnostics")
            if response.status_code == 200:
                diag = response.json()
                st.json(diag)
            else:
                st.error("Failed to fetch diagnostics")
        except Exception as e:
            st.error(f"Error: {str(e)}")
```

Then create `requirements.txt` for the app:
```
streamlit==1.28.1
requests==2.31.0
```

---

## Docker Commands Reference

### Build Commands
```bash
# Build all services
docker-compose build

# Build specific service
docker-compose build backend

# Build without cache
docker-compose build --no-cache
```

### Container Management
```bash
# Start services in background
docker-compose up -d

# Start with logs output
docker-compose up

# Stop services
docker-compose stop

# Stop and remove containers
docker-compose down

# Remove volumes (database data)
docker-compose down -v
```

### Monitoring
```bash
# View container status
docker-compose ps

# View logs
docker-compose logs

# Follow backend logs
docker-compose logs -f backend

# View logs for specific time period
docker-compose logs --since 10m backend

# View container resource usage
docker stats
```

### Database Management
```bash
# Access PostgreSQL
docker-compose exec postgres psql -U postgres -d edgekit_db

# Create backup
docker-compose exec postgres pg_dump -U postgres edgekit_db > backup.sql

# Restore backup
docker-compose exec -T postgres psql -U postgres edgekit_db < backup.sql

# Access Qdrant database
docker-compose exec qdrant sh
```

### Image Management
```bash
# List images
docker images

# Remove image
docker rmi image-name

# Tag image
docker tag source-image:tag target-registry/image:tag

# Push to registry
docker push registry/image:tag

# Pull from registry
docker pull registry/image:tag
```

---

## Troubleshooting

### Common Issues

#### 1. **Port Already in Use**
```bash
# Find process using port
lsof -i :8000

# Kill process
kill -9 <PID>

# Or use different port in docker-compose
# Edit service ports: "9000:8000"
```

#### 2. **Database Connection Failed**
```bash
# Check database logs
docker-compose logs postgres

# Verify database is running
docker-compose ps

# Check database connectivity
docker-compose exec backend python -c "from sqlalchemy import create_engine; \
  engine = create_engine('postgresql://postgres:postgres@postgres:5432/edgekit_db'); \
  conn = engine.connect(); print('Connected!')"
```

#### 3. **psycopg2 Module Not Found**
- Already fixed in updated Dockerfile
- Ensure `libpq-dev` is installed during build
- Rebuild images: `docker-compose build --no-cache`

#### 4. **OpenAI Client Initialization Errors**
- Fixed in updated `openai_client.py`
- Ensure API key is set in `.env`
- Check OpenAI API key validity

#### 5. **Qdrant Connection Issues**
```bash
# Check Qdrant health
curl http://localhost:6333/readyz

# Check collection exists
docker-compose exec backend python -c "from src.clients.qdrant_client import qdrant_client; \
  print(qdrant_client.get_collections())"
```

#### 6. **Frontend Not Connecting to Backend**
```bash
# Check backend is accessible from frontend
docker-compose exec frontend curl http://backend:8000/health

# Check Nginx configuration
docker-compose exec frontend cat /etc/nginx/conf.d/default.conf

# Check CORS headers
curl -i -X OPTIONS http://localhost:8000/api/
```

#### 7. **Out of Memory**
```bash
# Check Docker memory usage
docker system df

# Increase Docker memory limit (Docker Desktop)
# Settings > Resources > Memory: increase to 8GB+

# Clean up unused resources
docker system prune -a
```

### Health Checks

```bash
# Backend health
curl http://localhost:8000/health

# Backend diagnostics
curl http://localhost:8000/diagnostics

# Database health
docker-compose exec postgres pg_isready

# Qdrant health
curl http://localhost:6333/readyz

# Frontend health
curl http://localhost/health
```

### Logs Analysis

```bash
# View all logs
docker-compose logs --all

# Search logs for errors
docker-compose logs | grep -i error

# Get logs since specific time
docker-compose logs --since 2024-01-01T12:00:00

# Get logs until specific time
docker-compose logs --until 2024-01-01T13:00:00

# Follow logs in real-time
docker-compose logs -f

# Show last 100 lines
docker-compose logs --tail 100
```

### Performance Optimization

```bash
# Profile backend with docker stats
docker stats physical-ai-backend

# Analyze image size
docker image ls --format "table {{.Repository}}\t{{.Size}}"

# Build with BuildKit for faster builds
DOCKER_BUILDKIT=1 docker build .

# Use .dockerignore to reduce context
# Create .dockerignore in root and backend/
```

---

## Next Steps

1. **Set up CI/CD Pipeline**
   - GitHub Actions for automated testing and building
   - Automatic deployment on push to main

2. **Monitor Production**
   - Set up logging (ELK Stack, CloudWatch)
   - Monitor resource usage
   - Set up alerts

3. **Scale for Production**
   - Use Kubernetes (K8s) instead of Docker Compose
   - Set up load balancing
   - Use managed database services

4. **Security Hardening**
   - Use secrets management (HashiCorp Vault)
   - Enable HTTPS/TLS
   - Implement rate limiting
   - Use network policies

---

## Support

For issues or questions:
- Check logs: `docker-compose logs -f`
- Run diagnostics: `curl http://localhost:8000/diagnostics`
- Check troubleshooting section above
- Open an issue on GitHub
