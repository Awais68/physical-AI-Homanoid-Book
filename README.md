---
title: Physical AI Humanoid Book - RAG Backend
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
dockerfile: Dockerfile.hf
app_port: 7860
pinned: false
license: mit
---

# Physical AI Humanoid Book - RAG Backend

## Overview

This is a comprehensive Physical AI platform with separated frontend and backend services. It provides:

- 📚 **RAG (Retrieval-Augmented Generation)** with 134+ documents
- 🤖 **AI-powered chat** using Gemini 2.5 Flash
- 🔍 **Semantic search** with Cohere embeddings
- 🌐 **Separate frontend and backend** services for scalability
- 🗄️ **Qdrant vector database** and PostgreSQL for data management
- 🎯 **Educational robotics platform** with safety controls

## Docker Deployment (Recommended)

### Quick Start

The easiest way to deploy the Physical AI Edge Kit is using Docker with separated frontend and backend services.

1. Make sure you have Docker and Docker Compose installed

2. Configure your environment variables:
   ```bash
   cp .env.example .env
   # Edit .env and add your API keys
   ```

3. Start the services:
   ```bash
   ./deploy.sh up
   ```

4. Access the services:
   - Frontend: http://localhost:3000
   - Backend API: http://localhost:8000
   - Qdrant UI: http://localhost:6333
   - API Documentation: http://localhost:8000/docs

### Development Mode

For development with hot reloading:
```bash
./deploy.sh dev
```

### Available Docker Commands

```bash
./deploy.sh [command]

# Commands:
#   up          - Start production services
#   dev         - Start development services
#   down        - Stop all services
#   logs        - View service logs
#   build       - Build services
#   rebuild     - Rebuild services from scratch
#   status      - Show service status
```

## Manual Installation

If you prefer to run the services manually:

### Prerequisites
- Python 3.11+
- Node.js 18+ (for frontend)
- Docker (for Qdrant and PostgreSQL)

### Backend Installation

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Install Node.js dependencies for frontend:
   ```bash
   cd ../frontend
   npm install
   ```

### Configuration

Create a `.env` file in the project root with your API keys:

```env
GEMINI_API_KEY=your_gemini_api_key
OPENAI_API_KEY=your_openai_api_key  # Optional
COHERE_API_KEY=your_cohere_api_key  # Optional
QDRANT_API_KEY=your_qdrant_api_key  # For cloud deployments
QDRANT_URL=http://localhost:6333    # Default local Qdrant
POSTGRES_USER=postgres
POSTGRES_PASSWORD=postgres
POSTGRES_DB=edgekit_db
DATABASE_URL=postgresql://postgres:postgres@localhost:5432/edgekit_db
```

## API Endpoints

The backend provides the following main API endpoints:

### Health Check

```bash
GET /health
```

### Diagnostics

```bash
GET /diagnostics
# Shows connection status of all services
```

### Chat

```bash
POST /api/chat/message
Content-Type: application/json

{
  "message": "What is Physical AI?",
  "conversationHistory": []
}
```

### API Documentation

```bash
GET /docs
# Interactive Swagger UI
```

## Architecture

With Docker deployment:

```
┌─────────────────────────────────────────────────────────┐
│                     Docker Network                      │
│                                                         │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐ │
│  │   Frontend  │◄──►│   Backend   │◄──►│   Qdrant    │ │
│  │   :3000     │    │   :8000     │    │ :6333/:6334 │ │
│  └─────────────┘    └─────────────┘    └─────────────┘ │
│                           ▲                    ▲       │
│                    FastAPI API            Vector DB    │
│                           ▼                    │       │
│                    ┌─────────────┐           │       │
│                    │ PostgreSQL  │◄──────────┘       │
│                    │   :5432     │                   │
│                    └─────────────┘                   │
└─────────────────────────────────────────────────────────┘
```

## Features

- ✅ **Separated frontend and backend** services for scalability
- ✅ **Auto-ingestion** of educational content on first startup
- ✅ **Graceful fallback** if AI APIs unavailable
- ✅ **Multi-model support** (Gemini, OpenAI, Cohere)
- ✅ **Comprehensive error handling**
- ✅ **Production-ready** Docker deployment
- ✅ **Development-friendly** with hot reloading
- ✅ **Security-focused** with non-root containers
- ✅ **Health checks** for all services

## Environment Variables

### Required for Production:

```bash
GEMINI_API_KEY=your_key_here
COHERE_API_KEY=your_key_here
OPENAI_API_KEY=your_key_here  # Optional
```

### Optional Configuration:

```bash
POSTGRES_USER=postgres
POSTGRES_PASSWORD=postgres
POSTGRES_DB=edgekit_db
QDRANT_URL=http://qdrant:6333  # Internal Docker network
BACKEND_API_URL=http://localhost:8000
BASE_URL=http://localhost:3000
```

## Usage Example

```bash
# Test the chat endpoint
curl -X POST http://localhost:8000/api/chat/message \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Explain Physical AI robotics",
    "conversationHistory": []
  }'
```

Response:

```json
{
  "answer": "Physical AI refers to...",
  "sources": [...],
  "source_count": 5,
  "confidence": 0.82
}
```

## Documentation

- [Docker Deployment Guide](DOCKER_DEPLOYMENT.md)
- [Architecture Documentation](docs/architecture.md)
- [Safety Framework](docs/safety.md)
- [Device Management](docs/devices.md)
- [Multilingual Support](docs/multilingual.md)

## Repository

[GitHub: Awais68/physical-AI-Homanoid-Book](https://github.com/Awais68/physical-AI-Homanoid-Book)

## License

MIT
