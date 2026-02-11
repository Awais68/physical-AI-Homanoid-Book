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

This is a FastAPI backend with built-in Qdrant vector database for Physical AI educational content. It provides:

- 📚 **RAG (Retrieval-Augmented Generation)** with 134+ documents
- 🤖 **AI-powered chat** using Gemini 2.5 Flash
- 🔍 **Semantic search** with Cohere embeddings
- 🗄️ **Self-hosted Qdrant** (no external dependencies)

## API Endpoints

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

```
┌─────────────────────────────────────┐
│     Docker Container (HF Space)     │
│                                      │
│  ┌──────────┐      ┌─────────────┐ │
│  │  Qdrant  │◄────►│   Backend   │ │
│  │  :6333   │      │   :7860     │ │
│  └──────────┘      └─────────────┘ │
│       ▲                   ▲         │
│   134 Docs          FastAPI         │
└─────────────────────────────────────┘
```

## Features

- ✅ Self-contained (Qdrant runs inside container)
- ✅ Auto-ingestion on first startup
- ✅ Graceful fallback if AI APIs unavailable
- ✅ Multi-model support (Gemini, OpenAI)
- ✅ Comprehensive error handling

## Environment Variables

Set these in your Space settings:

```bash
GEMINI_API_KEY=your_key_here
COHERE_API_KEY=your_key_here
OPENAI_API_KEY=your_key_here  # Optional
```

Internal variables (auto-configured):

```bash
QDRANT_URL=http://localhost:6333
QDRANT_COLLECTION=rag_chatbot
DATABASE_URL=sqlite:///./edgekit.db
```

## Usage Example

```bash
# Test the chat endpoint
curl -X POST https://your-space.hf.space/api/chat/message \
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

## Repository

[GitHub: Awais68/physical-AI-Homanoid-Book](https://github.com/Awais68/physical-AI-Homanoid-Book)

## License

MIT
