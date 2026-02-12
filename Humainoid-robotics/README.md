---
title: Humainoid Robotics
emoji: 🤖
colorFrom: blue
colorTo: pink
sdk: docker
pinned: false
license: mit
short_description: Physical AI & Humanoid Robotics RAG API
---

# 🤖 Humainoid Robotics — RAG API

An intelligent **RAG-powered API** for **Physical AI & Humanoid Robotics** education, grounded in 134+ documents.

## Features

- 💬 **RAG-Powered Q&A**: Answers grounded in 134+ documents from the Physical AI & Humanoid Robotics book
- 🧠 **AI Backends**: Gemini 2.5 Flash (primary) + OpenAI GPT-4o-mini (fallback)
- 🔍 **Semantic Search**: Qdrant vector database with Gemini text-embedding-004
- 📚 **Source Citations**: Every answer includes relevant source documents
- 🎓 **Educational Focus**: Specialized for K-12 and higher education contexts
- 🚀 **FastAPI**: High-performance async API with automatic documentation

## Architecture

```
┌──────────────────────────────────────────────┐
│              HF Space (Docker)               │
│                                              │
│          ┌──────────────────┐                │
│          │  FastAPI App     │                │
│          │  (port 7860)     │                │
│          └──────────────────┘                │
│                  │                           │
│                  ▼                           │
│         ┌────────────────┐                   │
│         │   RAG Engine   │                   │
│         └────────────────┘                   │
│            │    │    │                       │
│            ▼    ▼    ▼                       │
│        Qdrant Gemini OpenAI                  │
│        (Cloud) (API)  (API)                  │
└──────────────────────────────────────────────┘
```

## API Endpoints

The FastAPI backend provides:

- `POST /api/chat/message` — Send a chat message (RAG-powered)
- `POST /api/chat/selected-text` — Query about selected text
- `POST /api/chat/index` — Index a new document
- `GET /api/chat/health` — RAG chatbot health check
- `GET /health` — Backend health check

## Environment Variables

Set these as HF Space secrets:

| Variable | Description |
|----------|-------------|
| `GEMINI_API_KEY` | Google Gemini API key |
| `OPENAI_API_KEY` | OpenAI API key (fallback) |
| `COHERE_API_KEY` | Cohere API key (embeddings) |
| `QDRANT_URL` | Qdrant Cloud URL |
| `QDRANT_API_KEY` | Qdrant API key |
| `QDRANT_COLLECTION` | Qdrant collection name |

## Links

- [GitHub Repository](https://github.com/Awais68/physical-AI-Homanoid-Book)
- [Documentation Site](https://awais68.github.io/physical-AI-Homanoid-Book/)
- [HF Space](https://huggingface.co/spaces/Awais68/Humainoid-robotics)
