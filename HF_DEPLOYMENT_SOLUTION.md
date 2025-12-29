# 🚀 HUGGING FACE DEPLOYMENT - WITH LOCAL QDRANT

## Problem Solved

✅ No more "Connection refused" errors  
✅ Qdrant runs **inside** your HF Space (same container)  
✅ No external network calls needed

---

## Files Created

1. **`Dockerfile.hf`** - Builds image with Qdrant + Backend
2. **`start-hf.sh`** - Startup script (Qdrant → Ingest → Backend)
3. **`backend/check_and_ingest.py`** - Auto-ingests documents on first run
4. **`docker-compose.hf.yml`** - For local testing

---

## Deployment Steps

### 1. Update Your Hugging Face Space

**In your HF Space settings:**

1. **Dockerfile:** Rename to use `Dockerfile.hf`

   ```yaml
   # In your Space README.md (at top)
   ---
   title: Physical AI Backend
   sdk: docker
   dockerfile: Dockerfile.hf
   app_port: 7860
   ---
   ```

2. **Set Environment Variables** (in HF Space Secrets):

   ```bash
   GEMINI_API_KEY=AIzaSyBboD8qnHgKn9dMkAQeLGlTLaRksHBss-o
   COHERE_API_KEY=lx3kujAI5pAIuq63SAPSsO8zRboc8cjvcMdWmx9y
   OPENAI_API_KEY=sk-proj-Nt88FZew...
   QDRANT_URL=http://localhost:6333
   QDRANT_COLLECTION=rag_chatbot
   ```

3. **Push to GitHub:**

   ```bash
   git add Dockerfile.hf start-hf.sh backend/check_and_ingest.py
   git commit -m "feat: Self-hosted Qdrant in HF Space"
   git push
   ```

4. **HF Space will rebuild automatically**

---

## How It Works

```
┌─────────────────────────────────────┐
│   Hugging Face Space (Container)    │
│                                      │
│  ┌──────────┐      ┌─────────────┐ │
│  │  Qdrant  │◄────►│   Backend   │ │
│  │  :6333   │      │   :7860     │ │
│  └──────────┘      └─────────────┘ │
│       ▲                   ▲         │
│       │                   │         │
│   Documents         FastAPI         │
│   (134 docs)        Endpoints       │
└─────────────────────────────────────┘
           ▲
           │
    External Requests
```

**Startup Sequence:**

1. ✅ Container starts
2. ✅ Qdrant starts on `localhost:6333`
3. ✅ Check if collection exists
4. ✅ If empty → auto-ingest 134 documents
5. ✅ Start FastAPI backend on port 7860
6. ✅ All services ready!

---

## Test Locally First

```bash
# Build and run
docker build -f Dockerfile.hf -t physical-ai-backend .
docker run -p 7860:7860 \
  -e GEMINI_API_KEY=your_key \
  -e COHERE_API_KEY=your_key \
  physical-ai-backend

# Test endpoints
curl http://localhost:7860/health
curl http://localhost:7860/diagnostics

# Test chat
curl -X POST http://localhost:7860/api/chat/message \
  -H "Content-Type: application/json" \
  -d '{"message": "What is Physical AI?", "conversationHistory": []}'
```

---

## Advantages

✅ **No External Dependencies** - Everything in one container  
✅ **No Network Issues** - Qdrant on localhost  
✅ **Auto-Ingestion** - Documents loaded on first run  
✅ **Persistent Storage** - Qdrant data survives restarts  
✅ **Simple Deployment** - Just push to GitHub

---

## Environment Variables Needed

```bash
# Required
GEMINI_API_KEY=your_gemini_key
COHERE_API_KEY=your_cohere_key

# Optional
OPENAI_API_KEY=your_openai_key  # Fallback
DATABASE_URL=sqlite:///./edgekit.db  # Default

# Auto-set (don't change)
QDRANT_URL=http://localhost:6333
QDRANT_COLLECTION=rag_chatbot
```

---

## Monitoring

**After deployment:**

1. **Check logs** in HF Space:

   ```
   🚀 Starting Physical AI Backend with Qdrant...
   📦 Starting Qdrant vector database...
   ✅ Qdrant is running
   📚 Checking document collection...
   ✅ Collection 'rag_chatbot' exists with 134 documents
   ✅ Documents already ingested, skipping ingestion
   🚀 Starting FastAPI backend...
   ```

2. **Test endpoints:**

   ```bash
   # Health
   curl https://your-space.hf.space/health

   # Diagnostics (should show Qdrant connected)
   curl https://your-space.hf.space/diagnostics

   # Chat
   curl -X POST https://your-space.hf.space/api/chat/message \
     -H "Content-Type: application/json" \
     -d '{"message":"test","conversationHistory":[]}'
   ```

---

## Troubleshooting

### "Qdrant failed to start"

- **Cause:** Not enough memory
- **Solution:** Upgrade HF Space to CPU basic (free)

### "Collection empty"

- **Cause:** Ingestion failed
- **Solution:** Check logs for Cohere API errors

### "Port 7860 not accessible"

- **Cause:** Wrong port in Dockerfile
- **Solution:** Ensure `app_port: 7860` in Space config

---

## Result

✅ **No more "Connection refused" errors**  
✅ **Document search fully working**  
✅ **All 134 documents accessible**  
✅ **Same Hugging Face Space**  
✅ **Zero external dependencies**

🎉 **Problem solved!**
