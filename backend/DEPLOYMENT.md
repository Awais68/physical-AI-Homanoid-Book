# Deployment Guide - Hugging Face Spaces

## Issues Fixed

### 1. ✅ Qdrant Connection Error

**Error:** `Search error: [Errno 111] Connection refused`

**Solution:**

- Added proper error handling in `qdrant_client.py`
- System now runs without Qdrant if connection fails
- Graceful degradation with clear logging

### 2. ✅ Database Error

**Error:** `No module named 'psycopg2'`

**Solution:**

- Added `psycopg2-binary==2.9.9` to `requirements.txt`
- Database connection now optional with fallback

### 3. ✅ OpenAI Client Error

**Error:** `Client.__init__() got an unexpected keyword argument 'proxies'`

**Solution:**

- Removed `proxies` parameter from OpenAI client init
- Added proper timeout configuration
- Better error handling

### 4. ✅ Gemini API Quota

**Error:** "AI-powered responses temporarily unavailable"

**Solution:**

- Enhanced fallback responses
- System returns documentation context even without AI
- Improved user messaging

## Hugging Face Environment Variables

Set these in your Space settings:

```bash
# Required - Gemini API
GEMINI_API_KEY=your_gemini_key_here
GEMINI_CHAT_MODEL=gemini-2.5-flash

# Required - Qdrant Cloud
QDRANT_URL=https://your-cluster.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_key_here
QDRANT_COLLECTION=rag_chatbot

# Required - Cohere for Embeddings
COHERE_API_KEY=your_cohere_key_here
COHERE_EMBEDDING_MODEL=embed-english-v3.0

# Optional - Database (SQLite used if not provided)
DATABASE_URL=postgresql://user:pass@host/db

# Optional - OpenAI (if using instead of Gemini)
OPENAI_API_KEY=your_openai_key_here
OPENAI_CHAT_MODEL=gpt-4o-mini
```

## Deployment Steps

### 1. Push to GitHub

```bash
git add .
git commit -m "Fix: Deployment issues for Hugging Face"
git push origin main
```

### 2. Create Hugging Face Space

1. Go to https://huggingface.co/spaces
2. Click "Create new Space"
3. Select "Gradio" SDK
4. Connect your GitHub repository

### 3. Configure Space

1. Go to Space Settings → Environment Variables
2. Add all required environment variables (see above)
3. Set Python version: 3.11+

### 4. Create `app.py` (if not exists)

```python
import gradio as gr
import requests

API_URL = "http://localhost:8000"

def chat(message, history):
    response = requests.post(
        f"{API_URL}/api/chat/message",
        json={"message": message, "conversationHistory": []}
    )
    if response.status_code == 200:
        data = response.json()
        return data.get("answer", "No response")
    return "Error connecting to backend"

iface = gr.ChatInterface(
    fn=chat,
    title="Physical AI & Humanoid Robotics Assistant",
    description="Ask questions about Physical AI and educational robotics"
)

if __name__ == "__main__":
    # Start backend in background
    import subprocess
    import time
    subprocess.Popen(["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"])
    time.sleep(5)  # Wait for backend to start
    iface.launch(server_name="0.0.0.0", server_port=7860)
```

## Testing Locally

```bash
# Install dependencies
pip install -r requirements.txt

# Set environment variables
export GEMINI_API_KEY=your_key
export QDRANT_URL=your_url
export QDRANT_API_KEY=your_key
export COHERE_API_KEY=your_key

# Run backend
uvicorn src.main:app --host 0.0.0.0 --port 8000

# Test
curl -X POST http://localhost:8000/api/chat/message \
  -H "Content-Type: application/json" \
  -d '{"message": "What is Physical AI?", "conversationHistory": []}'
```

## Monitoring

Check these endpoints:

- Health: `https://your-space.hf.space/health`
- **Diagnostics: `https://your-space.hf.space/diagnostics`** (shows all service connections)
- API Docs: `https://your-space.hf.space/docs`
- Chat: `POST https://your-space.hf.space/api/chat/message`

## Troubleshooting

### "Search error: [Errno 111] Connection refused" (repeated)

**Symptoms:** Logs show repeated connection errors but API returns 200 OK

**Root Cause:** Qdrant Cloud is unreachable from Hugging Face servers. Possible reasons:

1. Qdrant Cloud firewall blocking Hugging Face IPs
2. Network timeout between HF → Qdrant Cloud
3. Invalid Qdrant URL or API key
4. Qdrant Cloud service temporarily down

**Solutions:**

1. **Check Qdrant Cloud Dashboard:**

   - Login to https://cloud.qdrant.io
   - Verify cluster is "Running" (green status)
   - Check if IP whitelist is enabled (disable for public access)
   - Verify API key is active

2. **Test Connection from Terminal:**

   ```bash
   curl -H "api-key: YOUR_KEY" \
        https://YOUR_CLUSTER.cloud.qdrant.io/collections
   ```

3. **Enable Public Access in Qdrant:**

   - Go to Cluster Settings → Network
   - Ensure "Public Access" is enabled
   - Remove IP restrictions if set

4. **Verify Environment Variables:**

   - Check `/diagnostics` endpoint output
   - Ensure QDRANT_URL uses `https://` not `http://`
   - Confirm QDRANT_API_KEY is set correctly

5. **Use Alternative Deployment:**
   If Hugging Face → Qdrant connection is blocked:
   - Deploy backend to Railway/Render/Fly.io (better network access)
   - Use Hugging Face only for frontend/Gradio UI
   - Point Gradio to external backend API

**Temporary Workaround:** System works without Qdrant (returns generic responses). Check `/diagnostics` to confirm other services are connected.

### No document results (sources: 0)

- **Cause:** Qdrant connection failed or collection empty
- **Solution:** Check QDRANT_URL and QDRANT_API_KEY, ensure collection has documents
- **Verify:** Visit `/diagnostics` to check `qdrant.connected` status

### Generic responses only

- **Cause:** Gemini API quota exceeded
- **Solution:** Wait for quota reset or upgrade to paid tier
- **Check:** `/diagnostics` shows `gemini.connected` status

### Database errors

- **Cause:** PostgreSQL connection issues
- **Solution:** System runs with SQLite by default, no action needed

## Performance Tips

1. **Use Qdrant Cloud** instead of local for better reliability
2. **Upgrade Cohere** to production key (1000+ calls/month)
3. **Monitor Gemini quota** - free tier: 20 requests/day
4. **Cache responses** to reduce API calls
5. **Use CDN** for frontend assets

## Support

- Logs: Check Hugging Face Space logs tab
- Issues: GitHub repository issues
- Docs: `/docs` endpoint for API documentation
