# 🚀 Hugging Face Spaces Deployment Guide

## Quick Start (Browser Method)

### Step 1: Create Space on Hugging Face

1. Go to https://huggingface.co/new-space
2. Fill in details:
   - **Space name:** `physical-AI-Humanoid-Book`
   - **License:** OpenRAIL
   - **Space SDK:** Docker
   - **Space hardware:** CPU (free tier)
3. Click "Create space"

### Step 2: Clone HF Space Repository

```bash
git clone https://huggingface.co/spaces/Awais68/physical-AI-Humanoid-Book
cd physical-AI-Humanoid-Book
```

### Step 3: Copy Project Files

```bash
# Copy all project files to the space directory
cp -r ~/physical-ai/* .
rm -rf .git  # Remove old git history

# Initialize new git repo for HF
git init
git add .
git commit -m "🚀 Initial deployment to HF Spaces"
```

### Step 4: Push to HF Spaces

```bash
# Add HF remote
git remote add origin https://huggingface.co/spaces/Awais68/physical-AI-Humanoid-Book

# Push to HF
git push -u origin main
```

### Step 5: Configure Environment Variables

1. Go to your Space: https://huggingface.co/spaces/Awais68/physical-AI-Humanoid-Book
2. Click **Settings** (gear icon)
3. Go to **Repository secrets**
4. Add these secrets:

```
OPENAI_API_KEY=sk-...
GEMINI_API_KEY=...
COHERE_API_KEY=...
QDRANT_URL=https://...
DATABASE_URL=postgresql://...
```

### Step 6: Monitor Deployment

- Check **Logs** tab to see build progress
- Space will automatically build and deploy
- Initial build: 5-15 minutes

---

## Alternative: Git-based Deployment

If you have HF token already set up:

```bash
cd ~/physical-ai

# Configure git
export HF_TOKEN="YOUR_HF_TOKEN_HERE"  # Get from https://huggingface.co/settings/tokens
git config --global user.name "Awais68"
git config --global user.email "awais@example.com"
git config --global credential.helper store
echo "https://Awais68:$HF_TOKEN@huggingface.co" > ~/.git-credentials

# Add HF remote
git remote add hf https://huggingface.co/spaces/Awais68/physical-AI-Humanoid-Book.git

# Push
git push -u hf main
```

---

## Docker Configuration

The Space uses `docker-compose.hf.yml` which:

- ✅ Uses Streamlit as frontend (lighter than Docusaurus)
- ✅ Runs backend FastAPI on port 7860 (HF default)
- ✅ Includes PostgreSQL and Qdrant
- ✅ Optimized for cloud environment

### Dockerfile

```dockerfile
FROM python:3.11-slim

WORKDIR /app

# Install dependencies
COPY backend/requirements.txt .
RUN pip install -r requirements.txt

# Copy code
COPY backend/ .

# Run Streamlit app
CMD ["streamlit", "run", "streamlit_app.py", "--server.port=7860"]
```

---

## Environment Setup

### Required API Keys

```env
# LLM APIs (get from their dashboards)
OPENAI_API_KEY=sk-...
GEMINI_API_KEY=...
COHERE_API_KEY=...

# Database
DATABASE_URL=postgresql://user:pass@postgres:5432/db
QDRANT_URL=http://qdrant:6333
```

### Optional Configurations

```env
# Environment
ENVIRONMENT=production
DEBUG=false
LOG_LEVEL=info

# Services
PYTHONUNBUFFERED=1
PORT=7860
```

---

## Monitoring & Debugging

### View Logs

```bash
# Space will show logs in real-time
# Check "Logs" tab in Space Settings
```

### Common Issues

**Issue: Build fails with "module not found"**
- Solution: Check `backend/requirements.txt` has all dependencies

**Issue: Space timeout during build**
- Solution: Reduce image size or use faster base image

**Issue: App crashes after deployment**
- Solution: Check Logs tab for error messages

### Health Checks

Once deployed, visit:
```
https://huggingface.co/spaces/Awais68/physical-AI-Humanoid-Book

# Test endpoints:
GET /health
POST /chat/message
```

---

## CI/CD Integration

### Auto-updates from GitHub

1. Set up GitHub repository sync
2. Any push to `main` branch auto-deploys to HF
3. Useful for continuous deployment

```bash
# In HF Space settings, link to GitHub
# Repository URL: https://github.com/Awais68/physical-AI-Humanoid-Book
```

---

## Performance Optimization

### For HF Spaces (Free Tier)

- ✅ Use Streamlit (lightweight)
- ✅ Enable caching for API responses
- ✅ Reduce model size if possible
- ✅ Use vector DB for fast search
- ✅ Implement rate limiting

### Expected Performance

- **Load time:** 3-5 seconds (first load)
- **Chat response:** 2-5 seconds (with RAG)
- **Memory:** ~500MB RAM
- **Uptime:** 24/7

---

## Scaling Options

### If Free Tier Isn't Enough

1. **HF Spaces Upgrade:** Pay for better hardware
   - Cost: $7-50/month depending on tier
   - CPU/GPU options available

2. **Alternative Platforms:**
   - AWS EC2
   - Google Cloud Run
   - Azure Container Instances
   - Railway.app
   - Render.com

---

## Troubleshooting Checklist

- [ ] Space created on HF
- [ ] Code pushed to Space repo
- [ ] All secrets configured
- [ ] Build completed without errors
- [ ] App running on port 7860
- [ ] Health check responding
- [ ] API endpoints accessible
- [ ] Chat functionality working
- [ ] Database connected
- [ ] LLM APIs responding

---

## Support

- **HF Documentation:** https://huggingface.co/docs/hub
- **Space FAQ:** https://huggingface.co/docs/hub/spaces-overview
- **GitHub Issues:** https://github.com/Awais68/physical-AI-Homanoid-Book/issues

---

## Next Steps

1. ✅ Create Space on HF (https://huggingface.co/new-space)
2. ✅ Configure secrets in Space Settings
3. ✅ Push code to Space repo
4. ✅ Monitor build in Logs tab
5. ✅ Test deployed application
6. ✅ Share Space URL with others!

**Your HF Space URL:**
```
https://huggingface.co/spaces/Awais68/physical-AI-Humanoid-Book
```

---

**Status:** Ready for deployment  
**Date:** February 2, 2026  
**Maintainer:** Awais68
