# Hugging Face Spaces Deployment Guide

This document explains how to deploy the Physical AI Humanoid Book to Hugging Face Spaces.

## Prerequisites

1. **Hugging Face Account** - [Sign up here](https://huggingface.co)
2. **HF Token** - Get from [HF Settings](https://huggingface.co/settings/tokens)
3. **Git configured** with SSH or HTTPS

## Quick Deploy

### Option 1: Using Deploy Script

```bash
./deploy_hf.sh your_hf_token_here
```

### Option 2: Manual Deployment

1. **Create Hugging Face Space**
   ```bash
   huggingface-cli repo create \
       --repo_id="Awais68/physical-AI-Humanoid-Book" \
       --type="space" \
       --space_sdk=docker
   ```

2. **Add HF Remote**
   ```bash
   git remote add hf https://huggingface.co/spaces/Awais68/physical-AI-Humanoid-Book
   ```

3. **Push Code**
   ```bash
   git push hf main
   ```

## Configuration

### Required Environment Variables (set in HF Space Secrets)

```env
OPENAI_API_KEY=your_openai_key
GEMINI_API_KEY=your_gemini_key
COHERE_API_KEY=your_cohere_key
QDRANT_URL=your_qdrant_url
```

### Space Settings

- **SDK:** Docker
- **Resources:** CPU / GPU (recommended for better performance)
- **Private:** False (for public access)

## Architecture

```
Streamlit Interface (Port 8501)
    ↓
Backend API (Port 8000)
    ├── FastAPI
    ├── PostgreSQL
    └── Qdrant Vector DB
```

## Access

After deployment:
- **Frontend:** https://huggingface.co/spaces/Awais68/physical-AI-Humanoid-Book
- **API:** Backend runs on port 8000 (internal)
- **Streamlit:** Port 8501

## Troubleshooting

### Build fails
- Check Docker configuration
- Verify all dependencies in `requirements.txt`
- Check HF Spaces logs

### API not responding
- Verify environment variables are set
- Check backend logs in HF Spaces
- Ensure database connections work

### Memory issues
- Reduce model size or use CPU-only
- Increase resources in HF Space settings

## Updating

To push updates:
```bash
git push hf main
```

HF Spaces will automatically rebuild and redeploy.

## Monitoring

1. Go to HF Space settings
2. Check "App logs" for Streamlit output
3. Check "Build logs" for Docker build status

## Production Best Practices

1. ✅ Use environment variables for secrets
2. ✅ Set up health checks
3. ✅ Configure auto-restart
4. ✅ Monitor API responses
5. ✅ Set rate limiting
6. ✅ Use caching for frequent queries

## Support

- 📖 [HF Spaces Docs](https://huggingface.co/docs/hub/spaces)
- 🐛 [Report Issues](https://github.com/Awais68/physical-AI-Homanoid-Book/issues)
- 💬 [Discussions](https://huggingface.co/spaces/Awais68/physical-AI-Humanoid-Book/discussions)
