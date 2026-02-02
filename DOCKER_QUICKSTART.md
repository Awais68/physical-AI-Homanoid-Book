# Quick Start Guide - Docker Deployment

## ⚡ 5-Minute Setup

### 1. Clone and Configure
```bash
git clone https://github.com/yourusername/physical-AI-Homanoid-Book-main.git
cd physical-AI-Homanoid-Book-main
cp .env.example .env
```

### 2. Add Your API Keys to `.env`
```bash
OPENAI_API_KEY=sk-your-key-here
GEMINI_API_KEY=your-gemini-key-here
COHERE_API_KEY=your-cohere-key-here
QDRANT_API_KEY=your-qdrant-key-here
```

### 3. Start Development Environment
```bash
docker-compose -f docker-compose.dev.yml up --build
```

### 4. Access Services
- 🌐 **Frontend**: http://localhost:3000
- 🔌 **Backend API**: http://localhost:8000
- 📖 **API Docs**: http://localhost:8000/docs
- 🗄️ **Qdrant**: http://localhost:6333/dashboard
- 🐘 **PostgreSQL**: localhost:5432

---

## 🚀 Production Deployment

### Local Production Build
```bash
docker-compose -f docker-compose.prod.yml up --build
```

### Deploy to Hugging Face Spaces

#### Option A: Using Streamlit (Easiest)
```bash
# 1. Create a Hugging Face Space
# Go to: https://huggingface.co/spaces/new
# Select "Docker" runtime

# 2. Clone your space
git clone https://huggingface.co/spaces/YOUR_USERNAME/YOUR_SPACE_NAME
cd YOUR_SPACE_NAME

# 3. Copy deployment files
python scripts/deploy_hf.py
# (Requires HF_TOKEN and HF_SPACE_ID environment variables)

# 4. View your deployment
open https://huggingface.co/spaces/YOUR_USERNAME/YOUR_SPACE_NAME
```

#### Option B: Using Full Docker Compose
```bash
# 1. Set secrets in HF Space settings:
#    - QDRANT_API_KEY
#    - OPENAI_API_KEY
#    - GEMINI_API_KEY
#    - COHERE_API_KEY

# 2. Copy docker-compose.prod.yml to your space as docker-compose.yml

# 3. Git push to trigger deployment
git add .
git commit -m "Deploy docker setup"
git push
```

---

## 📊 Verify Deployment

### Health Checks
```bash
# Backend health
curl http://localhost:8000/health

# Full diagnostics
curl http://localhost:8000/diagnostics

# Database status
docker-compose ps postgres

# All services
docker-compose ps
```

### View Logs
```bash
# All logs
docker-compose logs

# Backend only
docker-compose logs -f backend

# Frontend only
docker-compose logs -f frontend

# Database errors
docker-compose logs postgres | grep -i error
```

---

## 🔧 Common Commands

```bash
# Start services
docker-compose up -d

# Stop services
docker-compose stop

# View running containers
docker-compose ps

# Restart a service
docker-compose restart backend

# Rebuild images
docker-compose build --no-cache

# Clean up everything
docker-compose down -v

# Access database shell
docker-compose exec postgres psql -U postgres

# Access backend shell
docker-compose exec backend bash

# View service resource usage
docker stats
```

---

## 🐛 Troubleshooting

### Backend failing to start?
```bash
# Check logs
docker-compose logs backend

# Common issues:
# 1. Missing psycopg2 → Rebuild: docker-compose build --no-cache
# 2. Port in use → Change port in docker-compose.yml
# 3. API keys invalid → Check .env file
```

### Frontend not connecting to backend?
```bash
# Check Nginx config
docker-compose exec frontend cat /etc/nginx/conf.d/default.conf

# Test connectivity
docker-compose exec frontend curl http://backend:8000/health

# Check logs
docker-compose logs frontend
```

### Database connection failed?
```bash
# Check PostgreSQL
docker-compose logs postgres

# Verify credentials
docker-compose exec postgres psql -U postgres -d edgekit_db -c "SELECT 1"

# Reset database
docker-compose down -v
docker-compose up postgres
```

---

## 📈 Scale for Production

### Using Kubernetes
```bash
# Install kubectl and Helm
brew install kubectl helm

# Deploy using Helm
helm install physical-ai ./k8s --values values.prod.yml
```

### Using Cloud Platforms

**Google Cloud Run:**
```bash
gcloud run deploy physical-ai-backend \
  --image gcr.io/your-project/physical-ai-backend \
  --platform managed \
  --region us-central1
```

**AWS ECS:**
```bash
aws ecs create-service \
  --cluster physical-ai \
  --service-name backend \
  --task-definition physical-ai-backend:1
```

**Azure Container Instances:**
```bash
az container create \
  --resource-group physical-ai \
  --name backend \
  --image acr.azurecr.io/physical-ai-backend:latest
```

---

## 📚 Documentation

- [Full Docker Guide](./DOCKER_DEPLOYMENT_GUIDE.md)
- [API Documentation](http://localhost:8000/docs)
- [GitHub Actions CI/CD](./.github/workflows/)
- [Environment Variables](./.env.example)

---

## 🆘 Need Help?

1. **Check logs**: `docker-compose logs -f`
2. **Run diagnostics**: `curl http://localhost:8000/diagnostics`
3. **Test connection**: `curl http://localhost:8000/health`
4. **GitHub Issues**: [Open an issue](https://github.com/yourusername/physical-AI-Homanoid-Book-main/issues)
5. **Discord Community**: [Join our server](https://discord.gg/yourlink)

---

## Next Steps

✅ Containers running locally  
➜ Deploy to Hugging Face Spaces  
➜ Set up CI/CD pipeline  
➜ Configure monitoring & logging  
➜ Scale to production  

**Happy deploying! 🚀**
