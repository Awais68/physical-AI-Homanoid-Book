# Docker & Deployment Instructions

## 🎯 What's New?

Your project is now **fully containerized** and ready for production deployment on Hugging Face Spaces, cloud platforms, or local environments. All issues preventing deployment have been fixed.

### ✅ Fixed Issues
1. ✅ **psycopg2 module missing** - Added libpq-dev dependencies
2. ✅ **OpenAI client initialization error** - Removed invalid proxies parameter
3. ✅ **Database connection failures** - Proper health checks and dependencies
4. ✅ **Frontend-backend communication** - Nginx proxy configuration
5. ✅ **Missing environment configuration** - Comprehensive .env template

---

## 🚀 Quick Start (Choose One)

### Option 1: Development (with live reload)
```bash
cp .env.example .env
# Edit .env with your API keys

make dev-up
# or: docker-compose -f docker-compose.dev.yml up --build
```
- Frontend: http://localhost:3000
- Backend: http://localhost:8000
- Docs: http://localhost:8000/docs

### Option 2: Production (Local)
```bash
make prod-up
# or: docker-compose -f docker-compose.prod.yml up -d
```
- Frontend: http://localhost
- Backend: http://localhost:8000/api

### Option 3: Hugging Face Spaces (Recommended)
```bash
export HF_TOKEN=your_hf_token_here
export HF_SPACE_ID=username/space-name

make deploy-hf
# or: python scripts/deploy_hf.py
```
Then view at: https://huggingface.co/spaces/username/space-name

---

## 📁 New Files Created

### Configuration
- **`.env.example`** - Environment variables template
- **`Makefile`** - Easy command shortcuts

### Docker
- **`backend/Dockerfile`** - ✅ Updated with psycopg2 fix
- **`frontend/Dockerfile`** - ✅ New production build
- **`frontend/nginx.conf`** - ✅ New Nginx configuration
- **`docker-compose.dev.yml`** - ✅ Updated development setup
- **`docker-compose.prod.yml`** - ✅ New production setup

### Documentation
- **`DOCKER_QUICKSTART.md`** - Quick reference guide
- **`DOCKER_DEPLOYMENT_GUIDE.md`** - Comprehensive guide (500+ lines)
- **`DOCKER_IMPLEMENTATION_SUMMARY.md`** - Changes summary

### CI/CD & Deployment
- **`.github/workflows/docker-deploy.yml`** - GitHub Actions pipeline
- **`scripts/deploy_hf.py`** - Automated HF Spaces deployment

---

## 📖 Documentation Files

1. **[DOCKER_QUICKSTART.md](./DOCKER_QUICKSTART.md)** ⭐
   - 5-minute setup
   - Common commands
   - Quick troubleshooting

2. **[DOCKER_DEPLOYMENT_GUIDE.md](./DOCKER_DEPLOYMENT_GUIDE.md)** 📚
   - Complete deployment guide
   - All 3 HF deployment options
   - Docker commands reference
   - Advanced troubleshooting

3. **[DOCKER_IMPLEMENTATION_SUMMARY.md](./DOCKER_IMPLEMENTATION_SUMMARY.md)** ✅
   - All changes made
   - Fixed issues list
   - File structure
   - Deployment checklist

---

## 🛠️ Makefile Commands

Instead of typing long docker-compose commands, use simple commands:

```bash
# Development
make dev-up              # Start development
make dev-down            # Stop development
make dev-logs            # View all logs
make dev-shell-backend   # Access backend shell
make dev-shell-db        # Access database

# Production
make prod-build          # Build production images
make prod-up             # Start production
make prod-down           # Stop production
make prod-logs           # View logs

# Deployment
make deploy-hf           # Deploy to HF Spaces
make deploy-prod         # Deploy locally

# Health & Status
make health              # Check backend health
make diagnostics         # Get system diagnostics
make status              # Show all containers

# Database
make db-backup           # Backup database
make db-restore          # Restore database
make db-reset            # Reset database

# Cleanup
make clean               # Clean up containers
make clean-all           # Delete everything

# See all: make help
```

---

## 🐳 Docker Compose Files

### Development (`docker-compose.dev.yml`)
- Live reload for both frontend and backend
- Port 3000 (frontend), 8000 (backend)
- Volumes for development
- Interactive terminal support

### Production (`docker-compose.prod.yml`)
- Optimized images
- Port 80 (frontend), 8000 (backend)
- Full service stack ready
- Health checks and restart policies

---

## 🌐 Hugging Face Spaces Deployment

### Easiest Option: Streamlit + Docker
The `deploy_hf.py` script creates a Streamlit app that includes:
- 💬 Chat interface with RAG
- 📚 Documentation browser
- 📊 System diagnostics
- 🔗 Backend connection testing

### Manual Deployment
1. Create Space: https://huggingface.co/spaces/new
2. Select "Docker" runtime
3. Copy files to space directory
4. Set secrets in Space settings
5. Git push to deploy

---

## 🔍 Health Checks

Verify everything is running:

```bash
# Backend health
curl http://localhost:8000/health

# Full diagnostics
curl http://localhost:8000/diagnostics

# Database
docker-compose ps postgres

# All services
docker-compose ps
```

---

## 🧪 Testing

```bash
# Run tests
make test-backend

# Check all health endpoints
make test-health

# View logs during testing
make dev-logs
```

---

## 📊 Architecture

```
Your Application
├── Frontend (Docusaurus + React)
│   └── Nginx (production)
├── Backend (FastAPI)
│   ├── LLMs (OpenAI, Gemini, Cohere)
│   └── RAG (Qdrant Vector DB)
└── Data
    ├── PostgreSQL (user data)
    └── Qdrant (embeddings)
```

---

## 🚀 Production Deployment

### Cloud Platforms

**AWS ECS:**
```bash
# Push to ECR
docker tag physical-ai-backend:latest 123456789.dkr.ecr.us-east-1.amazonaws.com/physical-ai:latest
docker push 123456789.dkr.ecr.us-east-1.amazonaws.com/physical-ai:latest
```

**Google Cloud Run:**
```bash
gcloud run deploy physical-ai \
  --image gcr.io/your-project/physical-ai-backend:latest \
  --platform managed
```

**Kubernetes:**
```bash
# Use the provided k8s configs or Helm charts
helm install physical-ai ./k8s/
```

---

## 🐛 Common Issues & Fixes

### psycopg2 not found
```bash
# Already fixed in updated Dockerfile
docker-compose build --no-cache backend
```

### OpenAI client error
```bash
# Already fixed in updated openai_client.py
# Ensure API key is set in .env
```

### Port already in use
```bash
# Change ports in docker-compose.yml or kill the process
lsof -i :8000
kill -9 <PID>
```

### Database connection failed
```bash
# Check logs
docker-compose logs postgres

# Reset database
make db-reset
```

### Frontend can't connect to backend
```bash
# Check Nginx config
docker-compose exec frontend cat /etc/nginx/conf.d/default.conf

# Test connectivity from frontend
docker-compose exec frontend curl http://backend:8000/health
```

---

## 📚 Learning Resources

- [Docker Documentation](https://docs.docker.com/)
- [Docker Compose Guide](https://docs.docker.com/compose/)
- [Hugging Face Spaces Docs](https://huggingface.co/docs/hub/spaces)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [Docusaurus Documentation](https://docusaurus.io/)

---

## ✨ Features

✅ Multi-stage Docker builds (optimized images)
✅ Health checks for all services
✅ Automatic restart policies
✅ Database migrations support
✅ CORS enabled for frontend-backend communication
✅ API documentation (Swagger/OpenAPI)
✅ Logging and monitoring ready
✅ Security best practices (non-root user)
✅ Environment variable configuration
✅ Volume management for persistence

---

## 📝 Environment Variables

Key variables to set in `.env`:

```bash
# LLM APIs
OPENAI_API_KEY=sk-...
GEMINI_API_KEY=...
COHERE_API_KEY=...

# Qdrant
QDRANT_URL=https://...
QDRANT_API_KEY=...

# Database
POSTGRES_PASSWORD=...

# Deployment
ENVIRONMENT=production
```

Full list in: `.env.example`

---

## 🔐 Security

- ✅ Secrets via environment variables (not in code)
- ✅ Non-root containers
- ✅ Network isolation
- ✅ Health checks prevent zombie processes
- ✅ CORS properly configured
- ✅ Database credentials protected

---

## 🆘 Need Help?

1. Check relevant guide:
   - Quick answers: [DOCKER_QUICKSTART.md](./DOCKER_QUICKSTART.md)
   - Full guide: [DOCKER_DEPLOYMENT_GUIDE.md](./DOCKER_DEPLOYMENT_GUIDE.md)
   - Changes: [DOCKER_IMPLEMENTATION_SUMMARY.md](./DOCKER_IMPLEMENTATION_SUMMARY.md)

2. View logs: `make dev-logs` or `make prod-logs`

3. Run diagnostics: `make diagnostics`

4. Check health: `make health`

---

## ✅ Deployment Checklist

- [ ] Environment variables configured (`.env`)
- [ ] API keys added (OpenAI, Gemini, Cohere)
- [ ] Local test: `make dev-up` works
- [ ] Health check passes: `make health`
- [ ] Database backup created: `make db-backup`
- [ ] Production test: `make prod-up` works
- [ ] Images pushed to registry
- [ ] HF Spaces deployment: `make deploy-hf` successful
- [ ] Monitor first 24 hours
- [ ] Set up automated backups

---

## 🎉 Ready to Deploy!

Your application is now ready for production. Start with:

```bash
# 1. Test locally
make dev-up

# 2. Deploy to production
make deploy-hf
```

**Questions?** See [DOCKER_DEPLOYMENT_GUIDE.md](./DOCKER_DEPLOYMENT_GUIDE.md)

Happy deploying! 🚀
