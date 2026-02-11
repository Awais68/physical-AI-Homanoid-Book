# 🚀 DOCKER DEPLOYMENT - REFERENCE CARD

## ⚡ One-Line Commands

```bash
# Start development (hot reload)
make dev-up

# Start production (local)
make prod-up

# Deploy to Hugging Face Spaces
HF_TOKEN=xxx HF_SPACE_ID=user/space make deploy-hf

# Check system health
make health

# View all logs
make dev-logs

# Full diagnostics
make diagnostics

# Backup database
make db-backup

# Clean up
make clean
```

---

## 📚 Documentation Map

| Need | File | Lines |
|------|------|-------|
| **Quick Start** | DOCKER_QUICKSTART.md | 200+ |
| **Main Guide** | DOCKER_README.md | 300+ |
| **Full Reference** | DOCKER_DEPLOYMENT_GUIDE.md | 500+ |
| **Technical Details** | DOCKER_IMPLEMENTATION_SUMMARY.md | 400+ |
| **All Commands** | Makefile | 300+ |

---

## 🔧 Service URLs

| Service | Dev | Prod |
|---------|-----|------|
| Frontend | http://localhost:3000 | http://localhost |
| Backend | http://localhost:8000 | http://localhost:8000 |
| Docs | http://localhost:8000/docs | http://localhost:8000/docs |
| Qdrant | http://localhost:6333 | Internal |
| Database | localhost:5432 | Internal |

---

## 📋 Makefile Commands (Top 20)

```bash
# Development
make dev-up              # Start with hot reload
make dev-down            # Stop services
make dev-logs            # View logs
make dev-shell-backend   # Backend shell

# Production
make prod-up             # Start production
make prod-build          # Build images
make prod-logs           # View logs

# Deployment
make deploy-hf           # Deploy to HF Spaces
make deploy-prod         # Deploy locally

# Health
make health              # Backend health
make diagnostics         # Full diagnostics
make status              # Service status

# Database
make db-backup           # Backup DB
make db-restore          # Restore DB
make db-reset            # Reset DB

# Cleanup
make clean               # Clean containers
make help                # All commands
```

---

## 🔐 Required Environment Variables

```bash
# In .env file:

# LLM APIs (at least one required)
OPENAI_API_KEY=sk-...
GEMINI_API_KEY=...
COHERE_API_KEY=...

# Qdrant (optional - uses local by default)
QDRANT_URL=https://...
QDRANT_API_KEY=...

# Database (auto-configured)
POSTGRES_PASSWORD=postgres
POSTGRES_DB=edgekit_db
```

---

## ✅ Verification Checklist

```bash
# 1. Services running
docker-compose ps

# 2. Backend health
curl http://localhost:8000/health

# 3. Full diagnostics
curl http://localhost:8000/diagnostics

# 4. Database connected
docker-compose exec postgres pg_isready

# 5. Frontend accessible
curl http://localhost:3000 (dev) or http://localhost (prod)

# 6. Logs clean
docker-compose logs | grep -i error
```

---

## 🐛 Quick Fixes

| Problem | Fix |
|---------|-----|
| Port in use | `make clean` or `kill -9 <PID>` |
| DB won't start | `docker-compose logs postgres` |
| Frontend can't reach backend | Check nginx.conf in container |
| API key error | Verify .env file |
| Image build fails | `docker-compose build --no-cache` |
| Memory issues | Increase Docker memory limit |

---

## 📊 File Structure

```
project/
├── backend/
│   ├── Dockerfile ✅
│   └── src/
│       └── clients/openai_client.py ✅
├── frontend/
│   ├── Dockerfile ✅ NEW
│   └── nginx.conf ✅ NEW
├── docker-compose.dev.yml ✅
├── docker-compose.prod.yml ✅ NEW
├── .env.example ✅
├── Makefile ✅ NEW
├── DOCKER_*.md ✅ NEW (4 files)
├── scripts/deploy_hf.py ✅ NEW
└── .github/workflows/docker-deploy.yml ✅ NEW
```

---

## 🎯 Deployment Paths

### Path 1: Local Dev
```bash
cp .env.example .env
make dev-up
# http://localhost:3000
```

### Path 2: Local Prod
```bash
make prod-up
# http://localhost
```

### Path 3: Hugging Face Spaces
```bash
export HF_TOKEN=your_token
export HF_SPACE_ID=user/space
make deploy-hf
# https://huggingface.co/spaces/user/space
```

### Path 4: Cloud (AWS/GCP/Azure)
```bash
make prod-build
docker tag physical-ai-backend:latest registry/physical-ai-backend:latest
docker push registry/physical-ai-backend:latest
# Deploy using your cloud platform
```

---

## 🔍 Debugging Tips

```bash
# View specific logs
docker-compose logs backend
docker-compose logs frontend
docker-compose logs postgres

# Enter container
docker-compose exec backend bash
docker-compose exec postgres psql -U postgres

# Check resource usage
docker stats

# Test endpoints
curl http://localhost:8000/health
curl http://localhost:8000/diagnostics

# Database operations
make db-backup
make db-restore
```

---

## 📈 Performance Tuning

```bash
# Reduce image size
docker image prune -a

# Optimize builds
DOCKER_BUILDKIT=1 docker-compose build

# Check container sizes
docker images --format "table {{.Repository}}\t{{.Size}}"

# Monitor usage
docker stats
```

---

## 🔑 Environment Setup

```bash
# 1. Copy template
cp .env.example .env

# 2. Edit with your keys
nano .env
# or
vim .env

# 3. Required keys:
OPENAI_API_KEY=sk-...       # Get from: https://platform.openai.com
GEMINI_API_KEY=...          # Get from: https://makersuite.google.com
COHERE_API_KEY=...          # Get from: https://dashboard.cohere.com

# 4. Optional (uses local by default):
QDRANT_URL=https://...      # Your Qdrant Cloud instance
QDRANT_API_KEY=...
```

---

## 🚀 Three-Step Deployment

### Step 1: Test Locally
```bash
make dev-up
make health
make diagnostics
```

### Step 2: Build Production
```bash
make prod-build
```

### Step 3: Deploy
```bash
# Option A: Local production
make prod-up

# Option B: Hugging Face Spaces
make deploy-hf

# Option C: Cloud platform
docker push your-registry/physical-ai-backend:latest
```

---

## ✨ Key Features

| Feature | Status |
|---------|--------|
| Multi-stage builds | ✅ |
| Health checks | ✅ |
| Auto-restart | ✅ |
| CORS enabled | ✅ |
| Database persistence | ✅ |
| Nginx reverse proxy | ✅ |
| CI/CD pipeline | ✅ |
| HF Spaces integration | ✅ |
| Environment config | ✅ |
| Security hardened | ✅ |

---

## 📞 Need Help?

1. **Quick answers**: DOCKER_QUICKSTART.md
2. **Common issues**: DOCKER_DEPLOYMENT_GUIDE.md #Troubleshooting
3. **All commands**: `make help`
4. **View logs**: `docker-compose logs -f`
5. **Diagnostics**: `make diagnostics`

---

## 🎓 Learning Path

| Week | Goal | Command |
|------|------|---------|
| 1 | Local dev | `make dev-up` |
| 2 | HF deployment | `make deploy-hf` |
| 3 | CI/CD | GitHub Actions enabled |
| 4 | Cloud scale | AWS/GCP/Azure |

---

**Now you're ready! Start with:** `make dev-up` 🚀
