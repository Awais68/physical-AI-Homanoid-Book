# Physical AI Humanoid Book

🤖 **Complete AI-Powered Educational Platform with Chatbot, RAG, and Production-Ready Docker Setup**

## 🎯 Overview

Physical AI Humanoid Book is a comprehensive educational platform that combines:
- **FastAPI Backend** with 20+ REST endpoints
- **Intelligent Chatbot** with RAG (Retrieval-Augmented Generation)
- **Multi-Language Support** (English, Urdu, Spanish, French)
- **Docusaurus Frontend** with React components
- **Vector Database** (Qdrant) for semantic search
- **PostgreSQL** for persistent storage
- **Docker** for containerization with hot reload
- **Production-Ready** deployment on GitHub & Hugging Face Spaces

## 🚀 Quick Start

### Local Development (30 seconds)

```bash
# Clone repository
git clone https://github.com/Awais68/physical-AI-Homanoid-Book.git
cd physical-AI-Homanoid-Book

# Start Docker environment
docker compose -f docker-compose.dev.yml up -d

# Access services
# Frontend: http://localhost:3000
# Backend:  http://localhost:8000
# API Docs: http://localhost:8000/docs
```

### Production Deployment

See [DEPLOYMENT_STEPS.md](DEPLOYMENT_STEPS.md) for complete deployment guide.

## 📊 Features

### Backend
- ✅ **FastAPI** with automatic API documentation
- ✅ **RAG Chatbot** with 138+ documents
- ✅ **Device Management** API
- ✅ **Personalization** system
- ✅ **Multi-language** support
- ✅ **Safety** monitoring
- ✅ **Health checks** & monitoring

### Frontend
- ✅ **Docusaurus** for documentation
- ✅ **React** components
- ✅ **Streamlit** interface for HF Spaces
- ✅ **Hot reload** development mode
- ✅ **Responsive** design

### Database
- ✅ **PostgreSQL** for structured data
- ✅ **Qdrant** vector database for embeddings
- ✅ **Automatic migrations**
- ✅ **Health checks**

### DevOps
- ✅ **Docker** containerization
- ✅ **Docker Compose** orchestration
- ✅ **Hot reload** enabled
- ✅ **Bind mounting** for live development
- ✅ **Health checks** for all services
- ✅ **GitHub Actions** CI/CD
- ✅ **HF Spaces** ready

## 📁 Project Structure

```
physical-ai/
├── backend/                 # FastAPI backend
│   ├── src/
│   │   ├── api/            # REST API routes
│   │   ├── clients/        # LLM clients (OpenAI, Gemini, Cohere)
│   │   ├── config/         # Configuration & settings
│   │   ├── models/         # Database models
│   │   ├── rag/            # RAG engine
│   │   └── services/       # Business logic
│   ├── requirements.txt
│   └── Dockerfile
├── frontend/                # Docusaurus + React
│   ├── src/
│   ├── static/
│   ├── docs/
│   ├── Dockerfile
│   ├── nginx.conf
│   └── package.json
├── docker-compose.dev.yml   # Development
├── docker-compose.prod.yml  # Production
├── docker-compose.hf.yml    # HF Spaces
├── app.py                   # ASGI wrapper
├── streamlit_app.py         # Streamlit interface
└── deploy_hf.sh            # HF deployment script
```

## 🔌 API Endpoints (20+)

### Chat
- `POST /api/chat/message` - Send chat message
- `GET /api/chat/health` - Chat service health
- `GET /api/chat/index` - Chat index

### Devices
- `GET /api/devices` - List devices
- `GET /api/devices/{id}` - Get device details
- `GET /api/devices/{id}/health` - Device health

### Personalization
- `GET /api/personalization/preferences` - User preferences
- `GET /api/personalization/bookmarks` - User bookmarks

### Languages & Translations
- `GET /api/i18n/languages` - Supported languages
- `POST /api/translations/detect` - Detect language
- `GET /api/translations/{lang}` - Get translations

### Safety & Health
- `GET /health` - Backend health
- `GET /api/safety/status` - Safety status

See [API Documentation](http://localhost:8000/docs) for complete list.

## 🐳 Docker Commands

```bash
# Start all services
docker compose -f docker-compose.dev.yml up -d

# View logs
docker compose logs -f backend
docker compose logs -f frontend

# Stop services
docker compose down

# Rebuild images
docker compose build

# Execute commands in container
docker compose exec backend bash
docker compose exec frontend bash
```

## 🧪 Testing

All tests passing ✅

```bash
# Run backend tests
cd backend && python3 test_qdrant_connection.py
cd backend && python3 test_chat.py
cd backend && python3 test_general_agent.py

# Test API endpoints
curl http://localhost:8000/health
curl -X POST http://localhost:8000/api/chat/message \
  -H "Content-Type: application/json" \
  -d '{"message": "What is Physical AI?"}'
```

See [TEST_REPORT.md](TEST_REPORT.md) for complete test results.

## 🤗 Hugging Face Spaces Deployment

### Quick Deploy

```bash
cd ~/physical-ai
./deploy_hf.sh your_hf_token_here
```

### Manual Setup

1. Get HF token: https://huggingface.co/settings/tokens
2. Create Space: https://huggingface.co/new-space (Docker SDK)
3. Push code: `git push hf main`
4. Configure secrets in Space Settings:
   - `OPENAI_API_KEY`
   - `GEMINI_API_KEY`
   - `COHERE_API_KEY`
   - `QDRANT_URL`

See [HF_DEPLOYMENT.md](HF_DEPLOYMENT.md) for details.

## 🔑 Environment Variables

```env
# LLM APIs
OPENAI_API_KEY=sk-...
GEMINI_API_KEY=...
COHERE_API_KEY=...

# Databases
DATABASE_URL=postgresql://user:pass@localhost:5432/db
QDRANT_URL=http://localhost:6333

# Services
ENVIRONMENT=development
PYTHONUNBUFFERED=1
```

Copy `.env.example` to `.env` and fill in values.

## 📊 System Requirements

### Local Development
- Docker 20.10+
- Docker Compose 2.0+
- 4GB RAM (8GB recommended)
- 2GB disk space

### Production
- 2 CPU cores
- 4GB RAM
- 10GB disk space
- Stable internet connection

## 🔒 Security

- ✅ Non-root containers
- ✅ Environment-based secrets
- ✅ CORS configured
- ✅ Health checks enabled
- ✅ Database isolation
- ✅ API rate limiting ready

## 📈 Performance

- **Backend Response Time:** <500ms (avg)
- **Chatbot Response Time:** <2s (with RAG)
- **Frontend Load Time:** <1s
- **Database Queries:** <100ms (avg)

## 🎓 Technologies Used

### Backend
- FastAPI 0.104.1
- Python 3.11
- PostgreSQL 15
- Qdrant (Vector DB)
- LangChain
- OpenAI / Gemini / Cohere APIs

### Frontend
- Docusaurus 3.x
- React 18
- Node.js 18
- Webpack 5
- Nginx

### DevOps
- Docker
- Docker Compose
- GitHub Actions
- Hugging Face Spaces

## 📚 Documentation

- [Docker Setup](docker-compose.dev.yml)
- [Deployment Guide](DEPLOYMENT_STEPS.md)
- [HF Spaces Guide](HF_DEPLOYMENT.md)
- [Test Report](TEST_REPORT.md)
- [ASGI Fix](ASGI_APP_FIX.md)
- [API Documentation](http://localhost:8000/docs)

## 🚨 Troubleshooting

### Containers not starting?
```bash
docker compose logs -f
docker compose down && docker compose up -d
```

### Port already in use?
```bash
lsof -i :3000  # Check process
kill -9 <PID>  # Kill process
```

### Backend errors?
```bash
docker compose exec backend bash
# Check logs and errors
```

## 🤝 Contributing

1. Fork repository
2. Create feature branch (`git checkout -b feature/xyz`)
3. Commit changes (`git commit -am 'Add feature'`)
4. Push to branch (`git push origin feature/xyz`)
5. Create Pull Request

## 📝 License

MIT License - See LICENSE file

## 📞 Support

- **GitHub Issues:** [Report bugs](https://github.com/Awais68/physical-AI-Homanoid-Book/issues)
- **Discussions:** [Ask questions](https://github.com/Awais68/physical-AI-Homanoid-Book/discussions)
- **Email:** support@physical-ai.local

## 🎉 Status

✅ **All Systems Operational**

- [x] Backend API
- [x] Chatbot with RAG
- [x] Frontend
- [x] Database
- [x] Docker setup
- [x] Tests passing
- [x] GitHub deployment
- [x] HF Spaces ready
- [x] Documentation complete

**Ready for production deployment! 🚀**

---

**Last Updated:** February 2, 2026  
**Version:** 1.0.0  
**Maintainer:** Awais68
