# Physical AI Humanoid Book - Deployment Documentation

## Table of Contents
1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Docker Deployment](#docker-deployment)
4. [Kubernetes Deployment](#kubernetes-deployment)
5. [RAG Chatbot Features](#rag-chatbot-features)
6. [Configuration](#configuration)
7. [Troubleshooting](#troubleshooting)

## Overview

This project is a comprehensive Physical AI platform featuring:
- **RAG (Retrieval-Augmented Generation)** with 134+ documents
- **AI-powered chat** using Gemini 2.5 Flash
- **Semantic search** with Qdrant vector database
- **Educational robotics platform** with safety controls
- **Separate frontend (Docusaurus) and backend (FastAPI) services**

## Architecture

The system consists of four main components:

### 1. Frontend Service
- **Location**: `/frontend/`
- **Technology**: Docusaurus-based documentation website
- **Features**: Built with Node.js (Node 20-alpine), internationalization (i18n) support
- **Port**: 3000

### 2. Backend Service
- **Location**: `/backend/`
- **Technology**: Python 3.11 with FastAPI
- **Features**: Multiple API endpoints for devices, safety, users, chat, personalization
- **Port**: 8000

### 3. Vector Database
- **Service**: Qdrant
- **Purpose**: Document embeddings for RAG functionality
- **Ports**: 6333 (REST), 6334 (gRPC)

### 4. Relational Database
- **Service**: PostgreSQL
- **Purpose**: User data and authentication storage
- **Port**: 5432

## Docker Deployment

### Prerequisites
- Docker and Docker Compose installed
- At least 4GB RAM available
- API keys for Gemini, OpenAI, and Cohere (recommended)

### Quick Start

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd physical-AI-Humanoid-Book
   ```

2. **Set up environment variables**:
   ```bash
   cp .env.example .env
   # Edit .env to add your API keys (optional but recommended)
   ```

3. **Start the services**:
   ```bash
   # Production mode
   ./deploy.sh up

   # Development mode
   ./deploy.sh dev
   ```

4. **Access the applications**:
   - Frontend: http://localhost:3000
   - Backend API: http://localhost:8000
   - Qdrant: http://localhost:6333

### Available Commands

```bash
./deploy.sh up          # Start production services
./deploy.sh dev         # Start development services
./deploy.sh down        # Stop all services
./deploy.sh logs        # Show service logs
./deploy.sh build       # Build all services
./deploy.sh rebuild     # Rebuild all services (no cache)
./deploy.sh status      # Show service status
```

## Kubernetes Deployment

### Prerequisites
- Kubernetes cluster (Minikube, Docker Desktop, EKS, GKE, AKS, etc.)
- kubectl configured to connect to the cluster
- Docker installed for building images

### Quick Start

1. **Setup Kubernetes environment**:
   ```bash
   ./k8s-deploy.sh setup
   ```

2. **Build Docker images**:
   ```bash
   ./k8s-deploy.sh build
   ```

3. **Deploy to Kubernetes**:
   ```bash
   ./k8s-deploy.sh deploy
   ```

4. **Access the applications**:
   - Frontend: http://physical-ai.local (after configuring DNS/hosts)
   - Backend API: http://physical-ai.local/api
   - Qdrant: http://physical-ai.local/qdrant

### Kubernetes Commands

```bash
./k8s-deploy.sh setup           # Setup Kubernetes environment
./k8s-deploy.sh build           # Build Docker images
./k8s-deploy.sh deploy          # Deploy to Kubernetes
./k8s-deploy.sh undeploy        # Remove from Kubernetes
./k8s-deploy.sh status          # Show resource status
./k8s-deploy.sh logs            # Show service logs
./k8s-deploy.sh port-forward    # Port forward for local testing
```

### Manual Kubernetes Deployment

If you prefer to deploy manually:

1. **Create namespace**:
   ```bash
   kubectl apply -f k8s/namespace.yaml
   ```

2. **Create secrets** (update with your API keys first):
   ```bash
   # Encode your API keys in base64 format
   # Then create k8s/secrets.yaml with actual values
   kubectl apply -f k8s/secrets.yaml
   ```

3. **Create persistent volumes**:
   ```bash
   kubectl apply -f k8s/postgres/postgres-pvc.yaml
   kubectl apply -f k8s/qdrant/qdrant-pvc.yaml
   ```

4. **Deploy databases**:
   ```bash
   kubectl apply -f k8s/postgres/postgres-deployment.yaml
   kubectl apply -f k8s/qdrant/qdrant-deployment.yaml
   ```

5. **Deploy services**:
   ```bash
   kubectl apply -f k8s/backend/backend-deployment.yaml
   kubectl apply -f k8s/frontend/frontend-deployment.yaml
   ```

6. **Apply ingress**:
   ```bash
   kubectl apply -f k8s/ingress.yaml
   ```

## RAG Chatbot Features

The RAG (Retrieval-Augmented Generation) chatbot is a core feature of this platform:

### How it Works
1. **Document Ingestion**: Documents are ingested into Qdrant vector database with embeddings
2. **Query Processing**: User queries are converted to embeddings using Gemini API
3. **Similarity Search**: Qdrant performs similarity search to find relevant documents
4. **Contextual Answering**: Gemini generates answers based on retrieved context

### API Endpoints
- `/chat/message` - Send messages to the RAG chatbot
- `/chat/selected-text` - Query about selected text content
- `/chat/index` - Index new documents for RAG retrieval
- `/chat/health` - Check RAG chatbot health

### Features
- Context-aware responses using conversation history
- Source citations with document references
- Confidence scoring for answers
- Session management for conversations
- Support for technical and educational content

## Configuration

### Environment Variables

The following environment variables can be configured in the `.env` file:

#### Database Configuration
- `POSTGRES_USER` - PostgreSQL username (default: postgres)
- `POSTGRES_PASSWORD` - PostgreSQL password (default: postgres)
- `POSTGRES_DB` - PostgreSQL database name (default: edgekit_db)

#### AI API Keys (recommended for full functionality)
- `GEMINI_API_KEY` - Google Gemini API key
- `OPENAI_API_KEY` - OpenAI API key
- `COHERE_API_KEY` - Cohere API key

#### Application Settings
- `BASE_URL` - Base URL for the application
- `BACKEND_API_URL` - Backend API URL for frontend

### Kubernetes Secrets

For Kubernetes deployment, create `k8s/secrets.yaml` with your actual API keys:

```yaml
apiVersion: v1
kind: Secret
metadata:
  name: ai-api-keys
  namespace: physical-ai
type: Opaque
data:
  gemini-api-key: <base64-encoded-gemini-key>
  openai-api-key: <base64-encoded-openai-key>
  cohere-api-key: <base64-encoded-cohere-key>
```

## Troubleshooting

### Common Issues

1. **Database Connection Failures**:
   - Check environment variables and network connectivity
   - Verify PostgreSQL is running and accessible

2. **Qdrant Connection Issues**:
   - Check Qdrant service availability
   - Verify Qdrant URL configuration

3. **API Key Errors**:
   - Validate API key formats and quotas
   - Ensure API keys are properly configured in secrets

4. **Resource Constraints**:
   - Adjust resource limits in deployment files
   - Ensure sufficient memory and CPU available

### Debugging Commands

```bash
# Check all pods
kubectl get pods -n physical-ai

# Check pod logs
kubectl logs -f deployment/backend -n physical-ai
kubectl logs -f deployment/frontend -n physical-ai

# Port forward for local debugging
kubectl port-forward service/backend-service 8000:8000 -n physical-ai
kubectl port-forward service/qdrant-service 6333:6333 -n physical-ai

# Describe resources for detailed status
kubectl describe pod <pod-name> -n physical-ai

# Check service status
kubectl get svc -n physical-ai
kubectl get ingress -n physical-ai
```

### Docker Debugging

```bash
# Check Docker containers
docker ps

# View container logs
docker logs <container-id>

# Check Docker Compose services
docker compose ps

# View logs for specific service
docker compose logs backend
docker compose logs frontend
```

## Scaling and Production Considerations

### Horizontal Scaling
- Use HPA (Horizontal Pod Autoscaler) for backend and frontend
- Configure database connection pooling
- Implement load balancing

### Security
- Use TLS/SSL for production deployments
- Implement proper RBAC (Role-Based Access Control)
- Scan container images for vulnerabilities
- Rotate API keys regularly

### Monitoring
- Set up Prometheus for metrics collection
- Configure Grafana for dashboards
- Implement centralized logging
- Configure health checks and alerts

### Backup and Recovery
- Regular database backups
- Persistent volume snapshots
- Disaster recovery procedures
- Version control for configurations

## Development

### Local Development
- Use `./deploy.sh dev` for development mode
- Frontend will auto-reload on file changes
- Backend will restart on code changes

### Building Custom Images
- Backend: `docker build -t physical-ai-backend:latest -f backend/Dockerfile .`
- Frontend: `docker build -t physical-ai-frontend:latest -f frontend/Dockerfile.prod .`

---

For support or questions, please refer to the main repository documentation or contact the development team.