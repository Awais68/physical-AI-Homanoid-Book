# Physical AI Humanoid Book - Complete Deployment Guide

## Project Overview

This project is a comprehensive Physical AI platform featuring:
- RAG (Retrieval-Augmented Generation) with 134+ documents
- AI-powered chat using Gemini 2.5 Flash
- Semantic search with Qdrant vector database
- Educational robotics platform with safety controls
- Separate frontend (Docusaurus) and backend (FastAPI) services

## Architecture Components

### 1. Frontend Service
- **Location**: `/frontend/`
- **Technology**: Docusaurus-based documentation website
- **Features**: Built with Node.js (Node 20-alpine), internationalization (i18n) support
- **Ports**: 3000

### 2. Backend Service
- **Location**: `/backend/`
- **Technology**: Python 3.11 with FastAPI
- **Features**: Multiple API endpoints for devices, safety, users, chat, personalization
- **Ports**: 8000

### 3. Vector Database
- **Service**: Qdrant
- **Purpose**: Document embeddings for RAG functionality
- **Ports**: 6333 (REST), 6334 (gRPC)

### 4. Relational Database
- **Service**: PostgreSQL
- **Purpose**: User data and authentication storage
- **Ports**: 5432

## Docker Deployment

### Existing Docker Configuration

The project already includes comprehensive Docker configurations:

1. **Production Docker Compose**: `docker-compose.yml`
2. **Development Docker Compose**: `docker-compose.dev.yml`
3. **Hugging Face Deployment**: `docker-compose.hf.yml`

### Deploying with Docker

#### Prerequisites
- Docker and Docker Compose installed
- API keys for Gemini, OpenAI, and Cohere (optional but recommended)

#### Steps

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd physical-AI-Humanoid-Book
   ```

2. **Copy environment file**:
   ```bash
   cp .env.example .env
   ```

3. **Configure environment variables** (edit `.env` file):
   ```bash
   # API Keys (optional but recommended)
   GEMINI_API_KEY=your_gemini_api_key
   OPENAI_API_KEY=your_openai_api_key
   COHERE_API_KEY=your_cohere_api_key

   # Database configuration
   POSTGRES_USER=postgres
   POSTGRES_PASSWORD=postgres
   POSTGRES_DB=edgekit_db
   ```

4. **Start services**:
   ```bash
   # For production
   ./deploy.sh up

   # For development
   ./deploy.sh dev
   ```

5. **Access services**:
   - Frontend: http://localhost:3000
   - Backend: http://localhost:8000
   - Qdrant: http://localhost:6333

## Kubernetes Deployment

### Creating Kubernetes Manifests

#### 1. Namespace Configuration
```yaml
# k8s/namespace.yaml
apiVersion: v1
kind: Namespace
metadata:
  name: physical-ai
```

#### 2. PostgreSQL Deployment
```yaml
# k8s/postgres/postgres-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: postgres
  namespace: physical-ai
  labels:
    app: postgres
spec:
  replicas: 1
  selector:
    matchLabels:
      app: postgres
  template:
    metadata:
      labels:
        app: postgres
    spec:
      containers:
      - name: postgres
        image: postgres:15-alpine
        ports:
        - containerPort: 5432
        env:
        - name: POSTGRES_DB
          valueFrom:
            secretKeyRef:
              name: db-secret
              key: postgres-db
        - name: POSTGRES_USER
          valueFrom:
            secretKeyRef:
              name: db-secret
              key: postgres-user
        - name: POSTGRES_PASSWORD
          valueFrom:
            secretKeyRef:
              name: db-secret
              key: postgres-password
        volumeMounts:
        - name: postgres-storage
          mountPath: /var/lib/postgresql/data
      volumes:
      - name: postgres-storage
        persistentVolumeClaim:
          claimName: postgres-pvc

---
apiVersion: v1
kind: Service
metadata:
  name: postgres-service
  namespace: physical-ai
spec:
  selector:
    app: postgres
  ports:
  - protocol: TCP
    port: 5432
    targetPort: 5432
  type: ClusterIP
```

#### 3. PostgreSQL Persistent Volume Claim
```yaml
# k8s/postgres/postgres-pvc.yaml
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: postgres-pvc
  namespace: physical-ai
spec:
  accessModes:
    - ReadWriteOnce
  resources:
    requests:
      storage: 5Gi
```

#### 4. Qdrant Deployment
```yaml
# k8s/qdrant/qdrant-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: qdrant
  namespace: physical-ai
  labels:
    app: qdrant
spec:
  replicas: 1
  selector:
    matchLabels:
      app: qdrant
  template:
    metadata:
      labels:
        app: qdrant
    spec:
      containers:
      - name: qdrant
        image: qdrant/qdrant:latest
        ports:
        - containerPort: 6333
        - containerPort: 6334
        volumeMounts:
        - name: qdrant-storage
          mountPath: /qdrant/storage
      volumes:
      - name: qdrant-storage
        persistentVolumeClaim:
          claimName: qdrant-pvc

---
apiVersion: v1
kind: Service
metadata:
  name: qdrant-service
  namespace: physical-ai
spec:
  selector:
    app: qdrant
  ports:
  - name: rest
    protocol: TCP
    port: 6333
    targetPort: 6333
  - name: grpc
    protocol: TCP
    port: 6334
    targetPort: 6334
  type: ClusterIP
```

#### 5. Qdrant Persistent Volume Claim
```yaml
# k8s/qdrant/qdrant-pvc.yaml
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: qdrant-pvc
  namespace: physical-ai
spec:
  accessModes:
    - ReadWriteOnce
  resources:
    requests:
      storage: 10Gi
```

#### 6. Backend Deployment
```yaml
# k8s/backend/backend-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: backend
  namespace: physical-ai
  labels:
    app: backend
spec:
  replicas: 2
  selector:
    matchLabels:
      app: backend
  template:
    metadata:
      labels:
        app: backend
    spec:
      containers:
      - name: backend
        image: physical-ai-backend:latest
        ports:
        - containerPort: 8000
        env:
        - name: QDRANT_URL
          value: "http://qdrant-service:6333"
        - name: QDRANT_COLLECTION
          value: "physical_ai_docs"
        - name: DATABASE_URL
          value: "postgresql://$(POSTGRES_USER):$(POSTGRES_PASSWORD)@postgres-service:5432/$(POSTGRES_DB)"
        - name: GEMINI_API_KEY
          valueFrom:
            secretKeyRef:
              name: ai-api-keys
              key: gemini-api-key
        - name: OPENAI_API_KEY
          valueFrom:
            secretKeyRef:
              name: ai-api-keys
              key: openai-api-key
        - name: COHERE_API_KEY
          valueFrom:
            secretKeyRef:
              name: ai-api-keys
              key: cohere-api-key
        - name: ENVIRONMENT
          value: "kubernetes"
        livenessProbe:
          httpGet:
            path: /health
            port: 8000
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /health
            port: 8000
          initialDelaySeconds: 5
          periodSeconds: 5
        resources:
          requests:
            memory: "256Mi"
            cpu: "250m"
          limits:
            memory: "512Mi"
            cpu: "500m"

---
apiVersion: v1
kind: Service
metadata:
  name: backend-service
  namespace: physical-ai
spec:
  selector:
    app: backend
  ports:
  - protocol: TCP
    port: 8000
    targetPort: 8000
  type: ClusterIP
```

#### 7. Frontend Deployment
```yaml
# k8s/frontend/frontend-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: frontend
  namespace: physical-ai
  labels:
    app: frontend
spec:
  replicas: 2
  selector:
    matchLabels:
      app: frontend
  template:
    metadata:
      labels:
        app: frontend
    spec:
      containers:
      - name: frontend
        image: physical-ai-frontend:latest
        ports:
        - containerPort: 3000
        env:
        - name: BACKEND_API_URL
          value: "http://backend-service:8000"
        - name: NODE_ENV
          value: "production"
        livenessProbe:
          httpGet:
            path: /
            port: 3000
          initialDelaySeconds: 10
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /
            port: 3000
          initialDelaySeconds: 5
          periodSeconds: 5
        resources:
          requests:
            memory: "128Mi"
            cpu: "100m"
          limits:
            memory: "256Mi"
            cpu: "200m"

---
apiVersion: v1
kind: Service
metadata:
  name: frontend-service
  namespace: physical-ai
spec:
  selector:
    app: frontend
  ports:
  - protocol: TCP
    port: 3000
    targetPort: 3000
  type: ClusterIP
```

#### 8. Ingress Configuration
```yaml
# k8s/ingress.yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: physical-ai-ingress
  namespace: physical-ai
  annotations:
    nginx.ingress.kubernetes.io/rewrite-target: /
spec:
  ingressClassName: nginx
  rules:
  - host: physical-ai.local
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: frontend-service
            port:
              number: 3000
      - path: /api
        pathType: Prefix
        backend:
          service:
            name: backend-service
            port:
              number: 8000
      - path: /qdrant
        pathType: Prefix
        backend:
          service:
            name: qdrant-service
            port:
              number: 6333
```

#### 9. Secrets Configuration
```yaml
# k8s/secrets.yaml
apiVersion: v1
kind: Secret
metadata:
  name: db-secret
  namespace: physical-ai
type: Opaque
data:
  postgres-db: ZWRnZWtpdF9kYg==  # edgekit_db
  postgres-user: cG9zdGdyZXM=    # postgres
  postgres-password: cG9zdGdyZXM= # postgres

---
apiVersion: v1
kind: Secret
metadata:
  name: ai-api-keys
  namespace: physical-ai
type: Opaque
data:
  gemini-api-key: <base64-encoded-key>
  openai-api-key: <base64-encoded-key>
  cohere-api-key: <base64-encoded-key>
```

### Kubernetes Deployment Steps

#### Prerequisites
- Kubernetes cluster (Minikube, Kind, EKS, GKE, AKS, etc.)
- kubectl configured to connect to the cluster
- Docker installed for building images

#### Steps

1. **Build Docker Images**:
   ```bash
   # Build backend image
   cd backend
   docker build -t physical-ai-backend:latest .
   cd ..

   # Build frontend image
   cd frontend
   docker build -t physical-ai-frontend:latest -f Dockerfile.prod .
   cd ..
   ```

2. **Push Images to Registry** (if using remote cluster):
   ```bash
   # Tag and push images to registry
   docker tag physical-ai-backend:latest <registry>/physical-ai-backend:latest
   docker tag physical-ai-frontend:latest <registry>/physical-ai-frontend:latest
   docker push <registry>/physical-ai-backend:latest
   docker push <registry>/physical-ai-frontend:latest
   ```

   Update the image names in the deployment files accordingly.

3. **Create Kubernetes Manifests Directory**:
   ```bash
   mkdir -p k8s/{postgres,qdrant,backend,frontend}
   ```

4. **Apply Kubernetes Configurations**:
   ```bash
   # Create namespace
   kubectl apply -f k8s/namespace.yaml

   # Create secrets (update with your actual keys first!)
   kubectl apply -f k8s/secrets.yaml

   # Create PVCs
   kubectl apply -f k8s/postgres/postgres-pvc.yaml
   kubectl apply -f k8s/qdrant/qdrant-pvc.yaml

   # Deploy databases
   kubectl apply -f k8s/postgres/postgres-deployment.yaml
   kubectl apply -f k8s/qdrant/qdrant-deployment.yaml

   # Wait for databases to be ready
   kubectl rollout status deployment/postgres -n physical-ai
   kubectl rollout status deployment/qdrant -n physical-ai

   # Deploy services
   kubectl apply -f k8s/backend/backend-deployment.yaml
   kubectl apply -f k8s/frontend/frontend-deployment.yaml

   # Apply ingress
   kubectl apply -f k8s/ingress.yaml
   ```

5. **Verify Deployment**:
   ```bash
   # Check all pods
   kubectl get pods -n physical-ai

   # Check all services
   kubectl get svc -n physical-ai

   # Check ingress
   kubectl get ingress -n physical-ai
   ```

6. **Access Services**:
   - Frontend: http://physical-ai.local (or the configured ingress hostname)
   - Backend API: http://physical-ai.local/api
   - Qdrant: http://physical-ai.local/qdrant

## RAG Chatbot Functionality

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

## Deployment Considerations

### Security
- Use secrets for API keys and passwords
- Implement proper network policies
- Enable TLS/SSL for production deployments
- Regular security scanning of container images

### Scalability
- Horizontal Pod Autoscaling (HPA) for backend and frontend
- Database connection pooling
- CDN for frontend assets
- Load balancing for high availability

### Monitoring
- Prometheus for metrics collection
- Grafana for dashboard visualization
- Logging aggregation with ELK stack
- Health checks and readiness probes

### Backup and Recovery
- Regular database backups
- Persistent volume snapshots
- Disaster recovery procedures
- Version control for configurations

## Troubleshooting

### Common Issues
1. **Database Connection Failures**: Verify environment variables and network connectivity
2. **Qdrant Connection Issues**: Check service availability and API keys
3. **API Key Errors**: Validate API key formats and quotas
4. **Resource Constraints**: Adjust resource limits in deployment files

### Debugging Commands
```bash
# Check pod logs
kubectl logs -f deployment/backend -n physical-ai
kubectl logs -f deployment/frontend -n physical-ai

# Port forward for debugging
kubectl port-forward service/backend-service 8000:8000 -n physical-ai
kubectl port-forward service/qdrant-service 6333:6333 -n physical-ai

# Describe resources for detailed status
kubectl describe pod <pod-name> -n physical-ai
```