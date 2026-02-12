#!/bin/bash

# Kubernetes Deployment Script for Physical AI Edge Kit
# Separates frontend and backend services with Kubernetes

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚀 Physical AI Edge Kit Kubernetes Deployment Script${NC}"
echo "====================================================="

# Check if kubectl is installed
if ! command -v kubectl &> /dev/null; then
    echo -e "${RED}❌ kubectl is not installed. Please install kubectl first.${NC}"
    exit 1
fi

# Check if docker is installed (needed for building images)
if ! command -v docker &> /dev/null; then
    echo -e "${RED}❌ Docker is not installed. Please install Docker first.${NC}"
    exit 1
fi

# Function to display usage
usage() {
    echo "Usage: $0 [COMMAND]"
    echo "Commands:"
    echo "  setup       - Setup Kubernetes cluster (Minikube/Docker Desktop)"
    echo "  build       - Build Docker images for the services"
    echo "  push        - Push images to registry (if using remote cluster)"
    echo "  deploy      - Deploy all services to Kubernetes"
    echo "  undeploy    - Remove all services from Kubernetes"
    echo "  status      - Show status of all Kubernetes resources"
    echo "  logs        - Show logs from all services"
    echo "  port-forward - Port forward services for local testing"
    echo "  help        - Show this help message"
}

# Function to setup local Kubernetes (Minikube)
setup_minikube() {
    echo -e "${GREEN}🔧 Setting up Minikube...${NC}"

    if ! command -v minikube &> /dev/null; then
        echo -e "${RED}❌ Minikube is not installed. Please install Minikube first.${NC}"
        echo -e "${YELLOW}Visit: https://minikube.sigs.k8s.io/docs/start/${NC}"
        exit 1
    fi

    echo -e "${BLUE}Starting Minikube...${NC}"
    minikube start --memory=4096 --cpus=2

    echo -e "${BLUE}Enabling ingress addon...${NC}"
    minikube addons enable ingress

    echo -e "${GREEN}✅ Minikube setup completed!${NC}"
}

# Function to setup Docker Desktop Kubernetes (alternative)
setup_docker_desktop() {
    echo -e "${GREEN}🔧 Setting up Docker Desktop Kubernetes...${NC}"
    echo -e "${YELLOW}Please ensure Kubernetes is enabled in Docker Desktop Settings${NC}"

    # Check if Docker Desktop Kubernetes is enabled
    if kubectl cluster-info &> /dev/null; then
        echo -e "${GREEN}✅ Docker Desktop Kubernetes is already enabled${NC}"
    else
        echo -e "${RED}❌ Docker Desktop Kubernetes is not enabled${NC}"
        echo -e "${YELLOW}Please enable Kubernetes in Docker Desktop Settings > Kubernetes${NC}"
        exit 1
    fi
}

# Main command execution
case "${1:-help}" in
    "setup")
        echo -e "${GREEN}🔧 Setting up Kubernetes environment...${NC}"

        # Try to detect which Kubernetes environment to use
        if command -v minikube &> /dev/null && ! kubectl cluster-info &> /dev/null; then
            setup_minikube
        elif kubectl cluster-info &> /dev/null; then
            setup_docker_desktop
        else
            setup_minikube
        fi

        echo -e "${GREEN}✅ Kubernetes environment setup completed!${NC}"
        ;;

    "build")
        echo -e "${GREEN}🔨 Building Docker images...${NC}"

        # Build backend image
        echo -e "${BLUE}Building backend image...${NC}"
        cd backend
        docker build -t physical-ai-backend:latest .
        cd ..

        # Build frontend image
        echo -e "${BLUE}Building frontend image...${NC}"
        cd frontend
        docker build -t physical-ai-frontend:latest -f Dockerfile.prod .
        cd ..

        echo -e "${GREEN}✅ Docker images built successfully!${NC}"
        ;;

    "push")
        echo -e "${GREEN}📤 Pushing Docker images to registry...${NC}"
        echo -e "${YELLOW}Please configure your registry and update image names in deployment files${NC}"

        # Uncomment and customize these lines for your registry
        # docker tag physical-ai-backend:latest <your-registry>/physical-ai-backend:latest
        # docker tag physical-ai-frontend:latest <your-registry>/physical-ai-frontend:latest
        # docker push <your-registry>/physical-ai-backend:latest
        # docker push <your-registry>/physical-ai-frontend:latest

        echo -e "${YELLOW}💡 Remember to update image names in k8s deployment files${NC}"
        ;;

    "deploy")
        echo -e "${GREEN}🚀 Deploying Physical AI Edge Kit to Kubernetes...${NC}"

        # Create namespace
        echo -e "${BLUE}Creating namespace...${NC}"
        kubectl apply -f k8s/namespace.yaml

        # Create secrets (if they exist)
        if [ -f "k8s/secrets.yaml" ]; then
            echo -e "${BLUE}Creating secrets...${NC}"
            kubectl apply -f k8s/secrets.yaml
        else
            echo -e "${YELLOW}⚠️  Secrets file not found. Using template...${NC}"
            echo -e "${YELLOW}Please create k8s/secrets.yaml with your actual API keys${NC}"
        fi

        # Create PVCs
        echo -e "${BLUE}Creating PersistentVolumeClaims...${NC}"
        kubectl apply -f k8s/postgres/postgres-pvc.yaml
        kubectl apply -f k8s/qdrant/qdrant-pvc.yaml

        # Deploy databases
        echo -e "${BLUE}Deploying databases...${NC}"
        kubectl apply -f k8s/postgres/postgres-deployment.yaml
        kubectl apply -f k8s/qdrant/qdrant-deployment.yaml

        # Wait for databases to be ready
        echo -e "${BLUE}Waiting for databases to be ready...${NC}"
        kubectl rollout status deployment/postgres -n physical-ai --timeout=300s
        kubectl rollout status deployment/qdrant -n physical-ai --timeout=300s

        # Deploy services
        echo -e "${BLUE}Deploying backend service...${NC}"
        kubectl apply -f k8s/backend/backend-deployment.yaml

        echo -e "${BLUE}Deploying frontend service...${NC}"
        kubectl apply -f k8s/frontend/frontend-deployment.yaml

        # Apply ingress
        echo -e "${BLUE}Applying ingress configuration...${NC}"
        kubectl apply -f k8s/ingress.yaml

        echo -e "${GREEN}✅ Physical AI Edge Kit deployed successfully!${NC}"
        echo -e "${BLUE}📊 Services status:${NC}"
        kubectl get all -n physical-ai
        ;;

    "undeploy")
        echo -e "${RED}🗑️ Removing Physical AI Edge Kit from Kubernetes...${NC}"

        # Remove ingress first
        kubectl delete -f k8s/ingress.yaml || true

        # Remove services
        kubectl delete -f k8s/frontend/frontend-deployment.yaml || true
        kubectl delete -f k8s/backend/backend-deployment.yaml || true

        # Remove databases
        kubectl delete -f k8s/qdrant/qdrant-deployment.yaml || true
        kubectl delete -f k8s/postgres/postgres-deployment.yaml || true

        # Remove PVCs
        kubectl delete -f k8s/qdrant/qdrant-pvc.yaml || true
        kubectl delete -f k8s/postgres/postgres-pvc.yaml || true

        # Remove secrets
        kubectl delete -f k8s/secrets.yaml || true

        # Remove namespace
        kubectl delete -f k8s/namespace.yaml || true

        echo -e "${GREEN}✅ Physical AI Edge Kit removed successfully!${NC}"
        ;;

    "status")
        echo -e "${BLUE}📊 Current status of Physical AI services:${NC}"
        kubectl get all -n physical-ai
        echo -e ""
        echo -e "${BLUE}📊 Storage status:${NC}"
        kubectl get pvc -n physical-ai
        echo -e ""
        echo -e "${BLUE}📊 Ingress status:${NC}"
        kubectl get ingress -n physical-ai
        ;;

    "logs")
        echo -e "${BLUE}📋 Showing logs from backend service...${NC}"
        kubectl logs -l app=backend -n physical-ai --tail=50
        echo -e ""
        echo -e "${BLUE}📋 Showing logs from frontend service...${NC}"
        kubectl logs -l app=frontend -n physical-ai --tail=50
        ;;

    "port-forward")
        echo -e "${GREEN}🔗 Setting up port forwarding for local testing...${NC}"
        echo -e "${YELLOW}Press Ctrl+C to stop port forwarding${NC}"
        echo -e ""
        echo -e "${BLUE}Forwarding ports:${NC}"
        echo -e "  - Local port 3000 -> frontend-service:3000"
        echo -e "  - Local port 8000 -> backend-service:8000"
        echo -e "  - Local port 6333 -> qdrant-service:6333"
        echo -e ""
        echo -e "${YELLOW}Access services:${NC}"
        echo -e "  - Frontend: http://localhost:3000"
        echo -e "  - Backend API: http://localhost:8000"
        echo -e "  - Qdrant: http://localhost:6333"

        # Setup port forwarding
        kubectl port-forward service/frontend-service 3000:3000 -n physical-ai &
        FRONTEND_PID=$!

        kubectl port-forward service/backend-service 8000:8000 -n physical-ai &
        BACKEND_PID=$!

        kubectl port-forward service/qdrant-service 6333:6333 -n physical-ai &
        QDRANT_PID=$!

        # Wait for all processes
        wait $FRONTEND_PID $BACKEND_PID $QDRANT_PID
        ;;

    "help"|*)
        usage
        ;;
esac

echo -e "${NC}"