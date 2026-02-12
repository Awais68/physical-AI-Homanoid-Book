#!/bin/bash

# Kubernetes Deployment Script for Humainoid Robotics
# Deploys to HuggingFace Space at: https://huggingface.co/spaces/Awais68/Humainoid-robotics

set -e

NAMESPACE="humanoid-robotics"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "🚀 Deploying Humainoid Robotics to Kubernetes..."
echo "============================================="

# Check if kubectl is installed
if ! command -v kubectl &> /dev/null; then
    echo "❌ kubectl is not installed. Please install kubectl first."
    exit 1
fi

# Build the Docker image
echo "🔨 Building Docker image..."
cd "${SCRIPT_DIR}/.."
docker build -t humanoid-robotics:latest .

# Create namespace
echo "📦 Creating namespace..."
kubectl apply -f "${SCRIPT_DIR}/namespace.yaml"

# Apply secrets (after replacing placeholder values)
echo "🔑 Applying secrets..."
read -p "Enter Gemini API Key: " GEMINI_API_KEY
read -p "Enter OpenAI API Key: " OPENAI_API_KEY

# Create secrets with user input
kubectl create secret generic ai-api-keys \
    --namespace="${NAMESPACE}" \
    --from-literal=gemini-api-key="${GEMINI_API_KEY}" \
    --from-literal=openai-api-key="${OPENAI_API_KEY}" \
    --dry-run=client -o yaml | kubectl apply -f -

# Deploy the application
echo "🚀 Deploying application..."
kubectl apply -f "${SCRIPT_DIR}/deployment.yaml"

# Wait for deployment to be ready
echo "⏳ Waiting for deployment to be ready..."
kubectl rollout status deployment/humanoid-robotics -n "${NAMESPACE}" --timeout=300s

# Show status
echo "✅ Deployment complete!"
echo ""
echo "📊 Current status:"
kubectl get all -n "${NAMESPACE}"
echo ""
echo "🌐 Access the application via Ingress at: humanoid-robotics.local"
echo "   Or use port-forward:"
echo "   kubectl port-forward service/humanoid-robotics-service 8000:8000 -n ${NAMESPACE}"
