#!/bin/bash

# Deployment script for Physical AI Edge Kit
# Separates frontend and backend services with Docker

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚀 Physical AI Edge Kit Deployment Script${NC}"
echo "=========================================="

# Check if docker is installed
if ! command -v docker &> /dev/null; then
    echo -e "${RED}❌ Docker is not installed. Please install Docker first.${NC}"
    exit 1
fi

# Check if docker compose is installed
if ! command -v docker compose &> /dev/null; then
    echo -e "${RED}❌ Docker Compose is not installed. Please install Docker Compose first.${NC}"
    exit 1
fi

# Function to display usage
usage() {
    echo "Usage: $0 [COMMAND]"
    echo "Commands:"
    echo "  up          - Start all services (production)"
    echo "  dev         - Start all services (development)"
    echo "  down        - Stop all services"
    echo "  logs        - Show logs from all services"
    echo "  build       - Build all services"
    echo "  rebuild     - Rebuild all services"
    echo "  status      - Show status of all services"
    echo "  help        - Show this help message"
    echo ""
    echo "Environment variables (set in .env file):"
    echo "  GEMINI_API_KEY, OPENAI_API_KEY, COHERE_API_KEY - API keys for AI services"
    echo "  POSTGRES_USER, POSTGRES_PASSWORD, POSTGRES_DB - Database configuration"
}

# Main command execution
case "${1:-help}" in
    "up")
        echo -e "${GREEN}🚀 Starting production services...${NC}"
        echo -e "${YELLOW}📝 Using environment variables from .env file${NC}"

        # Check if .env file exists
        if [ ! -f .env ]; then
            echo -e "${YELLOW}⚠️  Warning: .env file not found. Creating a sample .env file...${NC}"
            cp .env.example .env || echo -e "${YELLOW}⚠️  Could not copy .env.example. Please create .env manually.${NC}"
        fi

        docker compose up -d
        echo -e "${GREEN}✅ Production services started successfully!${NC}"
        echo -e "${BLUE}🌐 Frontend: http://localhost:3000${NC}"
        echo -e "${BLUE}🔌 Backend: http://localhost:8000${NC}"
        echo -e "${BLUE}🔍 Qdrant: http://localhost:6333${NC}"
        ;;

    "dev")
        echo -e "${GREEN}🚀 Starting development services...${NC}"
        echo -e "${YELLOW}📝 Using environment variables from .env file${NC}"

        # Check if .env file exists
        if [ ! -f .env ]; then
            echo -e "${YELLOW}⚠️  Warning: .env file not found. Creating a sample .env file...${NC}"
            cp .env.example .env || echo -e "${YELLOW}⚠️  Could not copy .env.example. Please create .env manually.${NC}"
        fi

        docker compose -f docker-compose.dev.yml up -d
        echo -e "${GREEN}✅ Development services started successfully!${NC}"
        echo -e "${BLUE}🌐 Frontend: http://localhost:3000${NC}"
        echo -e "${BLUE}🔌 Backend: http://localhost:8000${NC}"
        echo -e "${BLUE}🔍 Qdrant: http://localhost:6333${NC}"
        ;;

    "down")
        echo -e "${RED}🛑 Stopping all services...${NC}"
        docker compose -f docker-compose.dev.yml down || true
        docker compose down
        echo -e "${GREEN}✅ All services stopped.${NC}"
        ;;

    "logs")
        echo -e "${BLUE}📋 Showing logs from all services...${NC}"
        docker compose logs -f
        ;;

    "build")
        echo -e "${GREEN}🔨 Building all services...${NC}"
        docker compose build
        echo -e "${GREEN}✅ All services built successfully!${NC}"
        ;;

    "rebuild")
        echo -e "${GREEN}🔄 Rebuilding all services...${NC}"
        docker compose build --no-cache
        echo -e "${GREEN}✅ All services rebuilt successfully!${NC}"
        ;;

    "status")
        echo -e "${BLUE}📊 Current status of services:${NC}"
        docker compose ps
        ;;

    "help"|*)
        usage
        ;;
esac

echo -e "${NC}"
