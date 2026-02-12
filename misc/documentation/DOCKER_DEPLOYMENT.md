# Docker Deployment Guide

This guide explains how to deploy the Physical AI Edge Kit with separated frontend and backend services using Docker.

## Prerequisites

- Docker Engine (v20.10.0 or higher)
- Docker Compose (v2.0.0 or higher)
- At least 4GB of RAM available for Docker

## Quick Start

### 1. Clone the repository

```bash
git clone <repository-url>
cd physical-AI-Homanoid-Book-main
```

### 2. Configure environment variables

Copy the example environment file and update with your API keys:

```bash
cp .env.example .env
# Edit .env and add your API keys
```

Required API keys:
- `GEMINI_API_KEY`: Google Gemini API key
- `OPENAI_API_KEY`: OpenAI API key (optional)
- `COHERE_API_KEY`: Cohere API key (optional)

### 3. Deploy the services

#### Production deployment:
```bash
# Build and start all services
./deploy.sh up

# Check status
./deploy.sh status

# View logs
./deploy.sh logs
```

#### Development deployment:
```bash
# Start services in development mode
./deploy.sh dev
```

## Services Overview

The deployment includes 4 main services:

### 1. Frontend (Docusaurus)
- Port: 3000
- Built with React and Docusaurus
- Static site generation
- Handles user interface and documentation

### 2. Backend (FastAPI)
- Port: 8000
- Python-based REST API
- Handles AI/ML processing, chat, and device management
- Connected to Qdrant and PostgreSQL

### 3. Qdrant (Vector Database)
- Ports: 6333 (REST), 6334 (gRPC)
- Stores document embeddings
- Powers semantic search functionality

### 4. PostgreSQL (Relational Database)
- Port: 5432
- Stores user data, configurations, and application state
- Handles authentication and user management

## Docker Compose Files

- `docker-compose.yml`: Production configuration
- `docker-compose.dev.yml`: Development configuration with hot reloading

## Available Commands

Using the deployment script:

```bash
./deploy.sh [command]

# Commands:
#   up          - Start production services
#   dev         - Start development services
#   down        - Stop all services
#   logs        - View service logs
#   build       - Build services
#   rebuild     - Rebuild services from scratch
#   status      - Show service status
```

## Customization

### Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `GEMINI_API_KEY` | Google Gemini API key | required |
| `OPENAI_API_KEY` | OpenAI API key | optional |
| `COHERE_API_KEY` | Cohere API key | optional |
| `POSTGRES_USER` | PostgreSQL username | postgres |
| `POSTGRES_PASSWORD` | PostgreSQL password | postgres |
| `POSTGRES_DB` | PostgreSQL database name | edgekit_db |
| `BASE_URL` | Frontend base URL | http://localhost:3000 |
| `BACKEND_API_URL` | Backend API URL | http://localhost:8000 |

### Volumes

- `qdrant_data`: Persistent storage for vector database
- `postgres_data`: Persistent storage for relational database

## Health Checks

Each service has built-in health checks:
- Frontend: `/health` endpoint
- Backend: `/health` endpoint
- Qdrant: `/readyz` endpoint
- PostgreSQL: `pg_isready` command

## Troubleshooting

### Common Issues

1. **Port already in use**: Check if services are already running with `docker-compose ps`
2. **API keys missing**: Ensure all required API keys are in the `.env` file
3. **Database connection issues**: Verify PostgreSQL service is running and credentials are correct
4. **Insufficient memory**: Ensure Docker has at least 4GB RAM allocated

### Useful Commands

```bash
# View logs for specific service
docker-compose logs backend
docker-compose logs frontend

# Restart a specific service
docker-compose restart backend

# Clean up stopped containers
docker-compose down -v
```

## Architecture

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Frontend  │◄──►│   Backend   │◄──►│   Qdrant    │
│  (Port 3000)│    │  (Port 8000)│    │ (Ports 6333│
└─────────────┘    └─────────────┘    │     6334)   │
                                    └─────────────┘
                                           ▲
                                           │
                                    ┌─────────────┐
                                    │ PostgreSQL  │
                                    │ (Port 5432) │
                                    └─────────────┘
```

The architecture maintains clear separation of concerns with each service running in its own container.