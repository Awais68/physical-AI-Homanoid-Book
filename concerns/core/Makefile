.PHONY: help build up down logs test clean deploy-hf deploy-prod

# Variables
DOCKER_COMPOSE_DEV = docker-compose -f docker-compose.dev.yml
DOCKER_COMPOSE_PROD = docker-compose -f docker-compose.prod.yml
BACKEND_IMAGE = physical-ai-backend:latest
FRONTEND_IMAGE = physical-ai-frontend:latest

help: ## Show this help message
	@echo "Physical AI Docker Commands"
	@echo ""
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

# Development Commands
dev-up: ## Start development environment
	@echo "Starting development environment..."
	$(DOCKER_COMPOSE_DEV) up --build

dev-down: ## Stop development environment
	@echo "Stopping development environment..."
	$(DOCKER_COMPOSE_DEV) down

dev-logs: ## View development logs
	$(DOCKER_COMPOSE_DEV) logs -f

dev-logs-backend: ## View backend logs only
	$(DOCKER_COMPOSE_DEV) logs -f backend

dev-logs-frontend: ## View frontend logs only
	$(DOCKER_COMPOSE_DEV) logs -f frontend

dev-test: ## Run backend tests
	$(DOCKER_COMPOSE_DEV) exec backend pytest

dev-shell-backend: ## Access backend shell
	$(DOCKER_COMPOSE_DEV) exec backend bash

dev-shell-db: ## Access database shell
	$(DOCKER_COMPOSE_DEV) exec postgres psql -U postgres -d edgekit_db

# Production Commands
prod-build: ## Build production images
	@echo "Building production images..."
	docker build -t $(BACKEND_IMAGE) ./backend
	docker build -t $(FRONTEND_IMAGE) ./frontend
	@echo "✅ Images built successfully"

prod-up: ## Start production environment
	@echo "Starting production environment..."
	$(DOCKER_COMPOSE_PROD) up -d

prod-down: ## Stop production environment
	@echo "Stopping production environment..."
	$(DOCKER_COMPOSE_PROD) down

prod-logs: ## View production logs
	$(DOCKER_COMPOSE_PROD) logs -f

prod-restart: ## Restart production services
	$(DOCKER_COMPOSE_PROD) restart

# Health & Diagnostics
health: ## Check backend health
	@echo "Checking backend health..."
	@curl -s http://localhost:8000/health | jq . || echo "Backend not responding"

diagnostics: ## Get system diagnostics
	@echo "Fetching diagnostics..."
	@curl -s http://localhost:8000/diagnostics | jq . || echo "Diagnostics endpoint not available"

status: ## Show container status
	@echo "=== Development Containers ==="
	@$(DOCKER_COMPOSE_DEV) ps 2>/dev/null || echo "Not running"
	@echo ""
	@echo "=== Production Containers ==="
	@$(DOCKER_COMPOSE_PROD) ps 2>/dev/null || echo "Not running"

# Database Commands
db-backup: ## Backup database to backup.sql
	@echo "Backing up database..."
	$(DOCKER_COMPOSE_DEV) exec postgres pg_dump -U postgres edgekit_db > backup.sql
	@echo "✅ Backup saved to backup.sql"

db-restore: ## Restore database from backup.sql
	@echo "Restoring database..."
	$(DOCKER_COMPOSE_DEV) exec -T postgres psql -U postgres edgekit_db < backup.sql
	@echo "✅ Database restored"

db-reset: ## Reset database (deletes all data)
	@echo "Resetting database..."
	$(DOCKER_COMPOSE_DEV) down -v
	$(DOCKER_COMPOSE_DEV) up -d postgres
	@echo "✅ Database reset"

# Cleaning Commands
clean: ## Remove stopped containers and dangling images
	@echo "Cleaning up Docker resources..."
	docker container prune -f
	docker image prune -f
	@echo "✅ Cleanup complete"

clean-all: ## Remove all containers, images, volumes (DANGEROUS!)
	@echo "⚠️  WARNING: This will delete all containers, images, and volumes!"
	@read -p "Are you sure? (y/N) " -n 1 -r; \
	echo; \
	if [[ $$REPLY =~ ^[Yy]$$ ]]; then \
		$(DOCKER_COMPOSE_DEV) down -v; \
		$(DOCKER_COMPOSE_PROD) down -v; \
		docker system prune -af --volumes; \
		echo "✅ Full cleanup complete"; \
	fi

# Deployment Commands
deploy-hf: ## Deploy to Hugging Face Spaces
	@echo "Deploying to Hugging Face Spaces..."
	@if [ -z "$$HF_TOKEN" ]; then \
		echo "❌ Error: HF_TOKEN environment variable not set"; \
		exit 1; \
	fi
	@if [ -z "$$HF_SPACE_ID" ]; then \
		echo "❌ Error: HF_SPACE_ID environment variable not set"; \
		exit 1; \
	fi
	python scripts/deploy_hf.py
	@echo "✅ Deployment to HF Spaces complete"

deploy-prod: ## Deploy production to docker-compose
	@echo "Deploying production setup..."
	$(DOCKER_COMPOSE_PROD) up -d
	@echo "✅ Production deployment complete"
	@echo "Frontend: http://localhost"
	@echo "Backend API: http://localhost:8000"

# Build Commands
build-dev: ## Build development images
	$(DOCKER_COMPOSE_DEV) build

build-prod: ## Build production images
	docker build -t $(BACKEND_IMAGE) ./backend
	docker build -t $(FRONTEND_IMAGE) ./frontend

build-all: build-dev build-prod ## Build all images
	@echo "✅ All images built"

# Registry Commands
push-backend: ## Push backend image to registry
	@echo "Pushing backend image..."
	docker tag $(BACKEND_IMAGE) ghcr.io/yourusername/$(BACKEND_IMAGE)
	docker push ghcr.io/yourusername/$(BACKEND_IMAGE)
	@echo "✅ Backend image pushed"

push-frontend: ## Push frontend image to registry
	@echo "Pushing frontend image..."
	docker tag $(FRONTEND_IMAGE) ghcr.io/yourusername/$(FRONTEND_IMAGE)
	docker push ghcr.io/yourusername/$(FRONTEND_IMAGE)
	@echo "✅ Frontend image pushed"

# Testing Commands
test-backend: ## Run backend tests
	$(DOCKER_COMPOSE_DEV) exec backend pytest -v

test-health: ## Test all health endpoints
	@echo "Testing health endpoints..."
	@echo "Backend: $$(curl -s http://localhost:8000/health | jq '.status')"
	@echo "PostgreSQL: $$($(DOCKER_COMPOSE_DEV) exec postgres pg_isready -U postgres)"
	@echo "Qdrant: $$(curl -s http://localhost:6333/readyz)"

# Info Commands
info: ## Show project information
	@echo "=== Physical AI Docker Project ==="
	@echo "Backend Image: $(BACKEND_IMAGE)"
	@echo "Frontend Image: $(FRONTEND_IMAGE)"
	@echo ""
	@echo "Development Services:"
	@echo "  - Frontend (Docusaurus): http://localhost:3000"
	@echo "  - Backend (FastAPI): http://localhost:8000"
	@echo "  - API Docs: http://localhost:8000/docs"
	@echo "  - Qdrant: http://localhost:6333/dashboard"
	@echo "  - PostgreSQL: localhost:5432"
	@echo ""
	@echo "Production Services:"
	@echo "  - Frontend (Nginx): http://localhost"
	@echo "  - Backend (FastAPI): http://localhost:8000/api"
	@echo ""
	@echo "Run 'make help' for all available commands"

# Default target
.DEFAULT_GOAL := help

# Phony targets (not file-based)
.PHONY: help dev-up dev-down dev-logs dev-shell-backend dev-shell-db \
        prod-build prod-up prod-down prod-logs prod-restart \
        health diagnostics status \
        db-backup db-restore db-reset \
        clean clean-all \
        deploy-hf deploy-prod \
        build-dev build-prod build-all \
        push-backend push-frontend \
        test-backend test-health \
        info
