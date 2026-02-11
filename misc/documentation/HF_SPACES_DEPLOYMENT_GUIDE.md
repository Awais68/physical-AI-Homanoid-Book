# Hugging Face Spaces Deployment Guide

## Overview
This guide explains how to deploy your backend and chatbot application to Hugging Face Spaces using the Docker-based setup provided in this project.

## Required Files for Hugging Face Space

When creating a Hugging Face Space, you need to place these essential files in the Space repository:

### 1. Core Application Files
```
app.py (main application entry point)
backend/ (your backend directory)
frontend/ (your frontend directory - if applicable)
```

### 2. Hugging Face Specific Files
```
Dockerfile.hf - Custom Dockerfile for Hugging Face deployment
README-HF-SPACE.md - Space-specific README with metadata
start-hf.sh - Startup script for the Hugging Face environment
.env.hf - Hugging Face environment variables
```

### 3. Configuration Files
```
docker-compose.hf.yml - Docker Compose configuration for Hugging Face
```

## Step-by-Step Deployment Process

### Option 1: Direct Repository Approach
1. Create a new Hugging Face Space or fork this repository to your Hugging Face account
2. Copy the following files to your Space repository:
   - `app.py` (or your main application file)
   - `backend/` directory
   - `frontend/` directory (if you have a frontend)
   - `Dockerfile.hf`
   - `start-hf.sh`
   - `README-HF-SPACE.md`
   - `.env.hf` (with your environment variables)

### Option 2: Using the Provided Scripts
1. Clone this repository to your local machine
2. Update the `.env.hf` file with your specific configuration
3. Use the deployment script: `./deploy-to-hf.sh`

## Hugging Face Space Configuration

### Runtime Environment
The application is configured to run in Hugging Face's Docker environment with:
- Python 3.11+
- FastAPI backend
- Support for Qdrant vector database (runs inside the container)
- GPU support (if using a GPU Space)

### Environment Variables
Update `.env.hf` with your specific settings:
```
HF_TOKEN=your_huggingface_token
HF_SPACE_ID=your_username/your_space_name
QDRANT_URL=http://localhost:6333  # Qdrant runs locally in the container
OPENAI_API_KEY=your_openai_key  # Or other LLM provider keys
```

## Dockerfile.hf Contents
The `Dockerfile.hf` is specifically configured for Hugging Face Spaces with:
- All necessary dependencies installed
- Qdrant vector database running inside the container
- Proper port exposure (typically port 7860 or 8000)
- Health checks for the Hugging Face infrastructure

## README-HF-SPACE.md
This file contains the Space metadata including:
- Library tag (likely `docker` since we're using custom Dockerfile)
- Title and description of your application
- License information
- Tags for discoverability

## Start Script (start-hf.sh)
The startup script handles:
- Environment setup
- Database initialization
- Starting the main application server
- Health check endpoints

## Backend and Chatbot Components

### Backend Structure
Your backend in the `backend/` directory should include:
- FastAPI application
- API routes for chatbot functionality
- Integration with vector databases (Qdrant)
- LLM integration (OpenAI, etc.)

### Chatbot Features
The chatbot functionality should include:
- Conversation history management
- Context retrieval from vector database
- Response generation using LLM
- Support for multiple languages (based on your i18n implementation)

## Testing Locally Before Deployment

Before deploying to Hugging Face Spaces, test locally:
```bash
# Build and run with Docker
docker build -f Dockerfile.hf -t my-chatbot-app .
docker run -p 8000:8000 my-chatbot-app
```

## Deployment Steps

1. Prepare your Hugging Face Space:
   - Go to https://huggingface.co/spaces
   - Click "Create New Space"
   - Choose "Docker" as the SDK
   - Choose your hardware requirements (CPU/GPU)

2. Add files to your Space:
   - Upload the required files mentioned above
   - Ensure your `Dockerfile.hf` is properly configured
   - Verify environment variables in `.env.hf`

3. Configure your Space settings:
   - Set environment variables in the Space settings
   - Configure secrets if needed
   - Set visibility (public/private)

4. Monitor the build:
   - Check the build logs in the Space interface
   - Wait for successful deployment
   - Test the application once deployed

## Troubleshooting

Common issues and solutions:
- **Build failures**: Check the Dockerfile and dependencies
- **Port issues**: Ensure your application listens on the correct port (usually 8000)
- **Environment variables**: Make sure all required variables are set
- **Resource limits**: Optimize your application to fit within Space resource limits

## Best Practices

- Keep the Docker image size reasonable for faster builds
- Implement proper error handling and logging
- Use efficient vector database queries
- Implement rate limiting to manage resources
- Add health check endpoints for better Space monitoring

## Scaling Considerations

- Hugging Face Spaces have resource limitations
- Consider implementing caching for frequently accessed data
- Optimize your model loading and inference times
- Plan for concurrent user sessions appropriately