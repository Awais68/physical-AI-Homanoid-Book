# Hugging Face Spaces Deployment Guide

## Overview
Deploy your Physical AI Humanoid Book RAG backend and chatbot to Hugging Face Spaces using the provided Docker configuration. This setup includes a self-contained environment with FastAPI backend and Qdrant vector database.

## Required Files for Your Hugging Face Space

Place these files in your Hugging Face Space repository root:

### 1. Core Configuration Files
```
Dockerfile.hf              # Docker configuration for Hugging Face
README-HF-SPACE.md         # Space metadata and documentation
start-hf.sh                # Startup script for the application
```

### 2. Application Directories
```
backend/                   # Main backend application with FastAPI
frontend/docs/             # Educational content for RAG
```

### 3. Backend Dependencies
```
backend/requirements.txt   # Python dependencies
backend/src/main.py        # Main FastAPI application
backend/check_and_ingest.py # Document ingestion script
backend/src/api/           # API route definitions
backend/src/clients/       # LLM and vector DB clients
backend/src/config/        # Configuration files
```

## Hugging Face Space Setup Instructions

### Step 1: Create Your Hugging Face Space
1. Go to https://huggingface.co/spaces
2. Click "Create New Space"
3. Choose the following settings:
   - **SDK**: Docker
   - **Hardware**: CPU (or GPU if needed for your use case)
   - **Visibility**: Public or Private (as per your preference)

### Step 2: Add Files to Your Space
Upload the following files to your Space repository:

1. **Clone your repository** (or use the Files tab in the Space UI):
```bash
git clone https://huggingface.co/spaces/YOUR_USERNAME/YOUR_SPACE_NAME
cd YOUR_SPACE_NAME
```

2. **Copy the required files**:
```bash
# Copy configuration files
cp /path/to/your/project/Dockerfile.hf .
cp /path/to/your/project/README-HF-SPACE.md .
cp /path/to/your/project/start-hf.sh .

# Copy backend directory
cp -r /path/to/your/project/backend .

# Create necessary directory structure
mkdir -p frontend/docs
# Copy your educational documents to frontend/docs/
```

### Step 3: Configure Environment Variables
In your Hugging Face Space settings:
1. Go to your Space page
2. Click on "Settings" → "Secrets"
3. Add the following environment variables:
   - `GEMINI_API_KEY`: Your Google Gemini API key
   - `COHERE_API_KEY`: Your Cohere API key (or other embedding provider)
   - `OPENAI_API_KEY`: Your OpenAI API key (optional)

### Step 4: Understanding the Architecture

The Dockerfile.hf sets up:
- Python 3.11 runtime
- Qdrant vector database (installed directly in the container)
- All Python dependencies from backend/requirements.txt
- Application listening on port 7860
- Automatic document ingestion on first startup

The start-hf.sh script:
1. Starts Qdrant vector database in the background
2. Waits for Qdrant to be ready
3. Runs document ingestion if needed
4. Starts the FastAPI backend on port 7860

### Step 5: Document Ingestion
Your educational content should be placed in:
- `frontend/docs/` directory
- Supported formats: Markdown, PDF, HTML, text files
- The `check_and_ingest.py` script will automatically process these documents on first startup

## API Endpoints Available

Once deployed, your chatbot will be accessible via:

### Health Check
```
GET /health
```

### Chat Endpoint
```
POST /api/chat/message
Content-Type: application/json

{
  "message": "Your question here",
  "conversationHistory": []
}
```

### API Documentation
```
GET /docs
```
Interactive Swagger UI for all available endpoints.

## Testing Locally Before Deployment

Before pushing to Hugging Face, test locally:

```bash
# Build the Docker image
docker build -f Dockerfile.hf -t hf-physical-ai .

# Run the container
docker run -p 7860:7860 -e GEMINI_API_KEY=your_key_here hf-physical-ai
```

Then visit `http://localhost:7860/docs` to access the API documentation.

## Troubleshooting

### Common Issues:
1. **Build failures**: Check the build logs for missing dependencies
2. **Port issues**: Ensure your application listens on port 7860 (as specified in Dockerfile)
3. **Qdrant startup**: The startup script waits for Qdrant, but you may need to adjust the sleep duration
4. **Document ingestion**: Large document sets may take time on first startup

### Monitoring:
- Check Space logs in the Hugging Face UI
- Verify environment variables are correctly set
- Test the `/health` endpoint to check service status

## Resource Optimization

- The setup includes Qdrant running inside the container, which uses additional resources
- Consider the number of documents you're ingesting based on Space resource limits
- The 134+ documents mentioned in the README may require a GPU Space depending on model size

## Updating Your Space

After initial deployment:
1. Make changes to your local files
2. Commit and push to your Hugging Face repository
3. The Space will automatically rebuild and redeploy

```bash
git add .
git commit -m "Update application"
git push
```

Your Physical AI Humanoid Book RAG backend and chatbot will be available at:
`https://YOUR_USERNAME.hf.space/YOUR_SPACE_NAME`