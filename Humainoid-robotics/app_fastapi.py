"""
Unified FastAPI app for Hugging Face Spaces deployment.
Serves both API and optional static files on port 7860.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from src.api.routers import chat

app = FastAPI(
    title="Humainoid Robotics RAG API",
    description="RAG-powered chatbot API for Physical AI & Humanoid Robotics",
    version="1.0.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include chat router with API prefix
app.include_router(chat.router, prefix="/api", tags=["chat"])


@app.get("/")
def read_root():
    """Root endpoint with API information."""
    return HTMLResponse(content="""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Humainoid Robotics RAG API</title>
        <style>
            body {
                font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
                max-width: 800px;
                margin: 50px auto;
                padding: 20px;
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                color: white;
            }
            .container {
                background: rgba(255, 255, 255, 0.95);
                color: #333;
                padding: 30px;
                border-radius: 15px;
                box-shadow: 0 10px 40px rgba(0, 0, 0, 0.3);
            }
            h1 { color: #667eea; margin-top: 0; }
            h2 { color: #764ba2; border-bottom: 2px solid #667eea; padding-bottom: 10px; }
            code {
                background: #f5f5f5;
                padding: 2px 6px;
                border-radius: 3px;
                font-family: 'Courier New', monospace;
            }
            pre {
                background: #2d2d2d;
                color: #f8f8f2;
                padding: 15px;
                border-radius: 5px;
                overflow-x: auto;
            }
            .endpoint {
                background: #e8f5e9;
                padding: 10px;
                margin: 10px 0;
                border-left: 4px solid #4caf50;
                border-radius: 3px;
            }
            a {
                color: #667eea;
                text-decoration: none;
                font-weight: bold;
            }
            a:hover {
                text-decoration: underline;
            }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🤖 Humainoid Robotics RAG API</h1>
            <p>Welcome to the Physical AI & Humanoid Robotics RAG-powered chatbot API.</p>
            
            <h2>📚 API Documentation</h2>
            <ul>
                <li><a href="/docs">Interactive API Documentation (Swagger UI)</a></li>
                <li><a href="/redoc">ReDoc Documentation</a></li>
                <li><a href="/health">Health Check</a></li>
            </ul>
            
            <h2>🔗 Available Endpoints</h2>
            
            <div class="endpoint">
                <strong>POST</strong> <code>/api/chat/message</code>
                <p>Send a chat message and get a RAG-powered response</p>
            </div>
            
            <div class="endpoint">
                <strong>POST</strong> <code>/api/chat/selected-text</code>
                <p>Query about selected text content</p>
            </div>
            
            <div class="endpoint">
                <strong>POST</strong> <code>/api/chat/index</code>
                <p>Index a new document for RAG retrieval</p>
            </div>
            
            <h2>💡 Example Usage</h2>
            <pre><code>curl -X POST https://awais68-humainoid-robotics.hf.space/api/chat/message \\
  -H "Content-Type: application/json" \\
  -d '{
    "message": "What is Physical AI?",
    "conversation_history": [],
    "session_id": null
  }'</code></pre>
            
            <h2>🚀 Status</h2>
            <p>API is <span style="color: #4caf50; font-weight: bold;">RUNNING</span></p>
            <p>Powered by <strong>Gemini AI</strong>, <strong>Qdrant</strong>, and <strong>FastAPI</strong></p>
        </div>
    </body>
    </html>
    """)


@app.get("/health")
def health_check():
    """Health check endpoint."""
    return {"status": "healthy", "service": "rag-backend", "version": "1.0.0"}


@app.get("/api/health")
def api_health():
    """API health check endpoint."""
    return {"status": "healthy", "service": "rag-backend", "api": "v1"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=7860)
