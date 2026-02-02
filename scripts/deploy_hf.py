#!/usr/bin/env python3
"""Deploy to Hugging Face Spaces"""

import os
import subprocess
import sys
from pathlib import Path
from typing import Optional

def run_command(cmd: list[str], cwd: Optional[str] = None) -> int:
    """Run a shell command."""
    result = subprocess.run(cmd, cwd=cwd)
    return result.returncode

def get_hf_credentials() -> tuple[str, str]:
    """Get Hugging Face credentials from environment."""
    token = os.getenv("HF_TOKEN")
    space_id = os.getenv("HF_SPACE_ID")
    
    if not token or not space_id:
        raise ValueError("HF_TOKEN and HF_SPACE_ID environment variables are required")
    
    return token, space_id

def clone_hf_space(token: str, space_id: str, dest: str = "hf_space") -> str:
    """Clone Hugging Face space repository."""
    repo_url = f"https://{token}@huggingface.co/spaces/{space_id}"
    
    print(f"Cloning Hugging Face space: {space_id}")
    if run_command(["git", "clone", repo_url, dest]) != 0:
        raise RuntimeError(f"Failed to clone Hugging Face space")
    
    return dest

def prepare_space_files(space_dir: str) -> None:
    """Prepare files for Hugging Face space."""
    space_path = Path(space_dir)
    root_path = Path(__file__).parent.parent
    
    print("Preparing files for Hugging Face Space...")
    
    # Copy docker files
    files_to_copy = [
        ("backend/Dockerfile", "Dockerfile.backend"),
        ("frontend/Dockerfile", "Dockerfile.frontend"),
        ("frontend/nginx.conf", "nginx.conf"),
        ("docker-compose.prod.yml", "docker-compose.yml"),
        (".env.example", ".env.example"),
    ]
    
    for src, dst in files_to_copy:
        src_path = root_path / src
        dst_path = space_path / dst
        
        if src_path.exists():
            print(f"  Copying {src} → {dst}")
            import shutil
            shutil.copy2(src_path, dst_path)
        else:
            print(f"  Warning: {src} not found")
    
    # Create Streamlit app for better HF integration
    create_streamlit_app(space_path)

def create_streamlit_app(space_dir: str) -> None:
    """Create a Streamlit app for better Hugging Face integration."""
    streamlit_code = '''"""Physical AI Humanoid Book - Hugging Face Spaces Integration"""

import os
import streamlit as st
import requests
from datetime import datetime
import json

# Set page config
st.set_page_config(
    page_title="Physical AI - RAG Chatbot",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Custom CSS
st.markdown("""
<style>
    .main {
        padding: 2rem;
    }
    .stTabs [role="tab"] {
        font-size: 1.1rem;
    }
</style>
""", unsafe_allow_html=True)

# Initialize session state
if "backend_url" not in st.session_state:
    st.session_state.backend_url = os.getenv("BACKEND_URL", "http://backend:8000")

if "chat_history" not in st.session_state:
    st.session_state.chat_history = []

# Page title
st.title("🤖 Physical AI Humanoid Book")
st.markdown("### Interactive RAG Chatbot & Documentation")

# Sidebar
with st.sidebar:
    st.header("⚙️ Configuration")
    
    backend_url = st.text_input(
        "Backend URL",
        value=st.session_state.backend_url,
        help="URL of the backend API server"
    )
    st.session_state.backend_url = backend_url
    
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🔗 Test Connection", use_container_width=True):
            try:
                response = requests.get(
                    f"{backend_url}/health",
                    timeout=5
                )
                if response.status_code == 200:
                    st.success("✅ Backend connected!")
                else:
                    st.error(f"❌ Backend error: {response.status_code}")
            except Exception as e:
                st.error(f"❌ Connection failed: {str(e)}")
    
    with col2:
        if st.button("📊 Diagnostics", use_container_width=True):
            st.session_state.show_diagnostics = True
    
    st.divider()
    
    # Clear chat history
    if st.button("🗑️ Clear Chat History", use_container_width=True):
        st.session_state.chat_history = []
        st.success("Chat history cleared!")

# Main content tabs
tab1, tab2, tab3 = st.tabs(["💬 Chat", "📚 Documentation", "📊 Diagnostics"])

with tab1:
    st.header("Chat with RAG Chatbot")
    
    # Display chat history
    for i, message in enumerate(st.session_state.chat_history):
        if message["role"] == "user":
            st.chat_message("user").write(message["content"])
        else:
            st.chat_message("assistant").write(message["content"])
    
    # Chat input
    col1, col2 = st.columns([0.9, 0.1])
    with col1:
        query = st.chat_input("Ask me anything about Physical AI...")
    
    if query:
        # Add user message
        st.session_state.chat_history.append({
            "role": "user",
            "content": query
        })
        st.chat_message("user").write(query)
        
        # Get response from backend
        try:
            with st.spinner("Thinking..."):
                response = requests.post(
                    f"{st.session_state.backend_url}/api/chat/query",
                    json={"query": query},
                    timeout=30
                )
            
            if response.status_code == 200:
                result = response.json()
                bot_response = result.get("response", "No response received")
                
                # Add bot message
                st.session_state.chat_history.append({
                    "role": "assistant",
                    "content": bot_response
                })
                st.chat_message("assistant").write(bot_response)
            else:
                error_msg = f"Error: {response.status_code}"
                st.error(error_msg)
        except requests.exceptions.Timeout:
            st.error("❌ Request timeout. Backend might be slow.")
        except Exception as e:
            st.error(f"❌ Failed to get response: {str(e)}")

with tab2:
    st.header("📚 Documentation")
    
    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("Getting Started")
        st.write("""
        - Learn about Physical AI concepts
        - Explore robot programming basics
        - Understand RAG (Retrieval-Augmented Generation)
        """)
    
    with col2:
        st.subheader("Resources")
        st.write("""
        - [GitHub Repository](https://github.com/yourusername/physical-ai)
        - [Documentation](https://docs.example.com)
        - [Community Forum](https://forum.example.com)
        """)
    
    st.divider()
    
    # Fetch docs from backend
    try:
        response = requests.get(
            f"{st.session_state.backend_url}/api/docs",
            timeout=10
        )
        if response.status_code == 200:
            docs = response.json()
            for doc in docs.get("documents", []):
                with st.expander(f"📄 {doc.get('title', 'Document')}"):
                    st.write(doc.get('content', 'No content'))
        else:
            st.info("Documentation service not available")
    except Exception as e:
        st.warning(f"Could not load documentation: {str(e)}")

with tab3:
    st.header("📊 System Diagnostics")
    
    col1, col2 = st.columns(2)
    
    with col1:
        if st.button("🔄 Refresh Diagnostics", use_container_width=True):
            st.rerun()
    
    with col2:
        if st.button("💾 Download Logs", use_container_width=True):
            st.info("Logs download feature coming soon")
    
    # Fetch diagnostics
    try:
        with st.spinner("Fetching diagnostics..."):
            response = requests.get(
                f"{st.session_state.backend_url}/diagnostics",
                timeout=10
            )
        
        if response.status_code == 200:
            diag = response.json()
            
            # Overall status
            st.subheader("Overall Status")
            status_color = "🟢" if diag.get("status") == "running" else "🔴"
            st.write(f"{status_color} Status: {diag.get('status', 'unknown')}")
            
            # Services status
            st.subheader("Services Status")
            services = diag.get("services", {})
            
            cols = st.columns(len(services))
            for idx, (service, status) in enumerate(services.items()):
                with cols[idx]:
                    connected = status.get("connected", False)
                    icon = "🟢" if connected else "🔴"
                    st.metric(
                        f"{service.title()}",
                        "Connected" if connected else "Disconnected"
                    )
            
            # Detailed services info
            st.subheader("Services Details")
            col1, col2 = st.columns(2)
            
            with col1:
                st.write("**Qdrant Vector DB:**")
                st.json(services.get("qdrant", {}))
                
                st.write("**Gemini API:**")
                st.json(services.get("gemini", {}))
            
            with col2:
                st.write("**OpenAI API:**")
                st.json(services.get("openai", {}))
                
                st.write("**Cohere API:**")
                st.json(services.get("cohere", {}))
        else:
            st.error(f"Failed to fetch diagnostics: {response.status_code}")
    except Exception as e:
        st.error(f"Could not fetch diagnostics: {str(e)}")

# Footer
st.divider()
st.markdown("""
<div style="text-align: center; color: gray; font-size: 0.85rem;">
    Physical AI Humanoid Book | Powered by FastAPI + Streamlit | 🤖
    <br>
    Last updated: {} | Status: Healthy
</div>
""".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")), unsafe_allow_html=True)
'''
    
    app_path = Path(space_dir) / "app.py"
    with open(app_path, "w") as f:
        f.write(streamlit_code)
    
    print(f"  Created Streamlit app: app.py")
    
    # Create requirements.txt for Streamlit
    reqs_path = Path(space_dir) / "requirements.txt"
    with open(reqs_path, "w") as f:
        f.write("""streamlit==1.28.1
requests==2.31.0
python-dotenv==1.2.1
""")
    
    print(f"  Created Streamlit requirements: requirements.txt")

def push_to_hf(space_dir: str) -> None:
    """Push changes to Hugging Face."""
    print("Pushing to Hugging Face Spaces...")
    
    space_path = Path(space_dir)
    
    # Git operations
    commands = [
        ["git", "add", "-A"],
        ["git", "commit", "-m", "Automated deployment update"],
        ["git", "push"],
    ]
    
    for cmd in commands:
        print(f"  Running: {' '.join(cmd)}")
        if run_command(cmd, cwd=str(space_path)) != 0:
            raise RuntimeError(f"Failed to run command: {' '.join(cmd)}")
    
    print("✅ Successfully pushed to Hugging Face Spaces!")

def main():
    """Main deployment function."""
    try:
        # Get credentials
        token, space_id = get_hf_credentials()
        
        # Clone space
        space_dir = clone_hf_space(token, space_id)
        
        # Prepare files
        prepare_space_files(space_dir)
        
        # Push to HF
        push_to_hf(space_dir)
        
        print("✅ Deployment complete!")
        print(f"View your space: https://huggingface.co/spaces/{space_id}")
        
    except Exception as e:
        print(f"❌ Deployment failed: {str(e)}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
'''