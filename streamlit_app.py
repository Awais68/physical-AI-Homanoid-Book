#!/usr/bin/env python3
"""
Streamlit app for Hugging Face Spaces deployment
Provides web interface for the Physical AI chatbot
"""

import streamlit as st
import requests
import json
from datetime import datetime

# Page configuration
st.set_page_config(
    page_title="Physical AI Humanoid Book",
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
    .stButton > button {
        width: 100%;
        padding: 0.75rem;
        background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
        color: white;
        border: none;
        border-radius: 8px;
        font-weight: bold;
    }
    .stButton > button:hover {
        background: linear-gradient(135deg, #764ba2 0%, #667eea 100%);
    }
    </style>
    """, unsafe_allow_html=True)

# Backend configuration
BACKEND_URL = "http://localhost:8000"  # Update for HF deployment
API_ENDPOINTS = {
    "health": f"{BACKEND_URL}/health",
    "chat": f"{BACKEND_URL}/api/chat/message",
    "devices": f"{BACKEND_URL}/api/devices",
    "languages": f"{BACKEND_URL}/api/i18n/languages"
}

# Session state initialization
if "messages" not in st.session_state:
    st.session_state.messages = []
if "session_id" not in st.session_state:
    st.session_state.session_id = None

# Header
st.title("🤖 Physical AI Humanoid Book")
st.markdown("**Interactive Chatbot with RAG (Retrieval-Augmented Generation)**")

# Sidebar
with st.sidebar:
    st.header("⚙️ Settings")
    
    language = st.selectbox(
        "Language",
        ["English", "Urdu", "Spanish", "French"],
        index=0
    )
    
    temperature = st.slider(
        "Response Temperature (Creativity)",
        min_value=0.0,
        max_value=1.0,
        value=0.7,
        step=0.1
    )
    
    # Status check
    st.markdown("---")
    st.subheader("📊 System Status")
    
    try:
        response = requests.get(API_ENDPOINTS["health"], timeout=2)
        if response.status_code == 200:
            st.success("✅ Backend: Online")
        else:
            st.error("❌ Backend: Error")
    except:
        st.error("❌ Backend: Offline")

# Main chat interface
col1, col2 = st.columns([3, 1])

with col1:
    st.subheader("💬 Chat")
    
    # Display chat history
    chat_container = st.container()
    with chat_container:
        for message in st.session_state.messages:
            if message["role"] == "user":
                st.chat_message("user").write(message["content"])
            else:
                st.chat_message("assistant").write(message["content"])

# Chat input
if prompt := st.chat_input("Ask me anything about Physical AI..."):
    # Add user message to history
    st.session_state.messages.append({
        "role": "user",
        "content": prompt,
        "timestamp": datetime.now().isoformat()
    })
    
    # Show user message
    st.chat_message("user").write(prompt)
    
    # Get response from backend
    try:
        response = requests.post(
            API_ENDPOINTS["chat"],
            json={
                "message": prompt,
                "language": "en" if language == "English" else "ur"
            },
            timeout=10
        )
        
        if response.status_code == 200:
            data = response.json()
            assistant_message = data.get("answer", "Unable to process your request")
            
            # Add to history
            st.session_state.messages.append({
                "role": "assistant",
                "content": assistant_message,
                "timestamp": datetime.now().isoformat(),
                "sources": data.get("sources", []),
                "confidence": data.get("confidence", 0)
            })
            
            # Display response
            with st.chat_message("assistant"):
                st.write(assistant_message)
                
                # Show sources if available
                if data.get("sources"):
                    with st.expander("📚 Sources"):
                        for source in data["sources"]:
                            st.write(f"- {source}")
        else:
            st.error(f"Error: {response.status_code}")
            
    except requests.exceptions.ConnectionError:
        st.error("❌ Cannot connect to backend. Ensure the backend is running.")
    except Exception as e:
        st.error(f"Error: {str(e)}")

# Footer
st.markdown("---")
col1, col2, col3 = st.columns(3)

with col1:
    if st.button("🗑️ Clear Chat"):
        st.session_state.messages = []
        st.rerun()

with col2:
    if st.button("💾 Export Chat"):
        chat_json = json.dumps(st.session_state.messages, indent=2)
        st.download_button(
            label="Download as JSON",
            data=chat_json,
            file_name=f"chat_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )

with col3:
    st.info("💡 Tip: Use specific questions for better results")
