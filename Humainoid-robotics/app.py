"""
Streamlit chatbot interface for Physical AI & Humanoid Robotics Q&A System.
Connects to FastAPI RAG backend running on localhost:8000.
Falls back to demo mode if backend is unavailable.
"""
import streamlit as st
import requests
import time

BACKEND_URL = "http://localhost:8000"

st.set_page_config(
    page_title="Humainoid Robotics Chatbot",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded",
)

st.markdown("""
<style>
    .main-header {
        text-align: center;
        padding: 1.5rem;
        background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
        color: white;
        border-radius: 12px;
        margin-bottom: 2rem;
        box-shadow: 0 4px 15px rgba(102, 126, 234, 0.4);
    }
    .chat-message {
        padding: 1rem;
        border-radius: 10px;
        margin-bottom: 1rem;
    }
    .source-badge {
        background-color: #e8f5e9;
        color: #2e7d32;
        padding: 0.2rem 0.5rem;
        border-radius: 5px;
        font-size: 0.8rem;
        margin-right: 0.5rem;
        display: inline-block;
    }
    .confidence-bar {
        height: 6px;
        border-radius: 3px;
        margin-top: 0.5rem;
    }
    .status-connected {
        color: #4caf50;
        font-weight: bold;
    }
    .status-demo {
        color: #ff9800;
        font-weight: bold;
    }
</style>
""", unsafe_allow_html=True)

# Demo knowledge base (fallback)
KNOWLEDGE_BASE = {
    "physical ai": """**Physical AI** refers to artificial intelligence systems that interact with the physical world through sensors, actuators, and robotic hardware.

Unlike purely software-based AI, Physical AI:
- Perceives the environment through cameras, lidar, touch sensors
- Takes physical actions through motors, grippers, and actuators
- Learns from real-world interactions and feedback
- Combines perception, planning, and control

Key applications include autonomous vehicles, industrial robots, humanoid robots, and smart prosthetics.""",

    "humanoid robot": """**Humanoid robots** are robots designed to resemble the human body structure.

Key characteristics:
- Two arms, two legs, a torso, and a head
- Bipedal locomotion (walking on two legs)
- Human-like hands for manipulation
- Facial features for communication

Famous examples:
- **Atlas** (Boston Dynamics) - advanced mobility
- **Sophia** (Hanson Robotics) - human-like face
- **Pepper** (SoftBank) - social humanoid
- **ASIMO** (Honda) - early pioneer
- **Figure 01** - general purpose humanoid
- **Tesla Optimus** - manufacturing humanoid""",

    "rag": """**RAG (Retrieval-Augmented Generation)** improves AI responses:

1. **Retrieval**: Search a knowledge base for relevant documents
2. **Augmentation**: Add retrieved context to the prompt
3. **Generation**: Use an LLM to generate a response

Benefits:
- Grounded answers from source documents
- Reduced hallucination
- Can use private knowledge
- Updatable without retraining

This chatbot uses RAG with Qdrant vector database and Gemini AI.""",

    "education": """**AI in Education** transforms learning through:

- Personalized tutoring systems
- Automated grading and feedback
- Intelligent content recommendation
- Accessibility tools
- Natural language interfaces

For K-12: AI tutors provide individualized practice and identify learning gaps.
For higher education: AI assists with research, code debugging, and problem solving.
For robotics education: Hands-on STEM learning with physical AI platforms.""",

    "ros": """**ROS (Robot Operating System)** is a framework for robot software:

- Tools, libraries, and conventions
- Simplifies complex robot behavior
- Modular and reusable

Key concepts:
- **Nodes**: Individual processes (sensors, motors, AI)
- **Topics**: Message buses for communication
- **Services**: Request/response communication
- **Actions**: Long-running tasks with feedback

ROS 2 offers real-time support, security, and better multi-robot coordination.""",

    "sensor": """**Robotics Sensors** provide perception capabilities:

- **Cameras**: Visual input (RGB, depth, stereo)
- **LiDAR**: 3D mapping and distance measurement
- **IMU**: Orientation and acceleration
- **Force/Torque**: Contact and manipulation sensing
- **Microphones**: Audio input for voice commands
- **Ultrasonic**: Short-range distance measurement

Modern robots combine multiple sensors (sensor fusion) for robust perception.""",

    "edge computing": """**Edge Computing in Robotics** brings AI processing directly to the robot:

Benefits:
- Low latency real-time decisions
- Works without internet connection
- Privacy - data stays on device
- Reduced bandwidth usage

Hardware: NVIDIA Jetson, Google Coral, Intel Neural Compute Stick
Use cases: Object detection, navigation, voice processing, safety systems""",
}


def check_backend_health():
    """Check if the FastAPI backend is available."""
    try:
        resp = requests.get(f"{BACKEND_URL}/health", timeout=3)
        return resp.status_code == 200
    except Exception:
        return False


def query_backend(message, conversation_history=None):
    """Send a query to the FastAPI RAG backend."""
    try:
        payload = {
            "message": message,
            "conversation_history": conversation_history or [],
        }
        resp = requests.post(
            f"{BACKEND_URL}/api/chat/message",
            json=payload,
            timeout=30,
        )
        if resp.status_code == 200:
            return resp.json()
        return None
    except Exception as e:
        print(f"Backend query error: {e}")
        return None


def get_demo_response(message):
    """Get a response from the local knowledge base (demo mode)."""
    message_lower = message.lower()
    for key, response in KNOWLEDGE_BASE.items():
        if key in message_lower:
            return response, [f"Knowledge: {key}"]

    topics = ", ".join(KNOWLEDGE_BASE.keys())
    return (
        f"I can answer questions about: **Physical AI**, **Humanoid Robots**, "
        f"**RAG**, **AI in Education**, **ROS**, **Sensors**, and **Edge Computing**.\n\n"
        f'Try asking: "What is Physical AI?" or "Tell me about humanoid robots"',
        ["Demo mode"],
    )


def main():
    """Main Streamlit app."""
    st.markdown("""
    <div class="main-header">
        <h1>🤖 Humainoid Robotics Chatbot</h1>
        <p>AI assistant for Physical AI & Humanoid Robotics education — powered by RAG</p>
    </div>
    """, unsafe_allow_html=True)

    # Check backend status
    if "backend_available" not in st.session_state:
        st.session_state.backend_available = check_backend_health()
        st.session_state.last_health_check = time.time()

    # Re-check backend every 30 seconds
    if time.time() - st.session_state.get("last_health_check", 0) > 30:
        st.session_state.backend_available = check_backend_health()
        st.session_state.last_health_check = time.time()

    if st.session_state.backend_available:
        st.success("🟢 **RAG Backend Connected** — AI-powered responses with document retrieval")
    else:
        st.warning("🟡 **Demo Mode** — Backend starting up, using built-in knowledge base")

    # Sidebar
    with st.sidebar:
        st.title("📚 About")
        st.info("""
        This chatbot provides answers about:

        - 🤖 Physical AI systems
        - 🦾 Humanoid robotics
        - 📖 Educational technology
        - 🔧 Technical implementations
        - 🌐 ROS & Edge Computing
        """)

        st.title("🔗 Links")
        st.markdown("""
        - [GitHub](https://github.com/Awais68/physical-AI-Homanoid-Book)
        - [Docs](https://awais68.github.io/physical-AI-Homanoid-Book/)
        - [HF Space](https://huggingface.co/spaces/Awais68/Humainoid-robotics)
        """)

        st.divider()
        mode = "RAG Backend" if st.session_state.backend_available else "Demo Mode"
        st.caption(f"Mode: **{mode}**")

        if st.button("🔄 Check Backend"):
            st.session_state.backend_available = check_backend_health()
            st.session_state.last_health_check = time.time()
            st.rerun()

        if st.button("🗑️ Clear Chat"):
            st.session_state.messages = []
            st.rerun()

    # Chat interface
    if "messages" not in st.session_state:
        st.session_state.messages = []

    # Welcome message
    if not st.session_state.messages:
        welcome = (
            "👋 Hello! I'm the **Humainoid Robotics Chatbot**. "
            "Ask me about Physical AI, humanoid robots, RAG, AI in education, ROS, or edge computing!\n\n"
            "I'm powered by a RAG system with 134+ documents from the Physical AI & Humanoid Robotics book."
        )
        st.session_state.messages.append({"role": "assistant", "content": welcome})

    # Display messages
    for message in st.session_state.messages:
        with st.chat_message(message["role"]):
            st.markdown(message["content"])
            # Show sources if available
            if message.get("sources"):
                with st.expander("📚 Sources", expanded=False):
                    for src in message["sources"]:
                        if isinstance(src, dict):
                            title = src.get("title", "Document")
                            score = src.get("relevance_score", 0)
                            url = src.get("source_url", "")
                            st.markdown(f"- **{title}** (relevance: {score:.2f})")
                            if url:
                                st.markdown(f"  [{url}]({url})")
                        else:
                            st.markdown(f"- {src}")

    # Chat input
    if prompt := st.chat_input("Ask about Physical AI, robots, or AI education..."):
        st.session_state.messages.append({"role": "user", "content": prompt})
        with st.chat_message("user"):
            st.markdown(prompt)

        with st.chat_message("assistant"):
            with st.spinner("Thinking..."):
                sources = []
                if st.session_state.backend_available:
                    # Build conversation history for context
                    history = [
                        {"role": m["role"], "content": m["content"]}
                        for m in st.session_state.messages[:-1]  # exclude current
                    ]
                    result = query_backend(prompt, history)
                    if result:
                        answer = result.get("answer", "")
                        sources = result.get("sources", [])
                        confidence = result.get("confidence", 0)

                        st.markdown(answer)

                        if sources:
                            with st.expander("📚 Sources", expanded=False):
                                for src in sources:
                                    if isinstance(src, dict):
                                        title = src.get("title", "Document")
                                        score = src.get("relevance_score", 0)
                                        url = src.get("source_url", "")
                                        st.markdown(f"- **{title}** (relevance: {score:.2f})")
                                        if url:
                                            st.markdown(f"  [{url}]({url})")

                        if confidence > 0:
                            color = "#4caf50" if confidence > 0.7 else "#ff9800" if confidence > 0.4 else "#f44336"
                            st.markdown(
                                f'<div class="confidence-bar" style="background: linear-gradient(90deg, {color} {confidence*100}%, #eee {confidence*100}%);"></div>',
                                unsafe_allow_html=True,
                            )
                            st.caption(f"Confidence: {confidence:.1%}")

                        st.session_state.messages.append({
                            "role": "assistant",
                            "content": answer,
                            "sources": sources,
                        })
                    else:
                        # Backend returned error, fall back to demo
                        st.session_state.backend_available = False
                        answer, demo_sources = get_demo_response(prompt)
                        st.markdown(answer)
                        st.session_state.messages.append({
                            "role": "assistant",
                            "content": answer,
                            "sources": demo_sources,
                        })
                else:
                    answer, demo_sources = get_demo_response(prompt)
                    st.markdown(answer)
                    st.session_state.messages.append({
                        "role": "assistant",
                        "content": answer,
                        "sources": demo_sources,
                    })

        st.rerun()


if __name__ == "__main__":
    main()
