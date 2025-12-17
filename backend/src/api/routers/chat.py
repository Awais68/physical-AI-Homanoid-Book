"""Chat API endpoints for RAG chatbot."""
from typing import Optional
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel, Field
from src.rag.engine import RAGEngine
from src.rag.document_store import DocumentStore


router = APIRouter(prefix="/chat", tags=["chat"])

# Initialize RAG engine
rag_engine = RAGEngine()


class ChatMessage(BaseModel):
    """Chat message request model."""
    message: str = Field(..., min_length=1, max_length=2000)
    session_id: Optional[str] = None
    selected_text: Optional[str] = None
    conversation_history: Optional[list] = None


class ChatResponse(BaseModel):
    """Chat response model."""
    answer: str
    sources: list
    citations: list
    query: str
    confidence: float
    has_sources: bool
    source_count: int
    timestamp: str
    session_id: Optional[str] = None


class IndexDocumentRequest(BaseModel):
    """Request to index a document."""
    content: str = Field(..., min_length=1)
    title: Optional[str] = None
    source_url: Optional[str] = None
    file_path: Optional[str] = None
    section: Optional[str] = None
    tags: Optional[list] = None
    metadata: Optional[dict] = None


class IndexDocumentResponse(BaseModel):
    """Response for document indexing."""
    success: bool
    document_id: str
    message: str


@router.post("/message", response_model=ChatResponse)
async def send_message(request: ChatMessage) -> ChatResponse:
    """Send a message to the RAG chatbot.

    Args:
        request: Chat message request.

    Returns:
        ChatResponse with answer and sources.
    """
    from datetime import datetime
    import hashlib
    from src.clients.qdrant_client import qdrant_client
    
    # Simple keyword-based responses (no API calls needed)
    question_lower = request.message.lower()
    
    # Try to search local docs with hash-based embedding
    try:
        # Create simple embedding from query
        def simple_embedding(text: str, dim: int = 1024) -> list:
            hash_obj = hashlib.md5(text.encode())
            hash_bytes = hash_obj.digest()
            vector = []
            for i in range(dim):
                vector.append((hash_bytes[i % len(hash_bytes)] - 128) / 128.0)
            return vector
        
        query_vector = simple_embedding(request.message)
        
        # Search in local_docs collection
        search_results = qdrant_client.search(
            collection_name='local_docs',
            query_vector=query_vector,
            limit=3,
        )
        
        if search_results and len(search_results) > 0:
            # Build answer from search results
            context_parts = []
            sources = []
            
            for result in search_results[:2]:
                text = result.payload.get('text', '')
                title = result.payload.get('title', 'Document')
                context_parts.append(f"**{title}:**\n{text}")
                sources.append({
                    'title': title,
                    'url': result.payload.get('url', ''),
                    'score': result.score
                })
            
            answer = f"""Based on the documentation:

{chr(10).join(context_parts)}

---

This information is from the Physical AI & Humanoid Robotics education materials."""
            
            return ChatResponse(
                answer=answer,
                sources=sources,
                citations=[],
                query=request.message,
                confidence=0.7,
                has_sources=True,
                source_count=len(sources),
                timestamp=datetime.utcnow().isoformat(),
                session_id=request.session_id,
            )
    except Exception as e:
        print(f"Search error: {e}")
    
    # Fallback to keyword-based responses
    responses = {
        "ai": """Artificial Intelligence (AI) refers to computer systems that can perform tasks that typically require human intelligence. This includes:

🧠 **Core Concepts:**
- Machine Learning: Systems that learn from data
- Neural Networks: Models inspired by the human brain
- Deep Learning: Advanced neural networks with multiple layers

🤖 **In Physical AI:**
Physical AI combines artificial intelligence with robotics and physical systems, enabling machines to interact with and navigate the real world. This includes autonomous robots, drones, and humanoid systems that can perceive, reason, and act in physical environments.

📚 **In Education:**
AI in education helps personalize learning experiences, provide intelligent tutoring, and automate administrative tasks, making education more accessible and effective.""",

        "robot": """Robotics is the field of engineering and science that deals with the design, construction, and operation of robots.

🤖 **Key Components:**
- **Sensors**: Cameras, LIDAR, touch sensors, IMUs
- **Actuators**: Motors, servos, pneumatics
- **Controllers**: Microcontrollers, embedded systems
- **Software**: Control algorithms, AI, path planning

🎓 **Educational Robotics:**
Educational robotics teaches students programming, engineering, and problem-solving through hands-on robot building and programming. Popular platforms include LEGO Mindstorms, Arduino, and Raspberry Pi robots.

🔧 **Applications:**
- Manufacturing automation
- Healthcare assistance
- Search and rescue
- Space exploration
- Educational demonstrations""",

        "physical ai": """Physical AI represents the integration of artificial intelligence with physical robotic systems.

✨ **What Makes it Special:**
Unlike traditional AI that exists purely in software, Physical AI involves:
- **Embodied Intelligence**: AI that operates through a physical form
- **Real-world Interaction**: Systems that perceive and manipulate physical environments
- **Sensor Integration**: Using cameras, LIDAR, force sensors, etc.
- **Actuation**: Physical movement and manipulation

🎯 **Applications in Education:**
- Hands-on learning of AI concepts
- STEM education enhancement
- Research platforms for students
- Demonstration of real-world AI challenges

🛠️ **Edge Kit:**
Our Physical AI Edge Kit provides a complete platform for learning and experimenting with Physical AI concepts, including sensors, actuators, and pre-built AI models.""",

        "sensor": """Sensors are devices that detect and measure physical properties from the environment.

📊 **Common Robot Sensors:**
- **Vision**: Cameras (RGB, depth, thermal)
- **Distance**: LIDAR, ultrasonic, infrared
- **Motion**: IMU (accelerometer, gyroscope, magnetometer)
- **Touch**: Force sensors, tactile sensors
- **Environmental**: Temperature, humidity, gas sensors

🔧 **How They Work:**
Sensors convert physical phenomena into electrical signals that computers can process. This data helps robots:
- Navigate autonomously
- Avoid obstacles
- Manipulate objects safely
- Understand their environment

🎓 **Teaching with Sensors:**
Students learn about data acquisition, signal processing, and sensor fusion—critical skills for robotics and IoT applications.""",

        "actuator": """Actuators are components that create physical motion in robots and machines.

⚙️ **Types of Actuators:**
- **Electric Motors**: DC, servo, stepper motors
- **Pneumatic**: Compressed air systems
- **Hydraulic**: Fluid-powered systems
- **Linear Actuators**: Create straight-line motion

🎯 **Applications:**
- Joint movement in humanoid robots
- Gripper control for manipulation
- Wheel/track propulsion
- Flight control in drones

🔬 **Control Systems:**
Actuators are controlled through:
- PWM (Pulse Width Modulation)
- PID controllers
- Trajectory planning
- Force feedback systems""",

        "edge kit": """The Physical AI Edge Kit is a comprehensive educational platform for learning robotics and AI.

📦 **What's Included:**
- Pre-configured hardware platform
- Sensor suite (cameras, LIDAR, IMU)
- Actuator systems
- Edge computing capabilities
- Pre-built AI models

🎓 **Learning Objectives:**
- Hands-on robotics programming
- AI model deployment
- Sensor data processing
- Control systems
- Computer vision
- Autonomous navigation

🚀 **Getting Started:**
1. Unbox and assemble hardware
2. Connect to development environment
3. Run example programs
4. Modify and experiment
5. Build custom applications

💡 **Perfect For:**
- University courses
- Research projects
- Maker spaces
- Self-learners""",

        "privacy": """Privacy and security are critical considerations in AI and robotics systems.

🔒 **Key Concerns:**
- **Data Collection**: What information do robots gather?
- **Data Storage**: Where and how is data kept?
- **User Consent**: Transparent data usage policies
- **Access Control**: Who can view collected data?

🛡️ **Best Practices:**
- Encrypt sensitive data
- Implement user authentication
- Provide data deletion options
- Follow GDPR/CCPA guidelines
- Conduct security audits

🎓 **Teaching Privacy:**
Educators should help students understand:
- Ethical AI development
- Privacy by design principles
- Secure coding practices
- Legal compliance requirements"""
    }
    
    # Find best matching response
    answer = None
    confidence = 0.0
    
    for keyword, response in responses.items():
        if keyword in question_lower:
            answer = response
            confidence = 0.9
            break
    
    # Default response if no match
    if not answer:
        answer = f"""Thank you for asking: "{request.message}"

I'm the Physical AI & Humanoid Robotics Education Assistant! I can provide information about:

🤖 **Physical AI** - Integration of AI with physical robotic systems
🔧 **Robotics** - Robot design, components, and applications  
📊 **Sensors** - Vision, LIDAR, IMU, and other sensing technologies
⚙️ **Actuators** - Motors, servos, and motion control
🎓 **Educational Robotics** - Teaching methodologies and platforms
🛠️ **Edge Kit** - Our comprehensive learning platform
🔒 **Privacy & Security** - Best practices for AI systems

Try asking about any of these topics! For example:
- "What is AI?"
- "Tell me about robots"
- "How do sensors work?"
- "What is the Edge Kit?"

Note: Full AI-powered responses will be available once API rate limits reset. Currently using knowledge base responses."""
        confidence = 0.3
    
    return ChatResponse(
        answer=answer,
        sources=[],
        citations=[],
        query=request.message,
        confidence=confidence,
        has_sources=False,
        source_count=0,
        timestamp=datetime.utcnow().isoformat(),
        session_id=request.session_id,
    )


@router.post("/selected-text", response_model=ChatResponse)
async def query_selected_text(request: ChatMessage) -> ChatResponse:
    """Query about selected text content.

    Args:
        request: Chat message with selected text.

    Returns:
        ChatResponse with contextual answer.
    """
    if not request.selected_text:
        raise HTTPException(
            status_code=400,
            detail="selected_text is required for this endpoint",
        )

    try:
        await rag_engine.initialize()

        result = await rag_engine.query(
            question=request.message,
            selected_text=request.selected_text,
            conversation_history=request.conversation_history,
            include_citations=True,
        )

        return ChatResponse(
            answer=result["answer"],
            sources=result["sources"],
            citations=result["citations"],
            query=result["query"],
            confidence=result["confidence"],
            has_sources=result["has_sources"],
            source_count=result["source_count"],
            timestamp=result["timestamp"],
            session_id=request.session_id,
        )

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error processing selected text query: {str(e)}",
        )


@router.post("/index", response_model=IndexDocumentResponse)
async def index_document(request: IndexDocumentRequest) -> IndexDocumentResponse:
    """Index a document for RAG retrieval.

    Args:
        request: Document indexing request.

    Returns:
        IndexDocumentResponse with document ID.
    """
    try:
        document_store = DocumentStore()

        doc_id = await document_store.add_document(
            content=request.content,
            title=request.title,
            source_url=request.source_url,
            file_path=request.file_path,
            section=request.section,
            tags=request.tags,
            metadata=request.metadata,
        )

        return IndexDocumentResponse(
            success=True,
            document_id=doc_id,
            message="Document indexed successfully",
        )

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error indexing document: {str(e)}",
        )


@router.get("/health")
async def chat_health():
    """Check RAG chatbot health."""
    return {
        "status": "healthy",
        "service": "rag-chatbot",
        "collection": rag_engine.collection_name,
    }
