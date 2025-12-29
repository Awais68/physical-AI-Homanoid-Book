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
    try:
        # Initialize RAG engine
        await rag_engine.initialize()

        # Query using RAG engine with conversation history
        result = await rag_engine.query(
            question=request.message,
            conversation_history=request.conversation_history,
            selected_text=request.selected_text,
            top_k=5,
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
            detail=f"Error processing chat message: {str(e)}",
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
