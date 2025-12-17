"""RAG Engine - Core retrieval-augmented generation service."""
from typing import Optional
from src.clients.cohere_client import get_embedding
from src.clients.openai_client import chat_completion
from src.clients.qdrant_client import search_similar, ensure_collection_exists
from src.config.settings import settings
from .conversation_context import ConversationContext
from .citation_system import CitationSystem
from .response_formatter import ResponseFormatter


class RAGEngine:
    """Core RAG engine for question answering with document retrieval."""

    def __init__(self, collection_name: Optional[str] = None):
        """Initialize RAG engine.

        Args:
            collection_name: Qdrant collection name for document storage.
        """
        self.collection_name = collection_name or settings.QDRANT_COLLECTION
        self.context_manager = ConversationContext()
        self.citation_system = CitationSystem()
        self.response_formatter = ResponseFormatter()
        self._initialized = False

    async def initialize(self) -> bool:
        """Initialize the RAG engine and ensure collection exists.

        Returns:
            True if initialization successful.
        """
        if self._initialized:
            return True
        self._initialized = ensure_collection_exists(
            self.collection_name,
            vector_size=1024,  # Cohere embed-english-v3.0 dimensions
        )
        return self._initialized

    async def query(
        self,
        question: str,
        conversation_history: Optional[list] = None,
        selected_text: Optional[str] = None,
        top_k: int = 5,
        include_citations: bool = True,
    ) -> dict:
        """Process a question using RAG.

        Args:
            question: User question to answer.
            conversation_history: Previous conversation messages.
            selected_text: Optional selected text for context filtering.
            top_k: Number of documents to retrieve.
            include_citations: Whether to include source citations.

        Returns:
            Dictionary with answer, sources, and metadata.
        """
        # Build context-aware query
        enhanced_query = self.context_manager.build_query(
            question=question,
            conversation_history=conversation_history,
            selected_text=selected_text,
        )

        # Generate embedding for the query
        query_embedding = get_embedding(enhanced_query)

        # Search for relevant documents
        search_results = search_similar(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            top_k=top_k,
            score_threshold=settings.RAG_SIMILARITY_THRESHOLD,
        )

        # Build context from retrieved documents
        context_text = self._build_context(search_results)

        # Generate answer with context
        system_prompt = self._get_system_prompt(context_text, include_citations)

        messages = []
        if conversation_history:
            messages.extend(conversation_history[-6:])  # Last 6 messages for context

        messages.append({"role": "user", "content": question})

        answer = chat_completion(
            messages=messages,
            system_prompt=system_prompt,
            max_tokens=settings.RAG_MAX_RESPONSE_TOKENS,
            temperature=0.7,
        )

        # Format response with citations
        citations = []
        if include_citations:
            citations = self.citation_system.extract_citations(search_results)

        return self.response_formatter.format_response(
            answer=answer,
            sources=search_results,
            citations=citations,
            query=question,
        )

    def _build_context(self, search_results: list) -> str:
        """Build context string from search results.

        Args:
            search_results: List of search results from Qdrant.

        Returns:
            Formatted context string.
        """
        if not search_results:
            return "No relevant documentation found."

        context_parts = []
        for i, result in enumerate(search_results, 1):
            payload = result.get("payload", {})
            # Support both 'text' (from main.py ingestion) and 'content' (from src/ indexing)
            content = payload.get("text", payload.get("content", ""))
            title = payload.get("title", "Document")
            source = payload.get("url", payload.get("source_url", payload.get("file_path", "")))

            context_parts.append(
                f"[Source {i}] {title}\n"
                f"Content: {content}\n"
                f"Reference: {source}\n"
            )

        return "\n---\n".join(context_parts)

    def _get_system_prompt(self, context: str, include_citations: bool) -> str:
        """Generate system prompt for RAG responses.

        Args:
            context: Retrieved document context.
            include_citations: Whether to include citation instructions.

        Returns:
            System prompt string.
        """
        citation_instruction = ""
        if include_citations:
            citation_instruction = (
                "When answering, cite your sources using [Source N] notation "
                "where N corresponds to the source number in the context. "
            )

        return f"""You are an educational AI assistant for the Physical AI Edge Kit.
Your role is to help educators and students understand robotics, physical AI systems,
and educational content.

{citation_instruction}

IMPORTANT GUIDELINES:
1. Answer questions accurately based on the provided context
2. If the context doesn't contain relevant information, say so honestly
3. Keep answers clear, educational, and appropriate for the audience
4. For technical questions, provide step-by-step explanations when helpful
5. If asked about safety, always emphasize proper safety protocols

CONTEXT FROM DOCUMENTATION:
{context}

Remember: Only use information from the provided context. If you cannot find
relevant information, acknowledge this and suggest where the user might find help."""
