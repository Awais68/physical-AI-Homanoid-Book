"""RAG Engine - Core retrieval-augmented generation service."""
from typing import Optional
import hashlib
from src.clients.gemini_embedding_client import get_embedding
from src.clients.gemini_client import chat_completion
from src.clients.qdrant_client import search_similar, ensure_collection_exists, get_collection_info
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
        self._embedding_cache = {}  # Cache embeddings to avoid repeated API calls
        self._collection_has_data = False

    async def initialize(self) -> bool:
        """Initialize the RAG engine and ensure collection exists.

        Returns:
            True if initialization successful.
        """
        if self._initialized:
            return True
        
        self._initialized = ensure_collection_exists(
            self.collection_name,
            vector_size=768,  # Gemini text-embedding-004 dimensions
        )
        
        # Check if collection already has data
        if self._initialized:
            collection_info = get_collection_info(self.collection_name)
            if collection_info and collection_info.get('points_count', 0) > 0:
                self._collection_has_data = True
                print(f"✓ Collection '{self.collection_name}' has {collection_info['points_count']} documents - using existing data")
            else:
                print(f"⚠ Collection '{self.collection_name}' is empty - embeddings will be generated for new documents")
        
        return self._initialized
    
    def _get_cached_embedding(self, text: str) -> Optional[list]:
        """Get cached embedding for text if available.
        
        Args:
            text: Text to get embedding for.
            
        Returns:
            Cached embedding or None.
        """
        cache_key = hashlib.md5(text.encode()).hexdigest()
        return self._embedding_cache.get(cache_key)
    
    def _cache_embedding(self, text: str, embedding: list) -> None:
        """Cache embedding for text.
        
        Args:
            text: Text that was embedded.
            embedding: Embedding vector to cache.
        """
        cache_key = hashlib.md5(text.encode()).hexdigest()
        self._embedding_cache[cache_key] = embedding
        
        # Keep cache size limited
        if len(self._embedding_cache) > 100:
            # Remove oldest entry (first inserted)
            oldest_key = next(iter(self._embedding_cache))
            del self._embedding_cache[oldest_key]

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

        # Check cache first to avoid unnecessary API calls
        query_embedding = self._get_cached_embedding(enhanced_query)
        
        if query_embedding is None:
            # Only call embedding API if not in cache and needed
            if self._collection_has_data:
                # Collection has data, generate embedding for search
                query_embedding = get_embedding(enhanced_query)
                self._cache_embedding(enhanced_query, query_embedding)
                print("✓ Generated embedding for query (cached for future use)")
            else:
                # Collection is empty, use fallback
                print("⚠ Collection empty - using fallback embedding")
                from src.clients.gemini_embedding_client import simple_embedding
                query_embedding = simple_embedding(enhanced_query)
        else:
            print("✓ Using cached embedding for query")

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
