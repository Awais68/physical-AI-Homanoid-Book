"""RAG client for Qdrant vector database integration."""

import logging
import time
from typing import Any

from openai import OpenAI
from qdrant_client import QdrantClient, models
from qdrant_client.http.exceptions import UnexpectedResponse

from src.config import get_settings, is_rag_enabled
from src.text_processor.models import Citation, RAGContext

logger = logging.getLogger(__name__)


class RAGClient:
    """Client for retrieving context from Qdrant vector database."""

    def __init__(self, openai_client: OpenAI | None = None):
        """Initialize RAG client.

        Args:
            openai_client: Optional OpenAI client for query embeddings
        """
        self.settings = get_settings()
        self.enabled = is_rag_enabled() and self.settings.rag_enabled

        if not self.enabled:
            logger.info("RAG client disabled (rag_enabled=False or not configured)")
            return

        try:
            # Initialize Qdrant client
            self.qdrant = QdrantClient(
                url=self.settings.qdrant_url,
                api_key=self.settings.qdrant_api_key,
                timeout=self.settings.qdrant_timeout,
            )
            self.openai = openai_client or OpenAI(api_key=self.settings.openai_api_key)

            # Verify collection exists
            collections = self.qdrant.get_collections().collections
            collection_names = [c.name for c in collections]

            if self.settings.qdrant_collection not in collection_names:
                logger.warning(
                    f"Qdrant collection '{self.settings.qdrant_collection}' not found. "
                    f"Available collections: {collection_names}"
                )
                self.enabled = False
            else:
                logger.info(
                    f"RAG client initialized: collection={self.settings.qdrant_collection}, "
                    f"top_k={self.settings.rag_top_k}, "
                    f"threshold={self.settings.rag_similarity_threshold}"
                )
        except Exception as e:
            logger.error(f"Failed to initialize RAG client: {e}")
            self.enabled = False
            self.qdrant = None

    def retrieve_context(self, query: str) -> RAGContext:
        """Retrieve relevant context from Qdrant for a query.

        Args:
            query: User query to retrieve context for

        Returns:
            RAGContext with retrieved documents and metadata
        """
        if not self.enabled or not self.qdrant:
            logger.debug("RAG disabled or unavailable, returning empty context")
            return RAGContext(query=query, documents=[], retrieval_time_ms=0)

        start_time = time.time()

        try:
            # Generate query embedding
            embedding_response = self.openai.embeddings.create(
                input=query,
                model="text-embedding-3-small"
            )
            query_vector = embedding_response.data[0].embedding

            # Search Qdrant
            search_results = self.qdrant.search(
                collection_name=self.settings.qdrant_collection,
                query_vector=query_vector,
                limit=self.settings.rag_top_k,
                score_threshold=self.settings.rag_similarity_threshold,
                with_payload=True,
            )

            retrieval_time_ms = int((time.time() - start_time) * 1000)

            # Convert results to citations
            citations = self._convert_to_citations(search_results)

            # Format context for LLM
            formatted_context = self._format_context(citations)

            logger.info(
                f"RAG retrieval: query='{query[:50]}...', "
                f"documents={len(citations)}, time={retrieval_time_ms}ms"
            )

            return RAGContext(
                query=query,
                documents=citations,
                retrieval_time_ms=retrieval_time_ms,
                formatted_context=formatted_context,
            )

        except Exception as e:
            logger.error(f"RAG retrieval failed: {e}")
            # Graceful degradation: return empty context
            return RAGContext(
                query=query,
                documents=[],
                retrieval_time_ms=0,
                formatted_context="",
            )

    def _convert_to_citations(self, search_results: list[models.ScoredPoint]) -> list[Citation]:
        """Convert Qdrant search results to Citation objects.

        Args:
            search_results: Qdrant scored points

        Returns:
            List of Citation objects
        """
        citations = []

        for idx, result in enumerate(search_results):
            payload = result.payload or {}

            citation = Citation(
                source_index=idx,
                title=payload.get("title", "Unknown Document"),
                source_url=payload.get("source_url"),
                file_path=payload.get("file_path"),
                snippet=payload.get("content", payload.get("text", ""))[:500],
                relevance_score=float(result.score),
            )
            citations.append(citation)

        return citations

    def _format_context(self, citations: list[Citation]) -> str:
        """Format citations as context string for LLM prompt.

        Args:
            citations: List of citations to format

        Returns:
            Formatted context string
        """
        if not citations:
            return ""

        sections = []
        for idx, citation in enumerate(citations):
            sections.append(
                f"[Source {idx + 1}] {citation.title}\n"
                f"{citation.snippet}\n"
            )

        return "Based on the following Physical AI documentation:\n\n" + "\n".join(sections)


def format_citations_inline(citations: list[Citation]) -> str:
    """Format citations for inline inclusion in response.

    Args:
        citations: List of citations

    Returns:
        Formatted citation string
    """
    if not citations:
        return ""

    return "\n\nReferences:\n" + "\n".join(
        f"- [{c.source_index + 1}] {c.title}"
        for c in citations
    )


def format_citation_list(citations: list[Citation]) -> list[dict[str, Any]]:
    """Format citations for JSON response.

    Args:
        citations: List of citations

    Returns:
        List of citation dictionaries
    """
    return [
        {
            "index": c.source_index + 1,
            "title": c.title,
            "source_url": c.source_url,
            "file_path": c.file_path,
            "snippet": c.snippet,
            "relevance_score": c.relevance_score,
        }
        for c in citations
    ]
