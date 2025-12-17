"""Response formatter for RAG outputs."""
from typing import Optional
from datetime import datetime


class ResponseFormatter:
    """Formats RAG responses for API output."""

    def format_response(
        self,
        answer: str,
        sources: list,
        citations: list,
        query: str,
        confidence: Optional[float] = None,
    ) -> dict:
        """Format a complete RAG response.

        Args:
            answer: Generated answer text.
            sources: Retrieved source documents.
            citations: Extracted citations.
            query: Original query.
            confidence: Optional confidence score.

        Returns:
            Formatted response dictionary.
        """
        # Calculate confidence if not provided
        if confidence is None:
            confidence = self._calculate_confidence(sources)

        return {
            "answer": answer,
            "sources": self._format_sources(sources),
            "citations": citations,
            "query": query,
            "confidence": confidence,
            "has_sources": len(sources) > 0,
            "source_count": len(sources),
            "timestamp": datetime.utcnow().isoformat(),
        }

    def _format_sources(self, sources: list) -> list[dict]:
        """Format source documents for output.

        Args:
            sources: Raw source documents.

        Returns:
            Formatted source list.
        """
        formatted = []
        for i, source in enumerate(sources, 1):
            payload = source.get("payload", {})
            formatted.append({
                "index": i,
                "title": payload.get("title", "Untitled"),
                "content_preview": payload.get("content", "")[:200],
                "source_url": payload.get("source_url", ""),
                "file_path": payload.get("file_path", ""),
                "relevance_score": source.get("score", 0),
            })
        return formatted

    def _calculate_confidence(self, sources: list) -> float:
        """Calculate confidence score based on sources.

        Args:
            sources: Retrieved sources with scores.

        Returns:
            Confidence score between 0 and 1.
        """
        if not sources:
            return 0.0

        # Average of top source scores
        scores = [s.get("score", 0) for s in sources[:3]]
        if not scores:
            return 0.0

        return round(sum(scores) / len(scores), 3)

    def format_error_response(
        self,
        error_message: str,
        query: str,
    ) -> dict:
        """Format an error response.

        Args:
            error_message: Error description.
            query: Original query.

        Returns:
            Error response dictionary.
        """
        return {
            "answer": f"I apologize, but I encountered an issue: {error_message}",
            "sources": [],
            "citations": [],
            "query": query,
            "confidence": 0.0,
            "has_sources": False,
            "source_count": 0,
            "error": error_message,
            "timestamp": datetime.utcnow().isoformat(),
        }

    def format_no_answer_response(
        self,
        query: str,
        suggestion: Optional[str] = None,
    ) -> dict:
        """Format a response when no answer can be found.

        Args:
            query: Original query.
            suggestion: Optional suggestion for the user.

        Returns:
            No-answer response dictionary.
        """
        answer = (
            "I couldn't find relevant information in the documentation to answer "
            "your question."
        )
        if suggestion:
            answer += f" {suggestion}"

        return {
            "answer": answer,
            "sources": [],
            "citations": [],
            "query": query,
            "confidence": 0.0,
            "has_sources": False,
            "source_count": 0,
            "timestamp": datetime.utcnow().isoformat(),
        }
