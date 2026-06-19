"""Citation system for RAG responses."""
from typing import Optional


class CitationSystem:
    """Manages source citations for RAG responses."""

    def extract_citations(self, search_results: list) -> list[dict]:
        """Extract citations from search results.

        Args:
            search_results: List of search results from Qdrant.

        Returns:
            List of citation objects.
        """
        citations = []

        for i, result in enumerate(search_results, 1):
            payload = result.get("payload", {})
            score = result.get("score", 0)

            citation = {
                "number": i,
                "title": payload.get("title", "Untitled"),
                "source_url": payload.get("source_url", ""),
                "file_path": payload.get("file_path", ""),
                "section": payload.get("section", ""),
                "relevance_score": round(score, 3),
                "snippet": self._create_snippet(payload.get("content", "")),
            }

            citations.append(citation)

        return citations

    def _create_snippet(self, content: str, max_length: int = 150) -> str:
        """Create a snippet from content.

        Args:
            content: Full content text.
            max_length: Maximum snippet length.

        Returns:
            Truncated snippet.
        """
        if len(content) <= max_length:
            return content

        # Truncate at word boundary
        truncated = content[:max_length]
        last_space = truncated.rfind(" ")
        if last_space > max_length * 0.7:
            truncated = truncated[:last_space]

        return truncated + "..."

    def format_inline_citation(self, number: int) -> str:
        """Format an inline citation reference.

        Args:
            number: Citation number.

        Returns:
            Formatted citation string.
        """
        return f"[{number}]"

    def format_citation_list(self, citations: list[dict]) -> str:
        """Format citations as a readable list.

        Args:
            citations: List of citation objects.

        Returns:
            Formatted citation list string.
        """
        if not citations:
            return ""

        lines = ["**Sources:**"]
        for citation in citations:
            number = citation.get("number", "?")
            title = citation.get("title", "Untitled")
            source = citation.get("source_url") or citation.get("file_path", "")

            if source:
                lines.append(f"[{number}] {title} - {source}")
            else:
                lines.append(f"[{number}] {title}")

        return "\n".join(lines)

    def add_citations_to_response(
        self,
        response: str,
        citations: list[dict],
    ) -> str:
        """Add citation list to response.

        Args:
            response: Original response text.
            citations: List of citations.

        Returns:
            Response with citation list appended.
        """
        citation_list = self.format_citation_list(citations)
        if citation_list:
            return f"{response}\n\n{citation_list}"
        return response
