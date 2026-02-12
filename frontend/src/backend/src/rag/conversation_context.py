"""Conversation context management for RAG chatbot."""
from typing import Optional


class ConversationContext:
    """Manages conversation context for multi-turn interactions."""

    def __init__(self, max_history_length: int = 10):
        """Initialize conversation context manager.

        Args:
            max_history_length: Maximum number of messages to retain.
        """
        self.max_history_length = max_history_length

    def build_query(
        self,
        question: str,
        conversation_history: Optional[list] = None,
        selected_text: Optional[str] = None,
    ) -> str:
        """Build an enhanced query with context.

        Args:
            question: Current user question.
            conversation_history: Previous conversation messages.
            selected_text: Optional selected text for context.

        Returns:
            Enhanced query string.
        """
        query_parts = []

        # Add selected text context if provided
        if selected_text:
            query_parts.append(f"Context from selected text: {selected_text}")

        # Add relevant conversation context
        if conversation_history:
            recent_context = self._extract_relevant_context(
                conversation_history,
                question,
            )
            if recent_context:
                query_parts.append(f"Previous context: {recent_context}")

        # Add the current question
        query_parts.append(question)

        return " | ".join(query_parts)

    def _extract_relevant_context(
        self,
        conversation_history: list,
        current_question: str,
    ) -> str:
        """Extract relevant context from conversation history.

        Args:
            conversation_history: List of previous messages.
            current_question: Current question for relevance.

        Returns:
            Relevant context string.
        """
        if not conversation_history:
            return ""

        # Get last few messages for context
        recent_messages = conversation_history[-self.max_history_length:]

        # Extract key topics from recent messages
        context_parts = []
        for msg in recent_messages:
            if msg.get("role") == "user":
                content = msg.get("content", "")[:200]  # Limit length
                context_parts.append(content)

        return " ... ".join(context_parts[-3:])  # Last 3 user messages

    def format_history_for_prompt(
        self,
        conversation_history: list,
        max_messages: int = 6,
    ) -> list:
        """Format conversation history for prompt.

        Args:
            conversation_history: Full conversation history.
            max_messages: Maximum messages to include.

        Returns:
            Formatted history list.
        """
        if not conversation_history:
            return []

        # Take only recent messages
        recent = conversation_history[-max_messages:]

        # Format for OpenAI API
        formatted = []
        for msg in recent:
            role = msg.get("role", "user")
            content = msg.get("content", "")
            if role in ["user", "assistant", "system"]:
                formatted.append({"role": role, "content": content})

        return formatted

    def summarize_context(
        self,
        conversation_history: list,
    ) -> dict:
        """Summarize conversation context.

        Args:
            conversation_history: Full conversation history.

        Returns:
            Context summary dict.
        """
        if not conversation_history:
            return {
                "message_count": 0,
                "topics": [],
                "last_topic": None,
            }

        # Extract topics from user messages
        topics = []
        for msg in conversation_history:
            if msg.get("role") == "user":
                content = msg.get("content", "")
                # Simple topic extraction - first few words
                words = content.split()[:5]
                if words:
                    topics.append(" ".join(words))

        return {
            "message_count": len(conversation_history),
            "topics": topics[-5:],  # Last 5 topics
            "last_topic": topics[-1] if topics else None,
        }
