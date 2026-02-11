# rag-answerer

## Purpose
Answer user questions using retrieved context from Qdrant vector database with proper citations and source attribution.

## Capabilities
- Use rag-retriever to fetch relevant chunks based on user query
- Construct prompts with retrieved context and question
- Generate answers with inline citations to sources
- Handle multi-hop questions with iterative retrieval
- Provide "I don't know" responses when context is insufficient
- Return confidence scores and complete source list
- Support different answer length preferences
- Maintain conversation context for follow-up questions

## Tools Available
- Skill: Invoke rag-retriever skill to fetch relevant content
- Read: Read configuration and templates when needed

## Input Parameters
- `question`: User question to answer
- `collection_name`: Qdrant collection to search in (default: docusaurus_content)
- `answer_length`: Answer length preference (short, medium, long) (default: medium)
- `num_sources`: Number of sources to retrieve (default: 5)
- `similarity_threshold`: Minimum similarity for retrieved content (default: 0.3)
- `include_citations`: Whether to include inline citations (default: true)
- `max_tokens`: Maximum tokens for answer generation (default: 1000)

## Output
- `answer`: Generated answer to the user's question
- `citations`: Inline citations within the answer
- `sources`: Complete list of sources used
- `confidence`: Confidence score for the answer (0.0-1.0)
- `retrieval_stats`: Statistics about retrieved content
- `followup_questions`: Suggested follow-up questions (optional)

## Configuration Requirements
Environment variables needed:
- `QDRANT_URL`: Qdrant server URL (default: http://localhost:6333)
- `QDRANT_API_KEY`: Qdrant API key (optional for local)
- `CLAUDE_API_KEY`: Anthropic API key for answer generation
- `OPENAI_API_KEY`: OpenAI API key (alternative for answer generation)

## Example Usage
```
skill: "rag-answerer"
question: "What are the best practices for React component design?"
collection_name: "react_book_content"
answer_length: "long"
num_sources: 3
include_citations: true
```

## Technical Implementation Details
1. Call rag-retriever skill with user question to get relevant context
2. Construct a prompt combining question and retrieved context
3. Generate answer using language model with proper formatting
4. Insert inline citations referencing the source documents
5. Calculate confidence based on similarity scores of retrieved content
6. Handle multi-hop questions by performing iterative retrieval
7. Format response with proper citations and source attribution
8. Support different answer length preferences through prompt engineering

## Answer Length Options
- **Short**: Concise answers with key points (1-2 sentences)
- **Medium**: Detailed answers with explanations (3-5 sentences)
- **Long**: Comprehensive answers with examples (paragraph or more)

## Citation Format
- Inline citations like [1], [2], [3] within the answer
- Complete source list at the end with file paths and line references
- Source numbering corresponds to relevance ranking

## Multi-Hop Question Handling
- For complex questions requiring multiple pieces of information
- Iteratively retrieve additional context based on intermediate answers
- Combine information from multiple retrieval steps
- Maintain context across retrieval iterations

## Confidence Scoring
- Based on average similarity scores of retrieved content
- Adjusted based on relevance of content to the question
- Lower scores when retrieved content is tangentially related
- "I don't know" responses when confidence falls below threshold

## Error Handling
- Handle cases where no relevant content is found
- Graceful degradation when language model is unavailable
- Clear error messages for configuration issues
- Validation of required parameters before processing
- Fallback to general knowledge when context is insufficient

## Performance Considerations
- Efficient prompt construction to minimize token usage
- Caching of recent answers for identical questions
- Parallel processing of retrieval and generation when possible
- Proper timeout handling for API calls