# rag-retriever

## Purpose
Semantic search across indexed book content in Qdrant vector database to find relevant information based on natural language queries.

## Capabilities
- Query Qdrant for relevant chunks based on natural language
- Support hybrid search (vector + keyword)
- Re-rank results by relevance
- Return source references (file:line) for attribution
- Configurable result count and similarity threshold
- Filter by metadata (section, chapter, tags, version)
- Support for multiple query types (exact, fuzzy, semantic)
- Provide confidence scores for retrieved results

## Tools Available
- Bash: Interact with Qdrant API for search operations
- Read: Read configuration and metadata when needed

## Input Parameters
- `query`: Natural language query string to search for
- `collection_name`: Name of Qdrant collection to search in (default: docusaurus_content)
- `top_k`: Number of results to return (default: 5)
- `similarity_threshold`: Minimum similarity score (default: 0.3)
- `filters`: Metadata filters as JSON object (optional)
- `search_type`: Search method (vector, keyword, hybrid) (default: hybrid)
- `rerank`: Whether to re-rank results (default: true)

## Output
- Ranked results array with:
  - content: The retrieved text chunk
  - score: Similarity score (0.0-1.0)
  - source: File path and line reference
  - metadata: Associated metadata (title, section, tags)
  - confidence: Confidence score for the result
- Query execution time
- Total matches found before filtering

## Configuration Requirements
Environment variables needed:
- `QDRANT_URL`: Qdrant server URL (default: http://localhost:6333)
- `QDRANT_API_KEY`: Qdrant API key (optional for local)

## Example Usage
```
skill: "rag-retriever"
query: "How to implement authentication in React applications?"
collection_name: "react_book_content"
top_k: 3
similarity_threshold: 0.5
filters: {"tags": ["authentication", "security"]}
```

## Technical Implementation Details
1. Construct Qdrant search query with vector and optional keyword components
2. Apply metadata filters if provided
3. Execute hybrid search combining vector similarity and keyword matching
4. Re-rank results using secondary scoring if requested
5. Format results with proper source attribution
6. Include confidence scores based on similarity thresholds
7. Support pagination for large result sets

## Search Types
- **Vector**: Pure semantic similarity search
- **Keyword**: Traditional keyword-based search
- **Hybrid**: Combined approach using both methods with weighted scoring

## Metadata Filters
Support filtering by:
- tags: Array of tags from frontmatter
- section: Document section/chapter
- title: Document title
- file_path: Original file location
- version: Version information for versioned docs

## Error Handling
- Handle connection errors to Qdrant
- Graceful degradation if filters are malformed
- Return empty results instead of failing when no matches found
- Clear error messages for invalid query parameters
- Validation of collection existence before searching

## Performance Considerations
- Efficient query construction to minimize API calls
- Proper indexing of metadata fields for fast filtering
- Caching of frequent queries where appropriate
- Pagination support for large result sets