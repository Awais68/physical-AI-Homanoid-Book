# RAG Skills Configuration

## Environment Variables (.env)

```env
# Qdrant Configuration
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your_qdrant_api_key_here  # Optional for local Qdrant

# Embedding Provider Configuration (Choose one or more)
OPENAI_API_KEY=your_openai_api_key_here
ANTHROPIC_API_KEY=your_anthropic_api_key_here

# Alternative: Local embedding model (if using sentence-transformers)
# LOCAL_EMBEDDING_MODEL=all-MiniLM-L6-v2  # or other model name

# Optional: Configuration for answer generation
CLAUDE_API_KEY=your_claude_api_key_here  # For rag-answerer
```

## Dependencies

### Python Dependencies (requirements.txt)
```txt
qdrant-client>=1.9.0
openai>=1.0.0
anthropic>=0.5.0
sentence-transformers>=2.6.0
python-dotenv>=1.0.0
tiktoken>=0.5.0  # For token counting
PyYAML>=6.0
```

### Node.js Dependencies (package.json)
```json
{
  "dependencies": {
    "@qdrant/js-client-rest": "^1.9.0",
    "openai": "^4.0.0",
    "anthropic": "^0.5.0",
    "tiktoken": "^1.0.0",
    "glob": "^10.0.0",
    "yaml": "^2.0.0"
  }
}
```

## Qdrant Collection Schema

### Default Collection Configuration
```json
{
  "vector_size": 1536,
  "distance": "Cosine",
  "payload_schema": {
    "file_path": "keyword",
    "title": "keyword",
    "section": "keyword",
    "tags": "keyword[]",
    "created_at": "integer",
    "updated_at": "integer",
    "version": "keyword",
    "chunk_id": "keyword"
  }
}
```

## Configuration File (config.yaml)

```yaml
rag:
  # Default collection name
  default_collection: "docusaurus_content"

  # Chunking settings
  chunking:
    max_tokens: 500
    overlap_tokens: 50
    preserve_code_blocks: true
    respect_headers: true

  # Embedding settings
  embedding:
    provider: "openai"  # openai, anthropic, local
    model: "text-embedding-3-small"
    batch_size: 10

  # Retrieval settings
  retrieval:
    top_k: 5
    similarity_threshold: 0.3
    search_type: "hybrid"  # vector, keyword, hybrid

  # Q&A settings
  qa:
    max_tokens: 1000
    answer_length: "medium"  # short, medium, long
    include_citations: true
    confidence_threshold: 0.5
```

## Docusaurus Integration

### Reading Docusaurus Configuration
The RAG skills will automatically read from:
- `docusaurus.config.js` or `docusaurus.config.ts`
- `sidebars.js` or `sidebars.ts`
- Frontmatter in markdown files

### Supported Docusaurus Features
- Versioned documentation
- Multiple locales/languages
- Custom frontmatter fields
- MDX files support

## Setup Instructions

1. Install dependencies:
   ```bash
   npm install @qdrant/js-client-rest openai anthropic tiktoken glob yaml
   # OR
   pip install qdrant-client openai anthropic sentence-transformers python-dotenv tiktoken PyYAML
   ```

2. Set up Qdrant:
   ```bash
   # Using Docker
   docker run -p 6333:6333 -p 6334:6334 \
     -v $(pwd)/qdrant_storage:/qdrant/storage:z \
     qdrant/qdrant
   ```

3. Configure environment variables in `.env` file

4. Initialize collections using rag-manager:
   ```
   skill: "rag-manager"
   operation: "create"
   collection_name: "docusaurus_content"
   ```

5. Index your documentation:
   ```
   skill: "rag-indexer"
   docs_path: "./docs"
   collection_name: "docusaurus_content"
   ```

## Usage Examples

### Indexing Documentation
```yaml
skill: "rag-indexer"
docs_path: "./docs"
collection_name: "my_book_content"
chunk_size: 400
embedding_model: "openai-text-embedding-3-small"
```

### Searching Content
```yaml
skill: "rag-retriever"
query: "How to implement authentication?"
collection_name: "my_book_content"
top_k: 3
filters: {"tags": ["security", "authentication"]}
```

### Getting Answers
```yaml
skill: "rag-answerer"
question: "What are the best practices for React hooks?"
collection_name: "react_book_content"
answer_length: "long"
num_sources: 5
```

### Managing Collections
```yaml
skill: "rag-manager"
operation: "stats"
collection_name: "my_book_content"
```