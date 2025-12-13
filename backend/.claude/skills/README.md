# RAG Skills for Docusaurus + Qdrant

Comprehensive set of Claude Code Skills for implementing Retrieval-Augmented Generation (RAG) with Docusaurus documentation and Qdrant vector database.

## Overview

This package provides four specialized skills for building a complete RAG system:

1. **rag-indexer** - Index Docusaurus content into Qdrant
2. **rag-retriever** - Semantic search across indexed content
3. **rag-answerer** - Answer questions using retrieved context
4. **rag-manager** - Manage Qdrant collections and configurations

## Architecture

```text
Docusaurus Docs → rag-indexer → Qdrant Vector DB
                                    ↓
User Query → rag-retriever → Relevant Chunks
                                    ↓
User Query + Chunks → rag-answerer → Answer + Citations
                                    ↑
                              rag-manager (admin)
```

## Prerequisites

- Qdrant vector database (local or hosted)
- API keys for embedding providers (OpenAI, Anthropic, etc.)
- Docusaurus documentation site with markdown files

## Installation

1. Create the skills directory:

   ```bash
   mkdir -p .claude/skills
   ```

2. Copy all skill files to `.claude/skills/`:
   - `rag-indexer.md`
   - `rag-retriever.md`
   - `rag-answerer.md`
   - `rag-manager.md`
   - `config.md`

3. Set up environment variables in `.env`:

   ```env
   QDRANT_URL=http://localhost:6333
   OPENAI_API_KEY=your_key_here
   # ... other variables as needed
   ```

## Quick Start

### 1. Set up Qdrant Collection

```bash
# Create a collection for your documentation
skill: "rag-manager"
operation: "create"
collection_name: "my_docs"
vector_size: 1536
distance_metric: "cosine"
```

### 2. Index Your Documentation

```bash
# Index all markdown files from your Docusaurus docs
skill: "rag-indexer"
docs_path: "./docs"
collection_name: "my_docs"
chunk_size: 500
embedding_model: "openai-text-embedding-3-small"
```

### 3. Search and Retrieve

```bash
# Search for relevant content
skill: "rag-retriever"
query: "How to deploy a Docusaurus site?"
collection_name: "my_docs"
top_k: 3
```

### 4. Get Answers with Citations

```bash
# Ask questions about your documentation
skill: "rag-answerer"
question: "What are the deployment options for Docusaurus?"
collection_name: "my_docs"
answer_length: "medium"
```

## Detailed Usage

### rag-indexer

Indexes Docusaurus markdown content into Qdrant with intelligent chunking:

```yaml
skill: "rag-indexer"
docs_path: "./docs"                    # Path to Docusaurus docs
collection_name: "docusaurus_content"  # Qdrant collection name
chunk_size: 400                       # Max tokens per chunk
chunk_overlap: 50                     # Overlap between chunks
embedding_model: "openai-text-embedding-3-small"  # Embedding model
batch_size: 10                        # Batch processing size
```

Features:

- Semantic chunking respecting markdown headers
- Frontmatter metadata extraction
- Incremental updates (only new/changed files)
- Progress tracking and error handling

### rag-retriever

Performs semantic search with filtering and ranking:

```yaml
skill: "rag-retriever"
query: "authentication best practices"  # Natural language query
collection_name: "my_docs"
top_k: 5                              # Number of results
similarity_threshold: 0.4             # Minimum similarity
filters: {"tags": ["security"]}       # Metadata filters
search_type: "hybrid"                 # vector, keyword, or hybrid
```

Features:

- Hybrid search combining vector and keyword matching
- Metadata filtering support
- Re-ranking for better relevance
- Source attribution with file:line references

### rag-answerer

Generates answers with citations from retrieved context:

```yaml
skill: "rag-answerer"
question: "How to configure authentication?"  # User question
collection_name: "auth_docs"
answer_length: "long"                  # short, medium, or long
num_sources: 3                         # Number of sources to use
include_citations: true               # Include inline citations
max_tokens: 800                       # Max tokens in answer
```

Features:

- Context-aware answer generation
- Inline citations and source lists
- Confidence scoring
- Multi-hop question handling
- "I don't know" responses when context insufficient

### rag-manager

Manages Qdrant collections and configurations:

```yaml
skill: "rag-manager"
operation: "stats"                    # Operation type
collection_name: "my_docs"            # Target collection
```

Available operations:

- `create` - Create new collection
- `delete` - Delete collection and data
- `list` - List all collections
- `stats` - Get collection statistics
- `health` - Check collection health
- `export` - Export collection data
- `import` - Import collection data
- `clear_stale` - Remove outdated chunks

## Configuration

See `config.md` for detailed configuration options including:

- Environment variables
- Collection schema
- Chunking parameters
- Embedding provider settings

## Best Practices

1. **Chunking Strategy**: Use 400-500 tokens with 50-token overlap for optimal retrieval
2. **Collection Management**: Create separate collections for different documentation sets
3. **Metadata**: Leverage frontmatter for better filtering and organization
4. **Embeddings**: Use the latest embedding models for best semantic search results
5. **Monitoring**: Regularly check collection stats and health

## Error Handling

All skills provide:

- Clear error messages for configuration issues
- Graceful degradation when services are unavailable
- Progress tracking for long-running operations
- Validation of parameters before execution

## Performance Tips

- Use batch processing for indexing large document sets
- Configure appropriate similarity thresholds
- Enable payload indexing for faster metadata filtering
- Monitor Qdrant resource usage
- Consider caching for frequently asked questions

## Integration with Docusaurus

The skills automatically:

- Parse Docusaurus configuration files
- Extract metadata from frontmatter
- Handle versioned documentation
- Support multiple languages/translations
- Respect Docusaurus content structure
