# RAG Skills Setup Guide

## Prerequisites

Before using the RAG skills, ensure you have:

1. **Qdrant Vector Database**
   - Local installation via Docker or hosted service
   - Access to Qdrant API

2. **API Keys** (depending on embedding provider):
   - OpenAI API key for OpenAI embeddings
   - Anthropic API key for Anthropic models
   - Or local embedding model setup

3. **Docusaurus Documentation**
   - Markdown files in a docs directory
   - Optional: Docusaurus configuration files

## Installation Steps

### 1. Clone or Copy Skill Files
```bash
# Create the skills directory
mkdir -p .claude/skills

# Copy all skill files to the directory
# - rag-indexer.md
# - rag-retriever.md
# - rag-answerer.md
# - rag-manager.md
# - config.md
# - README.md
```

### 2. Set Up Environment Variables
Create a `.env` file in your project root:

```env
# Qdrant Configuration
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your_api_key_here  # Optional for local

# Embedding Provider (Choose one or more)
OPENAI_API_KEY=your_openai_api_key
ANTHROPIC_API_KEY=your_anthropic_api_key

# Language Model for Answer Generation
CLAUDE_API_KEY=your_claude_api_key
```

### 3. Install Dependencies

For Python-based implementation:
```bash
pip install qdrant-client openai anthropic sentence-transformers python-dotenv tiktoken PyYAML
```

For Node.js-based implementation:
```bash
npm install @qdrant/js-client-rest openai anthropic tiktoken yaml
```

### 4. Start Qdrant Database

Option 1: Docker (recommended for development)
```bash
docker run -p 6333:6333 -p 6334:6334 \
  -v $(pwd)/qdrant_storage:/qdrant/storage:z \
  qdrant/qdrant
```

Option 2: Cloud service (Qdrant Cloud or other provider)

## Configuration

### Collection Schema
The skills expect a Qdrant collection with the following payload schema:

```json
{
  "file_path": "keyword",
  "title": "keyword",
  "section": "keyword",
  "tags": "keyword[]",
  "created_at": "integer",
  "updated_at": "integer",
  "version": "keyword",
  "chunk_id": "keyword"
}
```

### Chunking Parameters
- Max tokens per chunk: 500
- Overlap tokens: 50
- Preserve code blocks: true
- Respect markdown headers: true

## First-Time Setup Workflow

### 1. Create Collection
```bash
skill: "rag-manager"
operation: "create"
collection_name: "my_documentation"
vector_size: 1536
distance_metric: "cosine"
```

### 2. Index Your Content
```bash
skill: "rag-indexer"
docs_path: "./docs"
collection_name: "my_documentation"
chunk_size: 500
embedding_model: "openai-text-embedding-3-small"
```

### 3. Test Retrieval
```bash
skill: "rag-retriever"
query: "test query"
collection_name: "my_documentation"
top_k: 1
```

### 4. Test Q&A
```bash
skill: "rag-answerer"
question: "What is this documentation about?"
collection_name: "my_documentation"
answer_length: "short"
```

## Troubleshooting

### Common Issues

1. **Connection Errors**
   - Verify QDRANT_URL is correct
   - Check network connectivity to Qdrant
   - Ensure API key is valid if required

2. **Embedding Errors**
   - Verify API key for embedding provider
   - Check rate limits on API provider
   - Ensure embedding model name is valid

3. **No Results Found**
   - Verify collection has been indexed
   - Check similarity threshold settings
   - Confirm query format and filters

4. **Performance Issues**
   - Reduce batch size during indexing
   - Optimize chunk size for your content
   - Consider collection partitioning for large datasets

### Verification Commands

Check collection exists and has data:
```bash
skill: "rag-manager"
operation: "stats"
collection_name: "my_documentation"
```

Verify search functionality:
```bash
skill: "rag-retriever"
query: "*"
collection_name: "my_documentation"
top_k: 1
```

## Advanced Configuration

### Custom Embedding Models
To use local embedding models:
```bash
# In .env file
EMBEDDING_PROVIDER=local
LOCAL_EMBEDDING_MODEL=all-MiniLM-L6-v2
```

### Multiple Collections
Create separate collections for different documentation sets:
- `api_docs` - For API reference documentation
- `tutorials` - For step-by-step guides
- `concepts` - For conceptual documentation

### Metadata Filtering
Leverage frontmatter in markdown files:
```yaml
---
title: My Document
tags: [tutorial, beginner, react]
section: Getting Started
---
```

## Maintenance Tasks

### Regular Maintenance
- Monitor collection size and performance
- Update content as documentation changes
- Clean up outdated content periodically

### Backup Strategy
```bash
skill: "rag-manager"
operation: "export"
collection_name: "my_documentation"
export_path: "./backups/docs_backup.json"
```

### Content Updates
The rag-indexer supports incremental updates by only processing changed files, making regular updates efficient.

## Integration with Docusaurus

The skills automatically:
- Read from `docusaurus.config.js`
- Parse frontmatter metadata
- Handle versioned documentation
- Support multiple locales
- Respect sidebar organization