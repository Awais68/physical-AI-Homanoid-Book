# RAG Skills Test Script

This script demonstrates how the RAG skills work together in a typical workflow.

## Scenario: Setting up RAG for a Docusaurus documentation site

### Step 1: Initialize Qdrant Collection
```
skill: "rag-manager"
operation: "create"
collection_name: "docusaurus_docs"
vector_size: 1536
distance_metric: "cosine"
payload_indexing: true
```

### Step 2: Index Documentation Content
```
skill: "rag-indexer"
docs_path: "./docs"
collection_name: "docusaurus_docs"
chunk_size: 500
chunk_overlap: 50
embedding_model: "openai-text-embedding-3-small"
batch_size: 10
```

### Step 3: Search for Relevant Information
```
skill: "rag-retriever"
query: "How to customize Docusaurus theme?"
collection_name: "docusaurus_docs"
top_k: 3
similarity_threshold: 0.4
filters: {"tags": ["styling", "theme"]}
```

### Step 4: Get Answer with Citations
```
skill: "rag-answerer"
question: "What are the recommended ways to customize Docusaurus theme colors?"
collection_name: "docusaurus_docs"
answer_length: "medium"
num_sources: 3
include_citations: true
similarity_threshold: 0.5
```

### Step 5: Monitor Collection Health
```
skill: "rag-manager"
operation: "stats"
collection_name: "docusaurus_docs"
```

## Expected Workflow Integration

1. **Setup Phase**: Use rag-manager to create collections
2. **Ingestion Phase**: Use rag-indexer to populate Qdrant
3. **Query Phase**: Use rag-retriever and rag-answerer for search/Q&A
4. **Maintenance Phase**: Use rag-manager for ongoing operations

## Error Handling Examples

If no relevant content is found:
- rag-retriever returns empty results
- rag-answerer responds with "I don't know" and confidence score

If Qdrant is unavailable:
- All skills return connection error messages
- Graceful degradation to alternative approaches

## Performance Considerations

- Batch operations during indexing for efficiency
- Caching of frequent queries where appropriate
- Proper similarity thresholds to balance precision/recall
- Monitoring of API usage and costs