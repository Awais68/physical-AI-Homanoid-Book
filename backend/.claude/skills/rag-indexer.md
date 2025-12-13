# rag-indexer

## Purpose
Index Docusaurus book content into Qdrant vector database for semantic search and retrieval.

## Capabilities
- Scan Docusaurus docs directory for markdown files
- Extract and chunk content with smart chunking and overlap
- Generate embeddings using OpenAI, Anthropic, or local models
- Upload to Qdrant with metadata (file path, section, title)
- Handle incremental updates (only index new/changed files)
- Support batch processing with progress tracking
- Parse frontmatter for metadata extraction
- Handle versioned docs if applicable

## Tools Available
- Read: Read markdown files and configuration
- Glob: Scan for markdown files in directory
- Bash: Interact with Qdrant API and CLI
- WebFetch: Retrieve external resources if needed

## Input Parameters
- `docs_path`: Path to Docusaurus docs directory (default: ./docs)
- `collection_name`: Name of Qdrant collection (default: docusaurus_content)
- `chunk_size`: Maximum tokens per chunk (default: 500)
- `chunk_overlap`: Overlap tokens between chunks (default: 50)
- `embedding_model`: Model to use for embeddings (default: openai-text-embedding-3-small)
- `batch_size`: Number of chunks to process in parallel (default: 10)

## Output
- Indexing report with statistics:
  - Files processed count
  - Total chunks created
  - Embeddings generated
  - Errors encountered
  - Time elapsed
  - Progress tracking during operation

## Configuration Requirements
Environment variables needed:
- `QDRANT_URL`: Qdrant server URL (default: http://localhost:6333)
- `QDRANT_API_KEY`: Qdrant API key (optional for local)
- `OPENAI_API_KEY`: OpenAI API key for embeddings (if using OpenAI)
- `ANTHROPIC_API_KEY`: Anthropic API key (if using Anthropic models)

## Example Usage
```
skill: "rag-indexer"
docs_path: "./docs"
collection_name: "my_book_content"
chunk_size: 400
embedding_model: "openai-text-embedding-3-small"
```

## Technical Implementation Details
1. Scan docs directory for .md and .mdx files
2. Parse frontmatter to extract metadata (title, tags, description)
3. Split content using semantic chunking respecting markdown headers
4. Preserve code blocks as complete units
5. Generate embeddings for each chunk
6. Upload to Qdrant with metadata including:
   - file_path: Original file location
   - title: Document title from frontmatter
   - section: H1/H2 headers context
   - tags: From frontmatter
   - version: If versioned docs exist
7. Track file modification times for incremental updates
8. Provide progress feedback during batch operations

## Error Handling
- Graceful handling of API rate limits
- Retry mechanism for failed embeddings
- Continue processing if individual files fail
- Clear error messages for configuration issues
- Validation of Qdrant connection before starting

## Performance Considerations
- Batch processing for efficiency
- Asynchronous operations where possible
- Memory management for large files
- Progress tracking for long-running operations