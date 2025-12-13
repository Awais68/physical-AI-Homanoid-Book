# rag-manager

## Purpose
Manage Qdrant collections and configurations including creation, deletion, monitoring, and maintenance operations.

## Capabilities
- Create new Qdrant collections with proper schema
- Delete Qdrant collections and associated data
- Update collection settings (vector size, distance metric, indexing)
- View collection statistics (count, size, schema, health)
- Export/import collection data for backup/migration
- Health check and diagnostic operations
- Clear stale or outdated chunks based on metadata
- List all available collections
- Modify collection vectors configuration
- Manage collection aliases

## Tools Available
- Bash: Interact with Qdrant API for management operations

## Input Parameters
- `operation`: Operation type (create, delete, update, list, stats, health, export, import, clear_stale)
- `collection_name`: Target collection name (required for most operations)
- `vector_size`: Size of embedding vectors (default: 1536 for OpenAI)
- `distance_metric`: Distance metric to use (cosine, euclidean, dot) (default: cosine)
- `payload_indexing`: Enable payload indexing for filtering (default: true)
- `export_path`: Path for export operations
- `import_path`: Path for import operations
- `stale_filter`: Filter criteria for clearing stale chunks (JSON)
- `settings`: Additional collection settings as JSON

## Output
- Operation status (success, failure, partial)
- Detailed results of the operation
- Collection statistics when applicable
- Error details if operation fails
- Progress tracking for long operations

## Configuration Requirements
Environment variables needed:
- `QDRANT_URL`: Qdrant server URL (default: http://localhost:6333)
- `QDRANT_API_KEY`: Qdrant API key (optional for local)

## Example Usage
```
skill: "rag-manager"
operation: "create"
collection_name: "new_book_content"
vector_size: 1536
distance_metric: "cosine"
payload_indexing: true
```

## Technical Implementation Details
### Create Operation
- Validate collection doesn't already exist
- Create collection with specified vector size and distance metric
- Configure payload indexing for metadata filtering
- Set up proper shard configuration for performance

### Delete Operation
- Verify collection exists before deletion
- Confirm deletion with user for safety
- Remove collection and all associated data
- Clean up any references to the collection

### Update Operation
- Modify collection settings while preserving data
- Update vector configuration if needed (with data migration)
- Adjust indexing settings
- Reload collection configuration

### Stats Operation
- Return point count in collection
- Provide storage size information
- Show vector configuration details
- Display payload schema information
- List indexed fields

### Health Operation
- Check Qdrant server connectivity
- Verify collection integrity
- Test search functionality
- Report any issues found

### Export/Import Operations
- Export collection data in Qdrant native format
- Import data from exported files
- Support for incremental imports
- Validation of imported data integrity

### Clear Stale Operation
- Identify chunks based on metadata filters
- Remove outdated or irrelevant content
- Support for date-based cleanup
- Dry-run capability to preview deletions

## Collection Schema Requirements
- Vector field for embeddings (size varies by model)
- Payload fields for metadata:
  - file_path: Original file location
  - title: Document title
  - section: Section/chapter information
  - tags: Array of tags
  - created_at: Timestamp
  - updated_at: Last modification timestamp
  - version: Document version if applicable

## Distance Metrics
- **Cosine**: Best for semantic similarity (recommended default)
- **Euclidean**: Good for geometric distance
- **Dot**: Suitable for certain embedding types

## Error Handling
- Handle connection errors to Qdrant
- Validate operation parameters before execution
- Prevent accidental deletion without confirmation
- Provide clear error messages for invalid operations
- Graceful handling of concurrent operations

## Performance Considerations
- Efficient bulk operations for large collections
- Proper indexing for metadata fields
- Monitoring of resource usage during operations
- Progress reporting for long-running tasks
- Batch processing where applicable