# Data Model: Docusaurus Embeddings Generation and Vector Database Storage

## DocumentChunk
Represents a segment of documentation text that has been extracted and prepared for embedding.

**Fields:**
- id: string (unique identifier for the chunk)
- content: string (the actual text content of the chunk)
- sourceUrl: string (URL of the original documentation page)
- title: string (title of the original documentation page)
- headings: array (hierarchy of headings that provide context)
- metadata: object (additional metadata like section, tags, etc.)
- createdAt: datetime (timestamp when the chunk was created)
- processedAt: datetime (timestamp when embedding was generated)

**Validation:**
- id must be unique
- content must not be empty
- sourceUrl must be a valid URL
- createdAt must be before processedAt if processing is complete

## EmbeddingVector
Represents the vector representation of a document chunk, with attributes for the vector data and associated metadata.

**Fields:**
- id: string (unique identifier, typically matches the DocumentChunk id)
- documentChunkId: string (reference to the original DocumentChunk)
- vector: array (numerical vector representation from Cohere)
- vectorSize: integer (dimension of the vector)
- similarityScore: float (for search results, null for stored vectors)
- metadata: object (metadata to be stored with the vector in Qdrant)
- storedAt: datetime (timestamp when vector was stored in Qdrant)

**Validation:**
- id must be unique
- documentChunkId must reference an existing DocumentChunk
- vector must have the expected size (typically 1024 for Cohere embeddings)
- vectorSize must match the actual vector length

## ProcessingJob
Represents a complete processing task, with attributes for source URL, status, and processing results.

**Fields:**
- id: string (unique identifier for the processing job)
- sourceUrl: string (base URL of the Docusaurus site to process)
- status: string (pending, in-progress, completed, failed)
- totalDocuments: integer (total number of documents to process)
- processedDocuments: integer (number of documents processed)
- failedDocuments: integer (number of documents that failed)
- startTime: datetime (when the job started)
- endTime: datetime (when the job completed or failed)
- errorDetails: object (details about any errors that occurred)
- config: object (configuration parameters for this job)

**Validation:**
- id must be unique
- sourceUrl must be a valid URL
- status must be one of: pending, in-progress, completed, failed
- startTime must be before endTime if job is completed
- processedDocuments + failedDocuments must not exceed totalDocuments

## Relationships

- One ProcessingJob can generate multiple DocumentChunks
- One DocumentChunk maps to exactly one EmbeddingVector
- Multiple DocumentChunks can be part of one ProcessingJob