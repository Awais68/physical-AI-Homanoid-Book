"""Document storage and retrieval for RAG system."""
from typing import Optional
import uuid
import hashlib
from src.clients.cohere_client import get_document_embedding
from src.clients.qdrant_client import upsert_documents, search_similar, get_collection_info
from src.config.settings import settings


class DocumentStore:
    """Manages document storage and retrieval for RAG."""

    def __init__(self, collection_name: Optional[str] = None):
        """Initialize document store.

        Args:
            collection_name: Qdrant collection name.
        """
        self.collection_name = collection_name or settings.QDRANT_COLLECTION
        self._embedding_cache = {}
        
    def _check_document_exists(self, content: str) -> Optional[str]:
        """Check if document with same content already exists.
        
        Args:
            content: Document content to check.
            
        Returns:
            Existing document ID if found, None otherwise.
        """
        content_hash = hashlib.md5(content.encode()).hexdigest()
        
        # Check in cache first
        if content_hash in self._embedding_cache:
            print(f"✓ Document already exists (cached) - skipping Cohere API call")
            return self._embedding_cache[content_hash]
        
        return None

    async def add_document(
        self,
        content: str,
        title: Optional[str] = None,
        source_url: Optional[str] = None,
        file_path: Optional[str] = None,
        section: Optional[str] = None,
        tags: Optional[list] = None,
        metadata: Optional[dict] = None,
    ) -> str:
        """Add a document to the store.

        Args:
            content: Document content text.
            title: Document title.
            source_url: URL of the source.
            file_path: File path of the source.
            section: Section name within the document.
            tags: List of tags.
            metadata: Additional metadata.

        Returns:
            Document ID.
        """
        # Check if document already exists
        existing_id = self._check_document_exists(content)
        if existing_id:
            return existing_id
        
        doc_id = str(uuid.uuid4())

        # Generate embedding only for new documents
        print(f"→ Generating embedding for new document: {title or 'Untitled'}")
        embedding = get_document_embedding(content)
        
        # Cache the document
        content_hash = hashlib.md5(content.encode()).hexdigest()
        self._embedding_cache[content_hash] = doc_id

        # Create payload
        payload = {
            "content": content,
            "title": title or "",
            "source_url": source_url or "",
            "file_path": file_path or "",
            "section": section or "",
            "tags": tags or [],
            "metadata": metadata or {},
            "content_hash": content_hash,  # Store hash for deduplication
        }

        # Upsert to Qdrant
        upsert_documents(
            collection_name=self.collection_name,
            documents=[{
                "id": doc_id,
                "vector": embedding,
                "payload": payload,
            }],
        )
        
        print(f"✓ Document added successfully: {doc_id}")
        return doc_id

    async def add_documents_batch(
        self,
        documents: list[dict],
    ) -> list[str]:
        """Add multiple documents in batch.

        Args:
            documents: List of document dicts with content and metadata.

        Returns:
            List of document IDs.
        """
        doc_ids = []
        prepared_docs = []

        for doc in documents:
            doc_id = str(uuid.uuid4())
            doc_ids.append(doc_id)

            content = doc.get("content", "")
            embedding = await get_embedding(content)

            prepared_docs.append({
                "id": doc_id,
                "vector": embedding,
                "payload": {
                    "content": content,
                    "title": doc.get("title", ""),
                    "source_url": doc.get("source_url", ""),
                    "file_path": doc.get("file_path", ""),
                    "section": doc.get("section", ""),
                    "tags": doc.get("tags", []),
                    "metadata": doc.get("metadata", {}),
                },
            })

        await upsert_documents(
            collection_name=self.collection_name,
            documents=prepared_docs,
        )

        return doc_ids

    async def search(
        self,
        query: str,
        top_k: int = 5,
        score_threshold: float = 0.4,
    ) -> list[dict]:
        """Search for documents similar to query.

        Args:
            query: Search query text.
            top_k: Number of results.
            score_threshold: Minimum similarity score.

        Returns:
            List of matching documents.
        """
        query_embedding = await get_embedding(query)

        return await search_similar(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            top_k=top_k,
            score_threshold=score_threshold,
        )
