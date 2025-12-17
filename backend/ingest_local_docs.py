"""Ingest local documentation without external API calls."""
import os
import hashlib
from pathlib import Path
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv

# Load environment
load_dotenv('../.env')

# Initialize Qdrant
qdrant = QdrantClient(
    url=os.getenv('QDRANT_URL'),
    api_key=os.getenv('QDRANT_API_KEY')
)

def simple_embedding(text: str, dim: int = 1024) -> list:
    """Create a simple deterministic embedding from text hash.
    This is NOT a real embedding but works for basic retrieval.
    """
    # Use MD5 hash to create deterministic vector
    hash_obj = hashlib.md5(text.encode())
    hash_bytes = hash_obj.digest()
    
    # Extend hash to required dimension
    vector = []
    for i in range(dim):
        vector.append((hash_bytes[i % len(hash_bytes)] - 128) / 128.0)
    
    return vector

def read_markdown_files(docs_dir: str) -> list:
    """Read all markdown files from docs directory."""
    docs = []
    docs_path = Path(docs_dir)
    
    if not docs_path.exists():
        print(f"Directory not found: {docs_dir}")
        return docs
    
    for md_file in docs_path.rglob("*.md"):
        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()
                
                # Extract title from filename or first heading
                title = md_file.stem.replace('-', ' ').title()
                
                # Chunk content (simple split by paragraphs)
                paragraphs = [p.strip() for p in content.split('\n\n') if p.strip() and len(p.strip()) > 50]
                
                for i, para in enumerate(paragraphs[:5]):  # Max 5 chunks per file
                    docs.append({
                        'text': para,
                        'title': f"{title} - Part {i+1}",
                        'file': str(md_file.relative_to(docs_path)),
                        'url': f'docs/{md_file.stem}'
                    })
                    
        except Exception as e:
            print(f"Error reading {md_file}: {e}")
    
    return docs

def ingest_documents():
    """Ingest local documents into Qdrant."""
    print("📚 Starting local document ingestion...")
    
    # Read docs from frontend/docs folder
    frontend_docs = '../frontend/docs'
    docs = read_markdown_files(frontend_docs)
    
    print(f"✓ Found {len(docs)} text chunks")
    
    if not docs:
        print("❌ No documents found!")
        return
    
    # Ensure collection exists
    try:
        collections = qdrant.get_collections().collections
        if 'local_docs' not in [c.name for c in collections]:
            print("Creating collection 'local_docs'...")
            qdrant.create_collection(
                collection_name='local_docs',
                vectors_config=models.VectorParams(
                    size=1024,
                    distance=models.Distance.COSINE
                )
            )
    except Exception as e:
        print(f"Collection setup: {e}")
    
    # Upload documents
    print("\n📤 Uploading documents...")
    batch_size = 10
    success = 0
    
    for i in range(0, len(docs), batch_size):
        batch = docs[i:i + batch_size]
        points = []
        
        for idx, doc in enumerate(batch):
            point_id = i + idx + 1
            embedding = simple_embedding(doc['text'])
            
            points.append(models.PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    'text': doc['text'],
                    'title': doc['title'],
                    'file': doc['file'],
                    'url': doc['url']
                }
            ))
        
        try:
            qdrant.upsert(
                collection_name='local_docs',
                points=points
            )
            success += len(points)
            print(f"  Uploaded batch {i//batch_size + 1}: {len(points)} docs (Total: {success})")
        except Exception as e:
            print(f"  Error uploading batch: {e}")
    
    print(f"\n✅ Ingestion complete! Total documents: {success}")
    print(f"📊 Collection: local_docs")
    print(f"🔍 Note: Using simple hash-based embeddings (no API required)")

if __name__ == '__main__':
    ingest_documents()
