"""Ingest documentation into Qdrant with Gemini embeddings."""
import os
import sys
from pathlib import Path
from dotenv import load_dotenv

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

# Load environment
load_dotenv('../.env')

from src.clients.gemini_embedding_client import get_embedding
from src.clients.qdrant_client import ensure_collection_exists
from qdrant_client import QdrantClient
from qdrant_client.http import models

def read_markdown_files(docs_dir: str) -> list:
    """Read all markdown files from docs directory."""
    docs = []
    docs_path = Path(docs_dir)
    
    if not docs_path.exists():
        print(f"❌ Directory not found: {docs_dir}")
        return docs
    
    print(f"📂 Reading from: {docs_path}")
    
    for md_file in docs_path.rglob("*.md"):
        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()
                
                # Skip empty files
                if len(content.strip()) < 100:
                    continue
                
                # Extract title
                title = md_file.stem.replace('-', ' ').title()
                
                # Chunk content by paragraphs
                paragraphs = [p.strip() for p in content.split('\n\n') if p.strip() and len(p.strip()) > 100]
                
                for i, para in enumerate(paragraphs[:10]):  # Max 10 chunks per file
                    docs.append({
                        'text': para,
                        'title': f"{title} - Section {i+1}",
                        'file': str(md_file.name),
                        'url': f'/docs/{md_file.stem}',
                        'source': 'frontend_docs'
                    })
                    
        except Exception as e:
            print(f"⚠️  Error reading {md_file.name}: {e}")
    
    return docs

def ingest_documents():
    """Ingest local documents into Qdrant."""
    print("\n" + "="*60)
    print("📚 DOCUMENT INGESTION - Physical AI Documentation")
    print("="*60 + "\n")
    
    # Initialize Qdrant
    qdrant_url = os.getenv('QDRANT_URL', 'http://localhost:6333')
    qdrant_key = os.getenv('QDRANT_API_KEY')
    collection_name = os.getenv('QDRANT_COLLECTION', 'physical_ai_docs')
    
    print(f"🔗 Qdrant URL: {qdrant_url}")
    print(f"📦 Collection: {collection_name}\n")
    
    qdrant = QdrantClient(url=qdrant_url, api_key=qdrant_key)
    
    # Read docs
    frontend_docs = '../frontend/docs'
    docs = read_markdown_files(frontend_docs)
    
    print(f"\n✓ Found {len(docs)} text chunks from documentation\n")
    
    if not docs:
        print("❌ No documents found to ingest!")
        return
    
    # Ensure collection exists
    print(f"🔧 Setting up collection '{collection_name}'...")
    ensure_collection_exists(collection_name, vector_size=768)
    
    # Upload documents with embeddings
    print("\n📤 Generating embeddings and uploading documents...")
    print("   (This may take a few minutes...)\n")
    
    batch_size = 5  # Small batches to avoid API rate limits
    success = 0
    failed = 0
    
    for i in range(0, len(docs), batch_size):
        batch = docs[i:i + batch_size]
        points = []
        
        for idx, doc in enumerate(batch):
            try:
                point_id = i + idx + 1
                
                # Generate embedding using Gemini
                print(f"  [{point_id}/{len(docs)}] Embedding: {doc['title'][:50]}...")
                embedding = get_embedding(doc['text'])
                
                points.append(models.PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        'text': doc['text'],
                        'title': doc['title'],
                        'file': doc['file'],
                        'url': doc['url'],
                        'source': doc['source']
                    }
                ))
                
            except Exception as e:
                print(f"  ⚠️  Error creating embedding: {e}")
                failed += 1
                continue
        
        if points:
            try:
                qdrant.upsert(
                    collection_name=collection_name,
                    points=points
                )
                success += len(points)
                print(f"  ✓ Uploaded batch {i//batch_size + 1}: {len(points)} documents\n")
            except Exception as e:
                print(f"  ❌ Error uploading batch: {e}\n")
                failed += len(points)
    
    print("\n" + "="*60)
    print("📊 INGESTION SUMMARY")
    print("="*60)
    print(f"✅ Successfully ingested: {success} documents")
    print(f"❌ Failed: {failed} documents")
    print(f"📦 Collection: {collection_name}")
    print(f"🔍 Documents are now searchable in the RAG system!")
    print("="*60 + "\n")

if __name__ == '__main__':
    try:
        ingest_documents()
    except KeyboardInterrupt:
        print("\n\n⚠️  Ingestion cancelled by user")
    except Exception as e:
        print(f"\n\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
