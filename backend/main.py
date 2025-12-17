import requests
import xml.etree.ElementTree as ET
import trafilatura
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import sys
import time
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv(os.path.join(os.path.dirname(__file__), '..', '.env'))

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")


# Force unbuffered output
sys.stdout.reconfigure(line_buffering=True)

# -------------------------------------
# CONFIG
# -------------------------------------
# Your Deployment Link:
SITEMAP_URL = "https://awais68.github.io/physical-AI-Homanoid-Book/sitemap.xml"
COLLECTION_NAME = "physical_ai_docs"

# Use environment variables or fallback to hardcoded values (not recommended for production)
if not COHERE_API_KEY:
    COHERE_API_KEY = "your_cohere_api_key_here"  # Replace with actual key
EMBED_MODEL = "embed-english-v3.0"

# Connect to Qdrant Cloud
QDRANT_URL = "https://d781f662-2044-4110-a0aa-9f08549ea800.us-east4-0.gcp.cloud.qdrant.io"
if not QDRANT_API_KEY:
    QDRANT_API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.BLhwxVqLVogzWmFWN3d6REjaZTaVmA1i7EniMI_82xM"

qdrant = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
    timeout=60,  # Increase timeout
)

# -------------------------------------
# Step 1 — Extract URLs from sitemap
# -------------------------------------
def get_all_urls(sitemap_url):
    xml = requests.get(sitemap_url).text
    root = ET.fromstring(xml)

    urls = []
    for child in root:
        loc_tag = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
        if loc_tag is not None:
            urls.append(loc_tag.text)

    print("\nFOUND URLS:")
    for u in urls:
        print(" -", u)

    return urls


# -------------------------------------
# Step 2 — Download page + extract text
# -------------------------------------
def extract_text_from_url(url):
    try:
        print(f"  Fetching {url}...")
        sys.stdout.flush()
        html = requests.get(url, timeout=30, allow_redirects=True).text
        print(f"  Got HTML: {len(html)} bytes")
        sys.stdout.flush()
        text = trafilatura.extract(html)
        print(f"  Extracted: {len(text) if text else 0} chars")
        sys.stdout.flush()

        if not text:
            print("[WARNING] No text extracted from:", url)

        return text
    except requests.exceptions.Timeout:
        print(f"[ERROR] Timeout fetching {url}")
        return None
    except Exception as e:
        print(f"[ERROR] Failed to fetch {url}: {e}")
        return None


# -------------------------------------
# Step 3 — Chunk the text
# -------------------------------------
def chunk_text(text, max_chars=1200):
    chunks = []
    while len(text) > max_chars:
        split_pos = text[:max_chars].rfind(". ")
        if split_pos == -1:
            # No sentence break found, just split at max_chars
            split_pos = max_chars
            chunks.append(text[:split_pos])
            text = text[split_pos:]
        else:
            # Include the period in this chunk, start next chunk after ". "
            chunks.append(text[:split_pos + 1])  # Include the period
            text = text[split_pos + 2:]  # Skip past ". "
    if text:  # Add remaining text if any
        chunks.append(text)
    return chunks


# -------------------------------------
# Step 4 — Create embedding (using requests directly)
# -------------------------------------
def embed(text):
    max_retries = 3
    for attempt in range(max_retries):
        try:
            response = requests.post(
                "https://api.cohere.ai/v1/embed",
                headers={
                    "Authorization": f"Bearer {COHERE_API_KEY}",
                    "Content-Type": "application/json"
                },
                json={
                    "model": EMBED_MODEL,
                    "input_type": "search_document",
                    "texts": [text],
                },
                timeout=60
            )
            response.raise_for_status()
            return response.json()["embeddings"][0]
        except Exception as e:
            print(f"[ERROR] Embedding failed (attempt {attempt + 1}/{max_retries}): {e}")
            if attempt < max_retries - 1:
                time.sleep(2 ** attempt)  # Exponential backoff
            else:
                raise


# -------------------------------------
# Step 5 — Store in Qdrant
# -------------------------------------
def collection_has_data() -> bool:
    """Check if collection exists and has data."""
    try:
        # First check if collection exists
        collections = qdrant.get_collections().collections
        collection_names = [c.name for c in collections]
        if COLLECTION_NAME not in collection_names:
            return False
        
        # Collection exists, now try to count points using scroll (more reliable)
        scroll_result = qdrant.scroll(
            collection_name=COLLECTION_NAME,
            limit=1,  # Just need to know if any points exist
            with_payload=False,
            with_vectors=False
        )
        # scroll_result is a tuple: (points, next_page_offset)
        points = scroll_result[0] if scroll_result else []
        return len(points) > 0
        
    except Exception as e:
        # If there's any error, assume no data exists
        return False


def create_collection(force_recreate: bool = False):
    print("\nChecking Qdrant collection...")

    # Check if collection already has data
    if not force_recreate and collection_has_data():
        # Get actual count for display
        scroll_result = qdrant.scroll(
            collection_name=COLLECTION_NAME,
            limit=1,
            with_payload=False,
            with_vectors=False
        )
        print(f"\n✅ Collection '{COLLECTION_NAME}' already exists with data.")
        print("📚 Data is already saved. Retrieving from Qdrant...")
        print("💡 Tip: Use --force or -f flag to re-index all data")
        return False  # Signal that we should skip ingestion

    # Delete collection if it exists (only when force_recreate is True or no data)
    try:
        collections = qdrant.get_collections().collections
        collection_names = [c.name for c in collections]
        if COLLECTION_NAME in collection_names:
            print(f"🗑️  Collection '{COLLECTION_NAME}' exists, deleting...")
            qdrant.delete_collection(collection_name=COLLECTION_NAME)
    except Exception as e:
        pass  # Silently continue if there's an issue

    # Create new collection
    print(f"🔨 Creating new collection '{COLLECTION_NAME}'...")
    qdrant.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
            size=1024,        # Cohere embed-english-v3.0 dimension
            distance=Distance.COSINE
        )
    )
    print(f"✅ Collection '{COLLECTION_NAME}' created successfully!")
    return True  # Signal that we should proceed with ingestion

def save_chunk_to_qdrant(chunk, chunk_id, url):
    try:
        vector = embed(chunk)

        qdrant.upsert(
            collection_name=COLLECTION_NAME,
            points=[
                PointStruct(
                    id=chunk_id,
                    vector=vector,
                    payload={
                        "url": url,
                        "text": chunk,
                        "chunk_id": chunk_id
                    }
                )
            ]
        )
        return True
    except Exception as e:
        print(f"[ERROR] Failed to save chunk {chunk_id}: {e}")
        return False


# -------------------------------------
# MAIN INGESTION PIPELINE
# -------------------------------------
def ingest_book(force_recreate: bool = False):
    """Ingest book content into Qdrant.

    Args:
        force_recreate: If True, re-embed even if data exists.
    """
    urls = get_all_urls(SITEMAP_URL)

    # Check if we should proceed with ingestion
    should_ingest = create_collection(force_recreate=force_recreate)
    if not should_ingest:
        print("\n" + "="*60)
        print("✔️  SKIPPING INGESTION - Data already exists in Qdrant!")
        print("="*60)
        print("\n📊 Your data is ready to use. No need to save again.")
        print("🔍 You can now query the collection for search.")
        if not force_recreate:
            print("\n💡 To force re-indexing, run: python main.py --force")
        return

    global_id = 1
    failed_chunks = 0
    skipped_urls = 0

    for url in urls:
        print("\nProcessing:", url)
        sys.stdout.flush()

        try:
            text = extract_text_from_url(url)

            if not text:
                skipped_urls += 1
                continue

            print(f"  Chunking text...")
            sys.stdout.flush()
            chunks = chunk_text(text)
            print(f"  Found {len(chunks)} chunks")
            sys.stdout.flush()

            for i, ch in enumerate(chunks):
                if save_chunk_to_qdrant(ch, global_id, url):
                    print(f"  Saved chunk {global_id} ({i+1}/{len(chunks)})")
                    sys.stdout.flush()
                else:
                    failed_chunks += 1
                global_id += 1

                # Small delay to avoid rate limiting
                time.sleep(0.3)

        except Exception as e:
            print(f"[ERROR] Failed to process {url}: {e}")
            skipped_urls += 1

    print("\n✔️ Ingestion completed!")
    print(f"Total chunks stored: {global_id - 1 - failed_chunks}")
    print(f"Failed chunks: {failed_chunks}")
    print(f"Skipped URLs: {skipped_urls}")


if __name__ == "__main__":
    import sys
    force = "--force" in sys.argv or "-f" in sys.argv
    
    print("\n" + "="*60)
    print("🚀 Physical AI Documentation Ingestion")
    print("="*60)
    
    if force:
        print("\n⚠️  FORCE MODE ENABLED - Will delete and re-embed all data")
    else:
        print("\n🔍 Checking for existing data...")
    
    ingest_book(force_recreate=force)