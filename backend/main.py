import requests
import xml.etree.ElementTree as ET
import trafilatura
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import sys
import time
import os
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

COHERE_API_KEY = "COHERE_API_KEY"
EMBED_MODEL = "embed-english-v3.0"

# Connect to Qdrant Cloud
# Note: Use the cluster URL from your Qdrant Cloud dashboard
# QDRANT_URL = "https://d781f662-2044-4110-a0aa-9f08549ea800.us-east4-0.gcp.cloud.qdrant.io"
# QDRANT_API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.BLhwxVqLVogzWmFWN3d6REjaZTaVmA1i7EniMI_82xM"

qdrant = QdrantClient(
    url="https://d781f662-2044-4110-a0aa-9f08549ea800.us-east4-0.gcp.cloud.qdrant.io",
    api_key="QDRANT_API_KEY",
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
        if not qdrant.collection_exists(collection_name=COLLECTION_NAME):
            return False
        info = qdrant.get_collection(collection_name=COLLECTION_NAME)
        return info.points_count > 0
    except Exception as e:
        print(f"[WARNING] Error checking collection: {e}")
        return False


def create_collection(force_recreate: bool = False):
    print("\nChecking Qdrant collection...")

    # Check if collection already has data
    if not force_recreate and collection_has_data():
        info = qdrant.get_collection(collection_name=COLLECTION_NAME)
        print(f"Collection '{COLLECTION_NAME}' already has {info.points_count} points.")
        print("Skipping re-embedding. Use force_recreate=True to re-index.")
        return False  # Signal that we should skip ingestion

    # Delete collection if it exists
    if qdrant.collection_exists(collection_name=COLLECTION_NAME):
        print(f"Collection '{COLLECTION_NAME}' exists, deleting...")
        qdrant.delete_collection(collection_name=COLLECTION_NAME)

    # Create new collection
    qdrant.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
            size=1024,        # Cohere embed-english-v3.0 dimension
            distance=Distance.COSINE
        )
    )
    print(f"Collection '{COLLECTION_NAME}' created successfully!")
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
        print("\n✔️ Data already exists in Qdrant. Skipping ingestion.")
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
    if force:
        print("Force mode enabled - will re-embed all data")
    ingest_book(force_recreate=force)