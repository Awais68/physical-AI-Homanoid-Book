#!/usr/bin/env python3
"""Test Qdrant connection with detailed diagnostics."""

import os
import sys
from dotenv import load_dotenv

# Load environment
load_dotenv('.env')

QDRANT_URL = os.getenv('QDRANT_URL')
QDRANT_API_KEY = os.getenv('QDRANT_API_KEY')
QDRANT_COLLECTION = os.getenv('QDRANT_COLLECTION', 'rag_chatbot')

print("=" * 60)
print("QDRANT CONNECTION DIAGNOSTICS")
print("=" * 60)

print(f"\n📍 Configuration:")
print(f"   URL: {QDRANT_URL}")
print(f"   Collection: {QDRANT_COLLECTION}")
print(f"   API Key: {'✓ Present' if QDRANT_API_KEY else '✗ Missing'}")

# Test 1: Basic connectivity
print(f"\n🔌 Test 1: Basic HTTP connectivity")
try:
    import requests
    response = requests.get(QDRANT_URL, timeout=5)
    print(f"   ✓ HTTP reachable (status: {response.status_code})")
except requests.exceptions.Timeout:
    print(f"   ✗ Connection timeout - server not reachable")
    sys.exit(1)
except requests.exceptions.ConnectionError as e:
    print(f"   ✗ Connection refused - {e}")
    sys.exit(1)
except Exception as e:
    print(f"   ⚠ Unexpected error: {e}")

# Test 2: Qdrant client connection
print(f"\n🔗 Test 2: Qdrant client connection")
try:
    from qdrant_client import QdrantClient
    
    client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
        timeout=10.0
    )
    
    # Get collections
    collections = client.get_collections()
    print(f"   ✓ Connected successfully")
    print(f"   Collections found: {len(collections.collections)}")
    
    for col in collections.collections:
        print(f"      - {col.name}")
    
except Exception as e:
    print(f"   ✗ Connection failed: {e}")
    print(f"\n💡 Troubleshooting:")
    print(f"   1. Check if Qdrant Cloud cluster is running")
    print(f"   2. Verify API key is correct")
    print(f"   3. Check if IP whitelist blocks Hugging Face")
    print(f"   4. Try accessing from different network")
    sys.exit(1)

# Test 3: Collection access
print(f"\n📦 Test 3: Collection '{QDRANT_COLLECTION}' access")
try:
    collection_info = client.get_collection(QDRANT_COLLECTION)
    print(f"   ✓ Collection accessible")
    print(f"   Points count: {collection_info.points_count}")
    print(f"   Status: {collection_info.status}")
    
except Exception as e:
    print(f"   ✗ Collection access failed: {e}")
    print(f"\n💡 Suggestion: Run document ingestion script")
    sys.exit(1)

# Test 4: Search test
print(f"\n🔍 Test 4: Sample search")
try:
    # Create a simple test vector
    test_vector = [0.1] * 1024  # Cohere embedding size
    
    results = client.query_points(
        collection_name=QDRANT_COLLECTION,
        query=test_vector,
        limit=3,
    )
    
    print(f"   ✓ Search successful")
    print(f"   Results found: {len(results.points)}")
    
    if results.points:
        print(f"   Top result score: {results.points[0].score:.3f}")
        
except Exception as e:
    print(f"   ✗ Search failed: {e}")
    sys.exit(1)

print("\n" + "=" * 60)
print("✅ ALL TESTS PASSED - Qdrant is working correctly!")
print("=" * 60)
print("\n💡 If errors occur on Hugging Face but not locally:")
print("   → Network firewall between HF and Qdrant Cloud")
print("   → Consider deploying backend to Railway/Render instead")
print("")
