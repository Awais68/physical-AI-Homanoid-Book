#!/usr/bin/env python3
"""Test chat API endpoint."""
import requests
import json

def test_chat():
    """Test chat message endpoint."""
    url = "http://localhost:8000/api/chat/message"
    
    payload = {
        "message": "What is robotics?",
        "session_id": "test123"
    }
    
    print("Testing chat endpoint...")
    print(f"URL: {url}")
    print(f"Payload: {json.dumps(payload, indent=2)}\n")
    
    try:
        response = requests.post(url, json=payload, timeout=30)
        print(f"Status Code: {response.status_code}")
        
        if response.status_code == 200:
            data = response.json()
            print("\n✅ SUCCESS!\n")
            print(f"Query: {data.get('query')}")
            print(f"Confidence: {data.get('confidence')}")
            print(f"Has Sources: {data.get('has_sources')}")
            print(f"Source Count: {data.get('source_count')}")
            print(f"\nAnswer:\n{data.get('answer')}")
            
            if data.get('sources'):
                print(f"\nSources:")
                for i, source in enumerate(data.get('sources', []), 1):
                    print(f"  {i}. {source.get('title')} - Score: {source.get('score', 'N/A')}")
        else:
            print(f"\n❌ ERROR: {response.status_code}")
            print(response.text)
            
    except Exception as e:
        print(f"\n❌ Exception: {e}")

if __name__ == "__main__":
    test_chat()
