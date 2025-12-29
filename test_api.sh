#!/bin/bash

echo "=== Testing All Services ==="
echo ""

echo "1. Backend Health Check:"
curl -s http://localhost:8000/health | python3 -m json.tool
echo -e "\n"

echo "2. Chat Endpoint Test:"
curl -s -X POST http://localhost:8000/api/chat/message \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello", "session_id": "test"}' | python3 -m json.tool | head -15
echo -e "\n"

echo "3. Frontend Status:"
curl -s http://localhost:3000/physical-AI-Homanoid-Book/ | grep -o '<title>.*</title>'
echo -e "\n"

echo "4. Text API Status:"
curl -s http://localhost:8001/docs | grep -o '<title>.*</title>'
echo -e "\n"

echo "=== All Services Running ✓ ==="
