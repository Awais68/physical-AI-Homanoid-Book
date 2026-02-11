"""Test chat API endpoint"""
import sys
sys.path.insert(0, '/media/data/physical-AI-Homanoid-Book-main/backend')

from src.rag.engine import RAGEngine
import asyncio

async def test_chat():
    engine = RAGEngine()
    await engine.initialize()
    
    result = await engine.query(
        question="what is ai",
        conversation_history=None,
        selected_text=None,
        include_citations=True
    )
    
    print("SUCCESS!")
    print(f"Answer: {result['answer'][:200]}...")
    print(f"Sources: {len(result['sources'])}")

if __name__ == "__main__":
    asyncio.run(test_chat())
