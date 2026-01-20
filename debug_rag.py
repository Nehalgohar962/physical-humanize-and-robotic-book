import sys
import os
import requests

# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

from backend.src.services.rag_service import RAGService

print("Debugging RAG Service...")

try:
    # Initialize the RAG service
    rag_service = RAGService()
    print("[SUCCESS] RAG Service initialized successfully")

    # Test embedding generation
    test_query = "What is Physical AI?"
    print(f"Testing embedding for query: '{test_query}'")

    embedding = rag_service.embedding_service.embed(test_query)
    print(f"[SUCCESS] Embedding generated successfully: {len(embedding)} dimensions")

    # Test search functionality
    print("Testing search functionality...")
    results = rag_service.textbook_service.search_content(test_query, limit=3)
    print(f"[SUCCESS] Search completed. Found {len(results)} results")

    # Test LLM client initialization (OpenAI or Gemini)
    print("Testing LLM client...")
    from backend.src.config import settings
    if settings.llm_provider.lower() == "gemini":
        response = rag_service.client.generate_content(
            "Hello, test message.",
            generation_config={
                "max_output_tokens": 10,
                "temperature": 0.3,
            }
        )
        if hasattr(response, 'text') and response.text:
            print("[SUCCESS] Google Gemini API call successful")
            print(f"Response: {response.text[:50]}...")
        else:
            print("[ERROR] Google Gemini API returned empty response")
    elif settings.llm_provider.lower() == "openai":
        response = rag_service.client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": "Hello, test message."}
            ],
            max_tokens=10,
            temperature=0.3
        )
        print("[SUCCESS] OpenAI API call successful")
        print(f"Response: {response.choices[0].message.content[:50]}...")

    print("\n[SUCCESS] All components working correctly!")
    print("The issue might be elsewhere in the request flow.")

except Exception as e:
    print(f"[ERROR] Error in RAG service: {e}")
    import traceback
    traceback.print_exc()