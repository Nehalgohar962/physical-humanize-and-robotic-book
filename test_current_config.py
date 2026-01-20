import sys
import os

# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

from backend.src.config import settings

print("Current configuration:")
print(f"LLM Provider: {settings.llm_provider}")
print(f"Gemini API Key (first 10 chars): {settings.gemini_api_key[:10] if settings.gemini_api_key else 'None'}")
print(f"OpenAI API Key (first 10 chars): {settings.openai_api_key[:10] if settings.openai_api_key else 'None'}")
print(f"Cohere API Key (first 10 chars): {settings.cohere_api_key[:10] if settings.cohere_api_key else 'None'}")
print(f"Qdrant URL: {settings.qdrant_url}")