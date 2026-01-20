import sys
import os

# Add the backend/src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

from config import Settings

# Load fresh settings
settings = Settings()

print("API Key loaded:")
print(f"Gemini API Key starts with: {settings.gemini_api_key[:10]}...")
print(f"Full length: {len(settings.gemini_api_key)} characters")
print(f"Cohere API Key starts with: {settings.cohere_api_key[:10]}...")

# Test creating a Gemini client with the key
try:
    import google.generativeai as genai
    genai.configure(api_key=settings.gemini_api_key)
    model = genai.GenerativeModel('gemini-pro')
    print("\nGoogle Gemini client created successfully")

    # Try a simple test call
    print("Testing Google Gemini connection...")
    response = model.generate_content(
        "Test",
        generation_config={
            "max_output_tokens": 5,
            "temperature": 0.0,
        }
    )
    if response.text:
        print("Google Gemini API call successful!")
        print(f"Response: {response.text}")
    else:
        print("Google Gemini API returned empty response")
except Exception as e:
    print(f"Google Gemini API call failed: {e}")