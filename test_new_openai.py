import sys
import os

# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

import google.generativeai as genai
from backend.src.config import settings

print("Testing Google Gemini client with new API key...")

# Configure the Gemini client with the loaded API key
genai.configure(api_key=settings.gemini_api_key)
model = genai.GenerativeModel('gemini-pro')

try:
    # Try a simple API call to test the key
    response = model.generate_content(
        "Hello, just testing the new API key.",
        generation_config={
            "max_output_tokens": 20,
            "temperature": 0.1,
        }
    )

    if response.text:
        print("SUCCESS: New Google Gemini API key is working!")
        print(f"Response: {response.text}")
    else:
        print("ERROR: Google Gemini API returned empty response")

except Exception as e:
    print(f"ERROR: New Google Gemini API key is not working - {e}")