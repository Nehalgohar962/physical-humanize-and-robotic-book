import google.generativeai as genai
import os
from dotenv import load_dotenv

# Load .env
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
load_dotenv(dotenv_path=os.path.join(BASE_DIR, ".env"))

# Configure API key
api_key = os.getenv("GEMINI_API_KEY")
genai.configure(api_key=api_key)

# List available models
print("Available Gemini models:")
models = genai.list_models()
for m in models:
    print(m.name, "-", getattr(m, "description", ""))
