# import os
# from pathlib import Path
# from pydantic_settings import BaseSettings, SettingsConfigDict
# from typing import Optional


# class Settings(BaseSettings):
#     cohere_api_key: str
#     openai_api_key: Optional[str] = None
#     gemini_api_key: str
#     llm_provider: str = "gemini"  # Default to gemini, can be "openai" or "gemini"
#     qdrant_url: str
#     qdrant_api_key: Optional[str] = None
#     target_url: str = "https://httpbin.org/html"
#     chunk_size: int = 1000
#     chunk_overlap: int = 100
#     rate_limit_delay: float = 1.0  # seconds
#     cohere_model: str = "embed-multilingual-v2.0"

#     model_config = SettingsConfigDict(env_file=Path(__file__).parent.parent.parent / ".env")


# settings = Settings()







# backend/src/config.py

import os
from pathlib import Path
from typing import Optional
from pydantic_settings import BaseSettings
from dotenv import load_dotenv

# 1️⃣ Load .env file manually
env_path = Path(__file__).parent.parent / ".env"  # backend/.env
if env_path.exists():
    load_dotenv(dotenv_path=env_path)
else:
    print(f"WARNING: .env file not found at {env_path}")

# 2️⃣ Settings class
class Settings(BaseSettings):
    cohere_api_key: str
    openai_api_key: Optional[str] = None
    gemini_api_key: str
    llm_provider: str = "gemini"  # Default to gemini, can be "openai" or "gemini"
    qdrant_url: str
    qdrant_api_key: Optional[str] = None
    target_url: str = "https://httpbin.org/html"
    chunk_size: int = 1000
    chunk_overlap: int = 100
    rate_limit_delay: float = 1.0  # seconds
    cohere_model: str = "embed-multilingual-v2.0"

    class Config:
        env_file = env_path  # Ensure Pydantic also reads from the .env
        case_sensitive = False  # Ignore case when loading env vars

# 3️⃣ Create settings instance
settings = Settings()
