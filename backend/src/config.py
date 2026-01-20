import os
from pathlib import Path
from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import Optional


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

    model_config = SettingsConfigDict(env_file=Path(__file__).parent.parent.parent / ".env")


settings = Settings()