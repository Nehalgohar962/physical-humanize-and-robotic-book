from pydantic import BaseModel
from typing import List, Dict, Optional
from datetime import datetime


class ContentChunkBase(BaseModel):
    text_content: str
    source_url: str
    content_type: str = "paragraph"
    chunk_order: int = 0
    metadata: Optional[Dict] = {}


class ContentChunkCreate(ContentChunkBase):
    embedding_vector: List[float]


class ContentChunk(ContentChunkBase):
    id: str
    embedding_vector: List[float]

    class Config:
        from_attributes = True