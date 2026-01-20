from pydantic import BaseModel
from typing import Optional, Dict
from datetime import datetime
import uuid


class TextbookContentBase(BaseModel):
    chapter_id: str
    section_title: str
    content_text: str
    page_reference: str
    embedding_vector: Optional[Dict] = None


class TextbookContentCreate(TextbookContentBase):
    pass


class TextbookContent(TextbookContentBase):
    id: str
    created_at: datetime

    class Config:
        from_attributes = True