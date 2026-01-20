from pydantic import BaseModel
from typing import Dict, Optional
from datetime import datetime


class SourceURLBase(BaseModel):
    url: str
    content_length: Optional[int] = 0
    processing_status: str = "queued"  # queued, processing, success, error, skipped
    error_message: Optional[str] = None
    metadata: Optional[Dict] = {}


class SourceURLCreate(SourceURLBase):
    pass


class SourceURL(SourceURLBase):
    id: str
    status_code: Optional[int] = None
    extraction_timestamp: Optional[datetime] = None

    class Config:
        from_attributes = True