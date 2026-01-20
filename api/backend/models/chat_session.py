from pydantic import BaseModel
from typing import Optional, List, Dict
from datetime import datetime
import uuid


class ChatSessionBase(BaseModel):
    user_id: Optional[str] = None
    metadata: Optional[Dict] = {}


class ChatSessionCreate(ChatSessionBase):
    pass


class ChatSession(ChatSessionBase):
    id: str
    created_at: datetime
    last_activity_at: datetime

    class Config:
        from_attributes = True


class MessageBase(BaseModel):
    session_id: str
    role: str  # "user" or "assistant"
    content: str
    context_reference: Optional[str] = None
    embeddings_metadata: Optional[Dict] = {}


class MessageCreate(MessageBase):
    pass


class Message(MessageBase):
    id: str
    timestamp: datetime

    class Config:
        from_attributes = True