import os
import uuid
import logging
from datetime import datetime
from typing import List, Dict, Optional

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ================= SETTINGS =================
class Settings:
    cohere_api_key = os.getenv("COHERE_API_KEY")
    gemini_api_key = os.getenv("GEMINI_API_KEY")
    llm_provider = os.getenv("LLM_PROVIDER", "gemini")

settings = Settings()

# ================= MODELS =================
class Message(BaseModel):
    id: str
    role: str
    content: str
    timestamp: datetime
    references: Optional[List[str]] = []

class ChatSession(BaseModel):
    id: str
    created_at: datetime
    messages: List[Message] = []

sessions: Dict[str, ChatSession] = {}

# ================= APP =================
app = FastAPI(title="RAG Chatbot API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ================= ROUTES =================
@app.get("/")
def root():
    return {
        "status": "API running",
        "cohere_key_loaded": bool(settings.cohere_api_key),
        "gemini_key_loaded": bool(settings.gemini_api_key),
        "llm_provider": settings.llm_provider
    }

@app.post("/api/chat/session")
def create_session():
    session_id = str(uuid.uuid4())
    session = ChatSession(
        id=session_id,
        created_at=datetime.now(),
        messages=[]
    )
    sessions[session_id] = session
    return session

@app.post("/api/chat")
def chat(session_id: str, message: str):
    if session_id not in sessions:
        raise HTTPException(status_code=404, detail="Session not found")

    # 🔴 SAFE RESPONSE (NO AI CALL YET)
    reply = f"You said: {message}"

    user_msg = Message(
        id=str(uuid.uuid4()),
        role="user",
        content=message,
        timestamp=datetime.now()
    )

    bot_msg = Message(
        id=str(uuid.uuid4()),
        role="assistant",
        content=reply,
        timestamp=datetime.now()
    )

    sessions[session_id].messages.extend([user_msg, bot_msg])
    return {"reply": reply}
