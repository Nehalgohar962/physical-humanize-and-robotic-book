from typing import Dict, List
from models.chat_session import ChatSession, Message
from uuid import uuid4
from datetime import datetime

# In-memory storage for sessions (in production, use database)
sessions: Dict[str, ChatSession] = {}
messages: Dict[str, List[Message]] = {}

class SessionService:
    @staticmethod
    def create_session(user_id: str = None) -> ChatSession:
        session_id = str(uuid4())
        session = ChatSession(
            id=session_id,
            user_id=user_id,
            created_at=datetime.utcnow(),
            last_activity_at=datetime.utcnow(),
            metadata={}
        )
        sessions[session_id] = session
        messages[session_id] = []  # Initialize empty message list
        return session

    @staticmethod
    def get_session(session_id: str) -> ChatSession:
        if session_id not in sessions:
            return None
        return sessions[session_id]

    @staticmethod
    def add_messages(session_id: str, user_message: Message, assistant_message: Message):
        if session_id not in messages:
            messages[session_id] = []
        messages[session_id].extend([user_message, assistant_message])

    @staticmethod
    def get_session_messages(session_id: str) -> List[Message]:
        return messages.get(session_id, [])

    @staticmethod
    def update_session_activity(session_id: str):
        if session_id in sessions:
            sessions[session_id].last_activity_at = datetime.utcnow()

    @staticmethod
    def clear_session(session_id: str):
        if session_id in sessions:
            del sessions[session_id]
        if session_id in messages:
            del messages[session_id]