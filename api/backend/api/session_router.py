import sys
import os
from fastapi import APIRouter, HTTPException
from typing import Dict

# Add the backend directory to the Python path to allow absolute imports
current_dir = os.path.dirname(os.path.abspath(__file__))
backend_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, backend_dir)

from models.chat_session import ChatSession, ChatSessionCreate
from services.session_service import SessionService

router = APIRouter()

@router.post("/sessions", response_model=ChatSession)
async def create_session(request: ChatSessionCreate = None):
    try:
        user_id = request.user_id if request else None
        session = SessionService.create_session(user_id)
        return session
    except Exception as e:
        import traceback
        print(f"Error in create session endpoint: {e}")
        print(f"Full traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.get("/sessions/{session_id}", response_model=ChatSession)
async def get_session(session_id: str):
    try:
        session = SessionService.get_session(session_id)
        if session is None:
            raise HTTPException(status_code=404, detail="Session not found")
        return session
    except HTTPException:
        raise
    except Exception as e:
        import traceback
        print(f"Error in get session endpoint: {e}")
        print(f"Full traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.get("/sessions/{session_id}/messages")
async def get_session_messages(session_id: str):
    try:
        session = SessionService.get_session(session_id)
        if session is None:
            raise HTTPException(status_code=404, detail="Session not found")

        return SessionService.get_session_messages(session_id)
    except HTTPException:
        raise
    except Exception as e:
        import traceback
        print(f"Error in get session messages endpoint: {e}")
        print(f"Full traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.put("/sessions/{session_id}/touch")
async def update_session_activity(session_id: str):
    try:
        SessionService.update_session_activity(session_id)
        return {"status": "updated"}
    except Exception as e:
        import traceback
        print(f"Error in update session endpoint: {e}")
        print(f"Full traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")