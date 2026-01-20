import sys
import os
from fastapi import APIRouter, HTTPException, Depends
from typing import List, Dict, Any
from pydantic import BaseModel

# Add the backend/src directory to the Python path to allow absolute imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from services.rag_service import RAGService
from services.session_service import SessionService
from models.chat_session import ChatSession, Message, ChatSessionCreate, MessageCreate
from uuid import uuid4
from datetime import datetime

router = APIRouter()

# Initialize RAG service (will be initialized per request to ensure fresh API key)
rag_service = None  # Will be initialized in the endpoint functions


class ChatRequest(BaseModel):
    session_id: str
    message: str
    mode: str = "full_book"  # "full_book" or "context_only"
    context_text: str = None
    modules: List[int] = [1, 2, 3, 4, 5, 6]  # List of modules to search (default: all 6 modules)


class ChatResponse(BaseModel):
    response: str
    session_id: str
    references: List[str]


class CreateSessionRequest(BaseModel):
    user_id: str = None


class CreateSessionResponse(BaseModel):
    session_id: str


@router.post("/sessions", response_model=CreateSessionResponse)
async def create_session(request: CreateSessionRequest = None):
    try:
        # Use shared session service to create session
        user_id = request.user_id if request else None
        session = SessionService.create_session(user_id)

        return CreateSessionResponse(session_id=session.id)
    except Exception as e:
        print(f"Error in create session endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


# DEBUG: Added comment to trigger reload
import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, Optional

# In-memory lock dictionary to manage per-session locks
session_locks: Dict[str, asyncio.Lock] = {}
lock_dict_lock = threading.Lock()  # Lock for the lock dictionary itself

@router.post("/chat", response_model=ChatResponse)
async def chat(chat_request: ChatRequest):
    # Ensure session_id exists
    if not chat_request.session_id:
        new_session = SessionService.create_session(user_id=None)
        chat_request.session_id = new_session.id

    # Get or create a lock for this session to ensure sequential processing
    with lock_dict_lock:
        if chat_request.session_id not in session_locks:
            session_locks[chat_request.session_id] = asyncio.Lock()
        session_lock = session_locks[chat_request.session_id]

    async with session_lock:
        try:
            # Initialize RAG service for this request to ensure fresh API key
            print(f"DEBUG: Creating RAGService instance for query: {chat_request.message}")
            print(f"DEBUG: Session ID: {chat_request.session_id}")

            # Auto-create session if missing or invalid
            session = SessionService.get_session(chat_request.session_id)
            if session is None:
                print(f"DEBUG: Session not found for ID: {chat_request.session_id}, creating new session...")
                # Create a new session using the session service
                new_session = SessionService.create_session(user_id=None)
                chat_request.session_id = new_session.id  # Update the request with new session ID
                print(f"DEBUG: Created new session with ID: {new_session.id}")
            else:
                print(f"DEBUG: Session found for ID: {chat_request.session_id}")

            # Validate the query
            from services.rag_service import RAGService
            rag_service = RAGService()
            if not rag_service.validate_query(chat_request.message):
                raise HTTPException(status_code=400, detail="Invalid query")

            # Get conversation history for context using shared service
            session_messages = SessionService.get_session_messages(chat_request.session_id)
            print(f"DEBUG: Found {len(session_messages)} messages in session")
            conversation_history = [
                {"role": msg.role, "content": msg.content}
                for msg in session_messages
            ]
            if conversation_history:
                print(f"DEBUG: Conversation history contains {len(conversation_history)} messages")
                for i, msg in enumerate(conversation_history):
                    print(f"DEBUG: History {i} - Role: {msg['role']}, Content preview: {msg['content'][:100]}...")

            # Prepare the query with conversation context
            contextual_query = rag_service.get_conversation_context(
                chat_request.message,
                conversation_history
            )
            print(f"DEBUG: Contextual query prepared, length: {len(contextual_query)}")
            if len(conversation_history) > 0:
                print(f"DEBUG: Contextual query preview: {contextual_query[:200]}...")

            # Try to generate response with retries
            rag_response = None
            max_retries = 3
            retry_count = 0

            while retry_count < max_retries:
                try:
                    # Generate response using RAG
                    rag_response = rag_service.generate_response(
                        query=contextual_query,
                        mode=chat_request.mode,
                        context_text=chat_request.context_text,
                        modules=chat_request.modules
                    )
                    print(f"DEBUG: RAG response generated successfully on attempt {retry_count + 1}")
                    break
                except Exception as rag_error:
                    retry_count += 1
                    print(f"DEBUG: RAG generation failed on attempt {retry_count}, error: {rag_error}")
                    if retry_count >= max_retries:
                        print(f"DEBUG: All {max_retries} attempts failed, using fallback response")
                        # Use fallback response when RAG fails completely
                        rag_response = {
                            "response": f"I couldn't retrieve book context, but I can provide a general explanation about: {chat_request.message}",
                            "references": []
                        }
                    else:
                        # Wait briefly before retry
                        await asyncio.sleep(0.5)

            # Create message records
            from uuid import uuid4
            from datetime import datetime
            user_message = Message(
                id=str(uuid4()),
                session_id=chat_request.session_id,
                role="user",
                content=chat_request.message,
                timestamp=datetime.utcnow()
            )

            assistant_message = Message(
                id=str(uuid4()),
                session_id=chat_request.session_id,
                role="assistant",
                content=rag_response["response"],
                timestamp=datetime.utcnow()
            )

            # Store messages using shared service
            SessionService.add_messages(chat_request.session_id, user_message, assistant_message)

            # Update session last activity using shared service
            SessionService.update_session_activity(chat_request.session_id)

            return ChatResponse(
                response=rag_response["response"],
                session_id=chat_request.session_id,
                references=rag_response["references"]
            )
        except HTTPException:
            raise
        except Exception as e:
            import traceback
            print(f"Error in chat endpoint: {e}")
            print(f"Full traceback: {traceback.format_exc()}")
            # Debug: Print the query that caused the error
            print(f"DEBUG: Original query that caused error: '{chat_request.message}'")
            print(f"DEBUG: Query lowercase: '{chat_request.message.lower().strip()}'")
            print(f"DEBUG: Session ID: {chat_request.session_id}")
            # Debug: Print session status
            session_check = SessionService.get_session(chat_request.session_id)
            print(f"DEBUG: Session exists: {session_check is not None}")
            if session_check:
                session_messages = SessionService.get_session_messages(chat_request.session_id)
                print(f"DEBUG: Number of messages in session: {len(session_messages)}")
                for i, msg in enumerate(session_messages):
                    print(f"DEBUG: Message {i} - Role: {msg.role}, Content preview: {msg.content[:100]}...")

            # Return a safe fallback response instead of the unavailable message
            fallback_response = f"I'm sorry, I couldn't process your request about '{chat_request.message}' right now. However, I can help with information from the Physical AI & Humanoid Robotics book. What else would you like to know?"

            # Create message records for the error case as well
            from uuid import uuid4
            from datetime import datetime
            user_message = Message(
                id=str(uuid4()),
                session_id=chat_request.session_id,
                role="user",
                content=chat_request.message,
                timestamp=datetime.utcnow()
            )

            assistant_message = Message(
                id=str(uuid4()),
                session_id=chat_request.session_id,
                role="assistant",
                content=fallback_response,
                timestamp=datetime.utcnow()
            )

            # Store messages using shared service
            SessionService.add_messages(chat_request.session_id, user_message, assistant_message)

            # Update session last activity using shared service
            SessionService.update_session_activity(chat_request.session_id)

            return ChatResponse(
                response=fallback_response,
                session_id=chat_request.session_id,
                references=[]  # No references for fallback response
            )


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
        print(f"Error in get session messages endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")