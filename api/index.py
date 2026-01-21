import os
import uuid
import random
import logging
from datetime import datetime
from typing import List, Dict, Any, Optional
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from mangum import Mangum
import requests
from bs4 import BeautifulSoup

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Settings
class Settings:
    def __init__(self):
        self.cohere_api_key = os.getenv("COHERE_API_KEY", "")
        self.gemini_api_key = os.getenv("GEMINI_API_KEY", "")
        self.llm_provider = os.getenv("LLM_PROVIDER", "gemini")
        self.qdrant_url = os.getenv("QDRANT_URL", "")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY", "")
        self.target_url = os.getenv("TARGET_URL", "https://httpbin.org/html")
        self.chunk_size = int(os.getenv("CHUNK_SIZE", "1000"))
        self.chunk_overlap = int(os.getenv("CHUNK_OVERLAP", "100"))
        self.rate_limit_delay = float(os.getenv("RATE_LIMIT_DELAY", "1.0"))
        self.cohere_model = os.getenv("COHERE_MODEL", "embed-multilingual-v2.0")

settings = Settings()

# Models
class Message(BaseModel):
    id: str
    role: str
    content: str
    timestamp: datetime
    references: Optional[List[str]] = []

class ChatSessionCreate(BaseModel):
    user_id: Optional[str] = None

class ChatSession(BaseModel):
    id: str
    user_id: Optional[str] = None
    created_at: datetime
    updated_at: datetime
    messages: List[Message] = []

# In-memory sessions
sessions: Dict[str, ChatSession] = {}

# Services
class TextChunker:
    def __init__(self):
        self.chunk_size = settings.chunk_size
        self.chunk_overlap = settings.chunk_overlap

    def chunk_text(self, text: str) -> List[str]:
        chunks = []
        start = 0
        while start < len(text):
            end = start + self.chunk_size
            if end >= len(text):
                chunks.append(text[start:])
                break
            while end > start and text[end] not in [' ', '.', '!', '?', '\n', '\t']:
                end -= 1
            if end == start:
                end = start + self.chunk_size
                if end > len(text):
                    end = len(text)
            chunks.append(text[start:end])
            start = end - self.chunk_overlap
            if start >= len(text):
                break
        return chunks

class EmbeddingService:
    def __init__(self):
        self.cohere_api_key = settings.cohere_api_key
        self.model = settings.cohere_model

    def get_embeddings(self, texts: List[str]):
        try:
            import cohere
            co = cohere.Client(self.cohere_api_key)
            response = co.embed(texts=texts, model=self.model)
            return response.embeddings
        except Exception as e:
            logger.error(f"Embedding error: {e}")
            return [[random.random() for _ in range(1024)] for _ in texts]

class WebScraper:
    def scrape_url(self, url: str) -> str:
        try:
            response = requests.get(url)
            response.raise_for_status()
            soup = BeautifulSoup(response.content, 'html.parser')
            for script in soup(["script", "style"]):
                script.decompose()
            text = ' '.join(line.strip() for line in soup.get_text().splitlines() if line.strip())
            return text
        except Exception as e:
            logger.error(f"Scraping error for {url}: {e}")
            return f"Error scraping content from {url}"

class SessionService:
    @staticmethod
    def create_session(user_id: Optional[str] = None) -> ChatSession:
        session_id = str(uuid.uuid4())
        now = datetime.now()
        session = ChatSession(
            id=session_id,
            user_id=user_id,
            created_at=now,
            updated_at=now
        )
        sessions[session_id] = session
        return session

    @staticmethod
    def get_session(session_id: str) -> Optional[ChatSession]:
        return sessions.get(session_id)

class RAGService:
    def __init__(self):
        self.text_chunker = TextChunker()
        self.embedding_service = EmbeddingService()
        self.web_scraper = WebScraper()

    def generate_response(self, query: str, mode: str = "full_book", modules: List[int] = [1,2,3,4,5,6]) -> Dict[str, Any]:
        try:
            if settings.llm_provider.lower() == "gemini":
                return self._generate_gemini_response(query, mode, modules)
            elif settings.llm_provider.lower() == "openai":
                return self._generate_openai_response(query, mode, modules)
            else:
                return self._generate_cohere_response(query, mode, modules)
        except Exception as e:
            logger.error(f"Error in generate_response: {e}")
            return {"response": f"Error: {str(e)}", "references": []}

    def _generate_gemini_response(self, query: str, mode: str, modules: List[int]) -> Dict[str, Any]:
        try:
            import google.generativeai as genai
            genai.configure(api_key=settings.gemini_api_key)
            model = genai.GenerativeModel('gemini-pro')
            prompt = f"Answer the following about Physical AI & Humanoid Robotics: {query}"
            response = model.generate_content(prompt)
            if hasattr(response, 'text') and response.text:
                response_text = response.text
            else:
                response_text = "Unable to generate response."
            references = [f"Module {mod}" for mod in modules[:3]]
            return {"response": response_text, "references": references}
        except Exception as e:
            logger.error(f"Gemini error: {e}")
            return {"response": f"Error: {str(e)}", "references": []}

    def _generate_openai_response(self, query: str, mode: str, modules: List[int]) -> Dict[str, Any]:
        try:
            from openai import OpenAI
            client = OpenAI(api_key=settings.openai_api_key)
            response = client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": query}],
                max_tokens=1000,
                temperature=0.1
            )
            references = [f"Module {mod}" for mod in modules[:3]]
            return {"response": response.choices[0].message.content, "references": references}
        except Exception as e:
            logger.error(f"OpenAI error: {e}")
            return {"response": f"Error: {str(e)}", "references": []}

    def _generate_cohere_response(self, query: str, mode: str, modules: List[int]) -> Dict[str, Any]:
        try:
            import cohere
            co = cohere.Client(settings.cohere_api_key)
            response = co.generate(model='command-xlarge-nightly', prompt=f"{query}", max_tokens=1000, temperature=0.1)
            references = [f"Module {mod}" for mod in modules[:3]]
            return {"response": response.generations[0].text, "references": references}
        except Exception as e:
            logger.error(f"Cohere error: {e}")
            return {"response": f"Error: {str(e)}", "references": []}

    def test_llm_connection(self):
        try:
            if settings.llm_provider.lower() == "gemini":
                import google.generativeai as genai
                genai.configure(api_key=settings.gemini_api_key)
                model = genai.GenerativeModel('gemini-pro')
                _ = model.generate_content("test")
            elif settings.llm_provider.lower() == "openai":
                from openai import OpenAI
                client = OpenAI(api_key=settings.openai_api_key)
                client.chat.completions.create(model="gpt-3.5-turbo", messages=[{"role": "user", "content": "test"}], max_tokens=5)
            elif settings.llm_provider.lower() == "cohere":
                import cohere
                co = cohere.Client(settings.cohere_api_key)
                co.generate(model='command-xlarge-nightly', prompt='test', max_tokens=5)
            return {"connected": True, "provider": settings.llm_provider, "message": "LLM connection successful"}
        except Exception as e:
            logger.error(f"LLM connection test failed: {e}")
            return {"connected": False, "error": str(e)}

# FastAPI app
app = FastAPI(title="RAG Chatbot API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Routes
@app.post("/api/chat/chat")
async def chat_endpoint(session_id: str, message: str, mode: str = "full_book", modules: List[int] = [1,2,3,4,5,6]):
    try:
        rag_service = RAGService()
        result = rag_service.generate_response(message, mode, modules)
        user_msg = Message(id=str(uuid.uuid4()), role="user", content=message, timestamp=datetime.now())
        assistant_msg = Message(id=str(uuid.uuid4()), role="assistant", content=result["response"], timestamp=datetime.now(), references=result.get("references", []))
        if session_id in sessions:
            session = sessions[session_id]
            session.messages.extend([user_msg, assistant_msg])
            session.updated_at = datetime.now()
        return {"response": result["response"], "references": result.get("references", []), "session_id": session_id}
    except Exception as e:
        logger.error(f"Chat endpoint error: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing chat: {str(e)}")

@app.post("/api/chat/sessions")
async def create_chat_session():
    session = SessionService.create_session()
    return session

@app.post("/api/session/sessions")
async def create_session(session_data: ChatSessionCreate = None):
    user_id = session_data.user_id if session_data else None
    session = SessionService.create_session(user_id)
    return session

@app.get("/api/session/sessions/{session_id}/messages")
async def get_session_messages(session_id: str):
    session = SessionService.get_session(session_id)
    if not session:
        raise HTTPException(status_code=404, detail="Session not found")
    return session.messages

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running!"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}

@app.get("/api/health")
def detailed_health_check():
    try:
        rag_service = RAGService()
        llm_test = rag_service.test_llm_connection()
        return {"status": "healthy", "llm_connection": llm_test, "qdrant_connection": "Not applicable in this serverless implementation", "environment": {"llm_provider": settings.llm_provider, "qdrant_url_set": bool(settings.qdrant_url)}}
    except Exception as e:
        logger.error(f"Health check error: {e}")
        return {"status": "unhealthy", "error": str(e)}

# Vercel serverless handler
handler = Mangum(app)
