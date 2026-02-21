# import sys
# import os
# from fastapi import FastAPI
# from fastapi.middleware.cors import CORSMiddleware

# # Add the backend/src directory to the Python path to allow absolute imports
# sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# from api.chat_router import router as chat_router
# from api.session_router import router as session_router
# from config import settings
# from services.rag_service import RAGService

# print("DEBUG: main.py has been loaded!")

# app = FastAPI(
#     title="RAG Chatbot API",
#     description="Retrieval-Augmented Generation Chatbot for Physical AI & Humanoid Robotics Textbook",
#     version="1.0.0"
# )

# # Add CORS middleware
# app.add_middleware(
#     CORSMiddleware,
#     allow_origins=["*"],  # In production, replace with specific origins
#     allow_credentials=True,
#     allow_methods=["*"],
#     allow_headers=["*"],
#     allow_origin_regex=r"https?://.*",  # Allow any HTTP/HTTPS origin during development
# )

# # Include routers
# app.include_router(chat_router, prefix="/api/chat", tags=["chat"])
# app.include_router(session_router, prefix="/api/session", tags=["session"])

# @app.get("/")
# def read_root():
#     return {"message": "RAG Chatbot API is running!"}

# @app.get("/health")
# def health_check():
#     return {"status": "healthy"}

# @app.get("/api/health")
# def detailed_health_check():
#     """Detailed health check that tests LLM and Qdrant connections"""
#     # Force a fresh RAGService to ensure latest settings are used
#     from services.rag_service import RAGService
#     rag_service = RAGService()
#     llm_test = rag_service.test_llm_connection()

#     return {
#         "status": "healthy",
#         "llm_connection": llm_test,
#         "qdrant_connection": "Assumed healthy based on service initialization"
#     }

# if __name__ == "__main__":
#     import uvicorn
#     uvicorn.run(
#         "main:app",
#         host="0.0.0.0",
#         port=8000,
#         reload=True
#     )







import sys
import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

# Add the backend/src directory to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from api.chat_router import router as chat_router
from api.session_router import router as session_router
from config import settings
from services.rag_service import RAGService

print("DEBUG: main.py has been loaded!")

app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation Chatbot for Physical AI & Humanoid Robotics Textbook",
    version="1.0.0"
)

# =========================
# CORS (frontend connect kare)
# =========================
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# =========================
# ROUTERS (IMPORTANT FIX)
# =========================
# chat messages
app.include_router(chat_router, prefix="/api/chat", tags=["chat"])

# sessions bhi same prefix me (IMPORTANT)
app.include_router(session_router, prefix="/api/chat", tags=["session"])

# =========================
# BASIC ROUTES
# =========================
@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running!"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}

@app.get("/api/health")
def detailed_health_check():
    rag_service = RAGService()
    llm_test = rag_service.test_llm_connection()

    return {
        "status": "healthy",
        "llm_connection": llm_test,
        "qdrant_connection": "Assumed healthy"
    }

# =========================
# RUN SERVER
# =========================
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )