from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from api.chat_router import router as chat_router
from api.session_router import router as session_router
from config import settings

app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation Chatbot for Physical AI & Humanoid Robotics Textbook",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    allow_origin_regex=r"https?://.*",  # Allow any HTTP/HTTPS origin during development
)

# Include routers
app.include_router(chat_router, prefix="/api/chat", tags=["chat"])
app.include_router(session_router, prefix="/api/session", tags=["session"])

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running!"}

@app.get("/health")
def health_check():
    return {"status": "healthy", "qdrant_available": True}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )