# RAG Chatbot Integration Guide

## Overview

The RAG (Retrieval-Augmented Generation) chatbot is integrated into the Physical AI & Humanoid Robotics textbook Docusaurus site. It allows users to ask questions about the textbook content and receive AI-generated responses based solely on the textbook material.

## Features

1. **Full-book RAG Search**: Search across the entire textbook content
2. **Context-only Mode**: Use selected text as context for questions
3. **Persistent Sessions**: Maintain conversation history across page navigations
4. **Real-time Interaction**: Instant responses to user queries

## Architecture

The system consists of:
- **Frontend**: React components integrated into Docusaurus
- **Backend**: FastAPI service with OpenAI integration
- **Vector Database**: Qdrant for storing textbook embeddings
- **Session Storage**: In-memory (for demo) with database-ready structure

## API Endpoints

- `POST /api/chat` - Send a message and receive a response
- `POST /api/sessions` - Create a new session
- `GET /api/sessions/{session_id}` - Get session details
- `GET /api/sessions/{session_id}/messages` - Get session messages

## Frontend Components

- `RAGChatbot.js` - Main integration component
- `ChatInterface.tsx` - Chat UI component
- `Message.tsx` - Individual message display
- `ContextSelector.tsx` - Context selection UI

## Setup

1. Install Python dependencies: `pip install -r requirements.txt`
2. Set environment variables in `.env`:
   - `OPENAI_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY` (if applicable)
   - `DATABASE_URL`
3. Start the backend: `uvicorn backend.src.main:app --reload`
4. Run Docusaurus: `npm start`

## Ingesting Textbook Content

Use the vector ingestion scripts to process textbook content:

```bash
python -m vector_ingestion.src.vector_ingestor --sample
```

Or to ingest from files:

```bash
python -m vector_ingestion.src.vector_ingestor --file path/to/textbook.txt
```

## Environment Variables

- `OPENAI_API_KEY`: Your OpenAI API key
- `QDRANT_URL`: URL to your Qdrant instance
- `QDRANT_API_KEY`: Qdrant API key (if authentication required)
- `DATABASE_URL`: Database connection string

## Troubleshooting

- If the chatbot doesn't appear, check that the component is properly included in your Docusaurus layout
- If API calls fail, verify your environment variables are set correctly
- If responses seem irrelevant, check that the textbook content was properly ingested into the vector database

## Development

The system is designed for easy extension:
- Add new content types by extending the data loader
- Modify response formatting in the RAG service
- Customize UI components as needed