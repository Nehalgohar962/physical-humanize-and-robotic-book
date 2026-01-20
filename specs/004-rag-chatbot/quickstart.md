# Quickstart Guide: RAG Chatbot for Physical AI & Humanoid Robotics Textbook

## Prerequisites

- Python 3.11+
- Node.js 18+ (for Docusaurus)
- Access to either OpenAI API key OR Google Gemini API key
- Qdrant Cloud account (free tier sufficient)
- Neon Serverless Postgres account

## Setup Backend

1. **Install Python dependencies**:
   ```bash
   pip install fastapi uvicorn python-dotenv google-generativeai qdrant-client psycopg2-binary
   ```

2. **Set up environment variables**:
   Create a `.env` file with:
   ```
   # LLM Provider Configuration (choose one)
   LLM_PROVIDER=gemini          # or "openai"

   # API Keys (only provide the one for your chosen provider)
   GEMINI_API_KEY=your_gemini_api_key
   # OPENAI_API_KEY=your_openai_api_key

   # Other required keys
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_API_KEY=your_qdrant_api_key
   DATABASE_URL=your_neon_database_url
   ```

3. **Initialize the vector database**:
   Run the vector ingestion script to process textbook content:
   ```bash
   python vector_ingestion/src/vector_ingestor.py
   ```

4. **Start the backend server**:
   ```bash
   uvicorn backend.src.main:app --reload
   ```

## Integrate with Docusaurus

1. **Install the chatbot component**:
   The chatbot is designed as a React component that can be embedded in Docusaurus pages.

2. **Add to your Docusaurus configuration**:
   Update `docusaurus.config.ts` to include the chatbot component.

3. **Initialize on pages**:
   The chatbot will automatically appear on textbook pages with initialization code.

## API Endpoints

- `POST /api/chat`: Send a message and receive a response
- `POST /api/chat/context`: Send a message with specific context (selected text)
- `GET /api/sessions/{session_id}`: Get session details
- `POST /api/sessions`: Create a new session

## Running Tests

```bash
# Backend tests
pytest backend/tests/

# Frontend tests (when implemented)
npm test
```

## Development Workflow

1. **Start backend**: `uvicorn backend.src.main:app --reload`
2. **Start Docusaurus**: `npm start` in the Docusaurus directory
3. **Access the chatbot**: Visit any textbook page to see the embedded chatbot

## Troubleshooting

- **API errors**: Verify your LLM API key (OpenAI or Gemini) is correct and has sufficient quota
- **Vector search issues**: Check that textbook content was properly ingested into Qdrant
- **Database connection**: Ensure Neon Postgres connection string is properly formatted