# Physical AI & Humanoid Robotics Textbook with RAG Chatbot

This project implements an AI-native Retrieval-Augmented Generation (RAG) chatbot for the Physical AI & Humanoid Robotics textbook, integrated directly into a Docusaurus-based documentation site.

## Features

- **Full-book RAG Search**: Ask questions about the entire textbook content
- **Context-only Mode**: Use selected text as context for focused questions
- **Persistent Conversations**: Maintain session context across page navigations
- **Real-time Interaction**: Instant AI responses based on textbook content
- **Docusaurus Integration**: Seamlessly embedded in the textbook website

## Architecture

- **Backend**: FastAPI application with configurable LLM provider (OpenAI or Google Gemini)
- **Vector Database**: Qdrant Cloud for semantic search
- **Session Storage**: Neon Serverless Postgres for conversation history
- **Frontend**: React components integrated with Docusaurus
- **AI Models**: OpenAI GPT or Google Gemini for response generation (embeddings still use Cohere)

## Quick Start

### Prerequisites

- Python 3.11+
- Node.js 18+ (for Docusaurus)
- Either OpenAI API key OR Google Gemini API key
- Qdrant Cloud account (free tier sufficient)

### Setup

1. **Install Python dependencies**
   ```bash
   pip install -r requirements.txt
   ```

2. **Set up environment variables**
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
   QDRANT_API_KEY=your_qdrant_api_key  # if authentication required
   DATABASE_URL=your_neon_database_url
   ```

3. **Initialize the vector database with textbook content**
   ```bash
   python -m vector_ingestion.src.vector_ingestor --sample
   # Or to ingest from your own textbook files:
   # python -m vector_ingestion.src.vector_ingestor --file path/to/textbook.txt
   ```

4. **Start the backend server**
   ```bash
   uvicorn backend.src.main:app --reload
   ```

5. **Start the Docusaurus frontend** (in a separate terminal)
   ```bash
   yarn
   yarn start
   ```

## Usage

1. Navigate to your Docusaurus textbook site
2. Click the "AI Assistant" button in the bottom-right corner
3. Ask questions about the textbook content
4. Toggle between "Full Book Search" and "Context Only" modes as needed

## Development

### Backend Structure

```
backend/
├── src/
│   ├── models/           # Data models (Pydantic)
│   ├── services/         # Business logic (embedding, RAG, etc.)
│   ├── api/              # API routes (FastAPI)
│   └── config/           # Configuration and settings
└── tests/                # Unit and integration tests
```

### Frontend Components

```
src/
└── components/
    └── Chatbot/          # React components for the chat interface
    └── Docusaurus/       # Integration components for Docusaurus
```

### Vector Ingestion

```
vector_ingestion/
└── src/                  # Scripts to process textbook content
```

## API Endpoints

- `POST /api/chat` - Send a message and receive a response
- `POST /api/sessions` - Create a new chat session
- `GET /api/sessions/{session_id}` - Get session details
- `GET /api/sessions/{session_id}/messages` - Get session messages
- `GET /health` - Health check endpoint

## Testing

Run backend tests:
```bash
pytest backend/tests/
```

## Local Development with Docusaurus

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

### Installation

```bash
yarn
```

### Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

### Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

### Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.




# 🤖 Physical AI & Humanoid Robotics – RAG Chatbot

A **Retrieval-Augmented Generation (RAG) Chatbot** built for the **Physical AI & Humanoid Robotics textbook**, designed to answer **book-based academic questions only** across **six core modules**.
This project was developed and deployed as part of a **Hackathon submission**, with a focus on reliability, academic integrity, and real-world AI system design.

---

## 🌟 Project Highlights

* 📚 **Strictly Book-Based Answers** (No hallucinations)
* 🧠 **6 Fully Indexed Academic Modules**
* 🔍 **Vector Search using Qdrant**
* 🤖 **RAG Architecture (Search + Generation)**
* ⚡ **Robust Fallback Handling (API quota safe)**
* 🎨 **Clean Academic UI (Deep Blue + Cyan Theme)**
* 🌐 **Deployed on Vercel**

---

## 📦 Modules Covered

1. **The Robotic Nervous System (ROS 2)**
   Nodes, topics, DDS middleware, real-time communication

2. **The Digital Twin (Gazebo & Unity)**
   Simulation, physics engines, virtual robot testing

3. **The AI-Robot Brain (NVIDIA Isaac)**
   GPU acceleration, Isaac Sim, Sim2Real transfer

4. **Vision-Language-Action (VLA)**
   Multimodal AI for perception, language understanding, and action

5. **Humanoid Robot Development**
   Mechanical design, locomotion, balance, actuation

6. **Conversational Robotics**
   Dialogue systems, context tracking, human-robot interaction

---

## 🧠 System Architecture

```text
User Query
   ↓
Frontend (React)
   ↓
Backend API (RAG Pipeline)
   ↓
Vector Search (Qdrant)
   ↓
Relevant Textbook Chunks
   ↓
LLM Response (Book-Only)
```

---

## ⚙️ Tech Stack

### Frontend

* React / Next.js
* Tailwind CSS
* Academic UI Design

### Backend

* Node.js / Python (RAG Service)
* Gemini / OpenAI (with fallback handling)

### Vector Database

* **Qdrant** (All 6 modules indexed)

### Deployment

* **GitHub** – Source control
* **Vercel** – Production deployment

---

## 🛡️ Key Features

* ✅ Searches **all 6 modules** by default
* ✅ Supports module-specific queries
* ✅ Graceful fallback when API quota is exceeded
* ✅ Maintains "References from Book" in every response
* ✅ Session-resilient backend
* ✅ No generic or external content allowed

---

## 🧪 Sample Questions

* *How do nodes and topics communicate in ROS 2, and why is DDS important?*
* *How does Unity differ from Gazebo in digital twin simulation?*
* *What role does NVIDIA Isaac Sim play in humanoid robot development?*
* *How does the Vision-Language-Action framework integrate multimodal AI?*
* *What are the core design principles of humanoid robots?*
* *How do robots maintain context in conversational systems?*

---

## 🚀 Deployment

The application is deployed on **Vercel** and connected directly to this GitHub repository.
Every push to the main branch triggers an automatic redeployment.

---

## 🏆 Hackathon Ready

This project demonstrates:

* Real-world **RAG system design**
* Academic content grounding
* Robust error handling
* Production-level deployment

It is fully suitable for **AI, Robotics, and Physical AI hackathon evaluations**.

---

## 🙏 Acknowledgements

* Physical AI & Humanoid Robotics Textbook
* ROS 2 & Open Robotics
* NVIDIA Isaac Platform
* Qdrant Vector Database

---

## 📌 Author

**Hackathon Participant**
Physical AI & Humanoid Robotics Project

---

> "Building trustworthy AI systems for embodied intelligence." 🤍
# 🤖 Physical AI & Humanoid Robotics – RAG Chatbot

A **Retrieval-Augmented Generation (RAG) Chatbot** built for the **Physical AI & Humanoid Robotics textbook**, designed to answer **book-based academic questions only** across **six core modules**.
This project was developed and deployed as part of a **Hackathon submission**, with a focus on reliability, academic integrity, and real-world AI system design.

---

## 🌟 Project Highlights

* 📚 **Strictly Book-Based Answers** (No hallucinations)
* 🧠 **6 Fully Indexed Academic Modules**
* 🔍 **Vector Search using Qdrant**
* 🤖 **RAG Architecture (Search + Generation)**
* ⚡ **Robust Fallback Handling (API quota safe)**
* 🎨 **Clean Academic UI (Deep Blue + Cyan Theme)**
* 🌐 **Deployed on Vercel**

---

## 📦 Modules Covered

1. **The Robotic Nervous System (ROS 2)**
   Nodes, topics, DDS middleware, real-time communication

2. **The Digital Twin (Gazebo & Unity)**
   Simulation, physics engines, virtual robot testing

3. **The AI-Robot Brain (NVIDIA Isaac)**
   GPU acceleration, Isaac Sim, Sim2Real transfer

4. **Vision-Language-Action (VLA)**
   Multimodal AI for perception, language understanding, and action

5. **Humanoid Robot Development**
   Mechanical design, locomotion, balance, actuation

6. **Conversational Robotics**
   Dialogue systems, context tracking, human-robot interaction

---

## 🧠 System Architecture

```text
User Query
   ↓
Frontend (React)
   ↓
Backend API (RAG Pipeline)
   ↓
Vector Search (Qdrant)
   ↓
Relevant Textbook Chunks
   ↓
LLM Response (Book-Only)
