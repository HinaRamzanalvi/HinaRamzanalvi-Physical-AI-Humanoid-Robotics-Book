# RAG Chatbot for Physical AI & Humanoid Robotics Textbook

This feature provides a Retrieval-Augmented Generation (RAG) chatbot that allows students to ask questions about the Physical AI & Humanoid Robotics textbook and receive answers based on textbook content with proper citations.

## Features

1. **Ask Questions About Entire Book**: Students can ask questions about any topic in the textbook and receive answers based only on textbook content with proper citations.

2. **Ask Questions About Selected Text Only**: Students can highlight text on a page and ask questions specifically about that selected content, with AI responding using only that selected text.

3. **Interactive Learning with AI Agent**: Students can engage in multi-turn conversations with an AI agent that explains concepts in beginner-friendly terms while providing technical depth when needed.

4. **Explanation Complexity Control**: Users can select the complexity level (Beginner, Intermediate, Advanced) for explanations.

5. **Chat History**: Conversations are stored in sessions with the ability to retrieve and clear history.

## API Endpoints

- `POST /api/v1/chat/query` - Submit a query to the RAG chatbot
- `GET /chat/session/{session_id}` - Retrieve chat history for a specific session
- `POST /chat/session/{session_id}/clear` - Clear chat history for a specific session
- `GET /health` - Check the health status of the RAG chatbot service

## Frontend Integration

The chatbot is integrated as a floating component in the Docusaurus textbook site. It features:
- Text selection capability
- Complexity level controls
- Loading indicators
- Citation display
- Session management

## Setup Instructions

### Prerequisites

- Python 3.11+ installed
- Node.js 18+ installed (for Docusaurus development)
- OpenAI API key
- Qdrant Cloud account and API key
- Neon Serverless Postgres account and connection details

### Environment Setup

1. **Create Python virtual environment**:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

2. **Install Python dependencies**:
   ```bash
   pip install fastapi uvicorn python-dotenv openai qdrant-client psycopg2-binary
   ```

3. **Set up environment variables**:
   Create a `.env` file in the project root:
   ```env
   OPENAI_API_KEY=your_openai_api_key_here
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   DATABASE_URL=your_neon_postgres_connection_string
   ```

### Backend Setup

1. **Navigate to backend directory**:
   ```bash
   cd backend
   ```

2. **Run the backend server**:
   ```bash
   uvicorn src.main:app --reload --port 8000
   ```

### Textbook Content Processing

1. **Parse and embed textbook content**:
   ```bash
   python -m src.scripts.process_textbook --source-dir ../book/docs --chunk-size 1000
   ```

### Frontend Integration

1. **Navigate to the Docusaurus book directory**:
   ```bash
   cd book
   ```

2. **Install dependencies**:
   ```bash
   npm install
   ```

3. **Start the development server**:
   ```bash
   npm start
   ```

## API Usage Examples

### Submit a Query

```bash
curl -X POST http://localhost:8000/api/v1/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "Explain ROS 2 architecture",
    "query_mode": "AskBook",
    "explanation_complexity": "beginner",
    "session_id": "optional-session-id"
  }'
```

### Submit a Query with Selected Text

```bash
curl -X POST http://localhost:8000/api/v1/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "What does this mean?",
    "query_mode": "AskSelectedText",
    "selected_text": "The ROS 2 architecture consists of nodes, topics, services, and actions...",
    "explanation_complexity": "intermediate",
    "session_id": "optional-session-id"
  }'
```