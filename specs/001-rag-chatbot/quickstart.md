# Quickstart Guide: RAG Chatbot for Physical AI & Humanoid Robotics Textbook

## Prerequisites

- Python 3.11+ installed
- Node.js 18+ installed (for Docusaurus development)
- OpenAI API key
- Qdrant Cloud account and API key
- Neon Serverless Postgres account and connection details

## Environment Setup

1. **Clone the repository**:
   ```bash
   git clone [repository-url]
   cd [repository-name]
   ```

2. **Create Python virtual environment**:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install Python dependencies**:
   ```bash
   pip install fastapi uvicorn python-dotenv openai qdrant-client psycopg2-binary
   ```

4. **Set up environment variables**:
   Create a `.env` file in the project root:
   ```env
   OPENAI_API_KEY=your_openai_api_key_here
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   DATABASE_URL=your_neon_postgres_connection_string
   ```

## Backend Setup

1. **Navigate to backend directory**:
   ```bash
   cd backend
   ```

2. **Run the backend server**:
   ```bash
   uvicorn src.main:app --reload --port 8000
   ```

3. **Verify the API is running**:
   Visit `http://localhost:8000/health` to check service status.

## Textbook Content Processing

1. **Parse and embed textbook content**:
   ```bash
   python -m src.scripts.process_textbook --source-dir ../book/docs --chunk-size 1000
   ```

2. **This script will**:
   - Read all Markdown files from the textbook
   - Chunk content by module/chapter/section
   - Generate embeddings for each chunk
   - Store chunks in Postgres with metadata
   - Store embeddings in Qdrant vector database

## Frontend Integration

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

4. **The chatbot component will be automatically integrated** into the textbook pages.

## API Usage Examples

### Submit a Query

```bash
curl -X POST http://localhost:8000/api/v1/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "Explain ROS 2 architecture",
    "query_mode": "AskBook",
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
    "session_id": "optional-session-id"
  }'
```

## Development Workflow

1. **Run backend with auto-reload**:
   ```bash
   cd backend
   uvicorn src.main:app --reload --port 8000
   ```

2. **In a separate terminal, run Docusaurus**:
   ```bash
   cd book
   npm start
   ```

3. **Access the textbook at** `http://localhost:3000` with the integrated chatbot.

## Testing

1. **Run backend tests**:
   ```bash
   cd backend
   pytest
   ```

2. **Run integration tests**:
   ```bash
   cd backend
   pytest tests/integration/
   ```

## Configuration

Key configuration parameters can be adjusted in `backend/src/config/settings.py`:
- `VECTOR_DB_TIMEOUT`: Timeout for vector database queries
- `LLM_TEMPERATURE`: Temperature setting for response generation
- `MAX_CONTEXT_LENGTH`: Maximum context length for LLM
- `CHUNK_SIZE`: Size of text chunks for RAG