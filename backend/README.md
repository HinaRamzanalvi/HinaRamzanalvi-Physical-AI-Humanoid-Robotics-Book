# RAG Chatbot Backend

This is the backend API for the RAG Chatbot for the Physical AI & Humanoid Robotics Textbook.

## Features

- RAG (Retrieval Augmented Generation) functionality
- Vector database integration (Qdrant)
- PostgreSQL for metadata storage
- OpenAI integration
- Session management
- Textbook content processing

## Tech Stack

- Python 3.11
- FastAPI
- Qdrant (Vector Database)
- PostgreSQL
- OpenAI API

## Installation

1. Make sure you have Python 3.11+ installed
2. Create a virtual environment:

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install dependencies:

```bash
pip install -r requirements.txt
```

## Configuration

Create a `.env` file in the backend root directory with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_postgres_connection_string
API_KEY=your_api_key_here
DEBUG=false
LOG_LEVEL=INFO
VECTOR_DB_TIMEOUT=30
LLM_TEMPERATURE=0.7
MAX_CONTEXT_LENGTH=4096
CHUNK_SIZE=1000
```

## Running the Server

To start the backend server:

```bash
uvicorn src.main:app --reload --port 8000
```

The API will be available at `http://localhost:8000`.

## API Endpoints

- `GET /api/v1/health` - Health check
- `POST /api/v1/chat/query` - Submit a chat query
- `GET /api/v1/chat/session/{session_id}` - Get session history
- `POST /api/v1/chat/session/{session_id}/clear` - Clear session history

## Processing Textbook Content

To process textbook content and populate the vector database:

```bash
python -m src.scripts.process_textbook --source-dir /path/to/textbook/markdown/files
```

## Environment Variables

All configuration is handled through environment variables as defined in the `.env` file.

## Project Structure

```
src/
├── models/         # Database models
├── services/       # Business logic
├── api/            # API endpoints
├── config/         # Configuration and settings
├── utils/          # Utility functions
├── scripts/        # Utility scripts
└── main.py         # Application entry point
```