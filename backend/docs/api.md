# RAG Chatbot API Documentation

## Overview
The RAG Chatbot API provides endpoints for interacting with a Retrieval-Augmented Generation system that answers questions based on the Physical AI & Humanoid Robotics textbook content.

## Authentication
No authentication required for basic functionality. Optional session-based identification for persistent chat history.

## Endpoints

### POST /api/v1/chat/query
Submit a query to the RAG chatbot.

#### Request Body
```json
{
  "query_text": "string (required) - The user's question",
  "query_mode": "enum (required) - 'AskBook' or 'AskSelectedText'",
  "selected_text": "string (optional) - Text selected by user for AskSelectedText mode",
  "explanation_complexity": "string (optional) - 'beginner', 'intermediate', or 'advanced'",
  "session_id": "string (optional) - Existing session ID, will be created if not provided"
}
```

#### Response
```json
{
  "query_id": "string - Unique identifier for the query",
  "session_id": "string - Session identifier",
  "response_text": "string - The AI-generated response",
  "citations": [
    {
      "module": "string - Module name",
      "chapter": "string - Chapter name",
      "section_title": "string - Section title",
      "source_file": "string - Source file path"
    }
  ],
  "confidence_score": "number - Confidence level (0.0-1.0)",
  "status": "string - 'completed' or 'failed'"
}
```

### GET /chat/session/{session_id}
Retrieve chat history for a specific session.

#### Response
```json
{
  "session_id": "string",
  "queries": [
    {
      "query_id": "string",
      "query_text": "string",
      "response_text": "string",
      "created_at": "datetime",
      "citations": "array of citation objects"
    }
  ],
  "created_at": "datetime",
  "updated_at": "datetime"
}
```

### POST /chat/session/{session_id}/clear
Clear chat history for a specific session.

#### Response
```json
{
  "session_id": "string",
  "message": "string - Confirmation message",
  "cleared_at": "datetime"
}
```

### GET /health
Check the health status of the RAG chatbot service.

#### Response
```json
{
  "status": "string - 'healthy'",
  "timestamp": "datetime",
  "services": {
    "vector_db": "string - status of vector database connection",
    "llm": "string - status of LLM service connection",
    "textbook_content": "string - status of textbook content availability"
  }
}
```