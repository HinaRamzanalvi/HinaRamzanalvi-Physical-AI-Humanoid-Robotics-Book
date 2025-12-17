# Chatbot API Contracts

**Feature**: 002-chatbot-error-fix
**Created**: 2025-12-16

## Chat API Endpoints

### POST /api/chat/send
**Description**: Send a message to the chatbot and receive a response

**Request**:
```json
{
  "message": "string (required) - The user's message",
  "sessionId": "string (optional) - Session identifier to maintain context",
  "userId": "string (optional) - User identifier"
}
```

**Response Success (200)**:
```json
{
  "id": "string - Unique response identifier",
  "requestId": "string - Reference to the original request",
  "content": "string - The chatbot's response",
  "type": "enum (success) - Response type",
  "timestamp": "datetime - ISO 8601 timestamp",
  "sources": "array of objects - Source documents used in RAG response"
}
```

**Response Error (4xx/5xx)**:
```json
{
  "error": {
    "code": "string - Error code",
    "message": "string - User-friendly error message",
    "details": "object (optional) - Technical error details for debugging"
  }
}
```

### POST /api/chat/session/new
**Description**: Create a new chat session

**Request**:
```json
{
  "userId": "string (optional) - User identifier"
}
```

**Response Success (200)**:
```json
{
  "sessionId": "string - New session identifier",
  "createdAt": "datetime - ISO 8601 timestamp",
  "status": "string - Session status"
}
```

### GET /api/chat/session/{sessionId}/history
**Description**: Retrieve chat history for a session

**Response Success (200)**:
```json
{
  "sessionId": "string - Session identifier",
  "history": [
    {
      "id": "string - Message identifier",
      "role": "enum (user|assistant) - Who sent the message",
      "content": "string - Message content",
      "timestamp": "datetime - ISO 8601 timestamp"
    }
  ]
}
```

## Error Response Format

All error responses follow this standard format:

```json
{
  "error": {
    "code": "string - Standard error code (e.g., CHAT_001, VALIDATION_ERROR)",
    "message": "string - User-friendly message explaining what went wrong",
    "details": {
      "technical": "string - Technical details for debugging",
      "timestamp": "datetime - When error occurred",
      "requestId": "string - Associated request identifier"
    }
  }
}
```

## Common Error Codes

- `CHAT_001`: Request validation failed
- `CHAT_002`: AI service unavailable
- `CHAT_003`: Rate limit exceeded
- `CHAT_004`: Session expired
- `CHAT_005`: Internal processing error
- `CHAT_006`: Network connectivity issue