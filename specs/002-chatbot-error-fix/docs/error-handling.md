# Chatbot Error Handling Documentation

## Overview
This document describes the error handling improvements made to the RAG Chatbot system to provide better user experience when errors occur.

## Error Response Format
All API errors now follow a consistent structured format:

```json
{
  "error": {
    "code": "string - Standard error code (e.g., CHAT_001, CHAT_005)",
    "message": "string - User-friendly message explaining what went wrong",
    "details": {
      "technical": "string - Technical details for debugging",
      "timestamp": "datetime - When error occurred",
      "requestId": "string - Associated request identifier (if available)",
      "validation_errors": "array - Detailed validation errors (if applicable)"
    }
  }
}
```

## Error Codes
- `CHAT_001`: Validation error (invalid input, missing required fields)
- `CHAT_002`: Service unavailable (backend service down)
- `CHAT_003`: Rate limit exceeded
- `CHAT_004`: Resource not found (session not found, etc.)
- `CHAT_005`: Internal server error (unexpected error)
- `CHAT_006`: Gateway timeout

## Frontend Error Handling
The frontend chatbot component now:
1. Extracts specific error messages from API responses
2. Provides context-appropriate user-friendly messages
3. Handles different error types with specific guidance:
   - Network errors: "Unable to connect to the server. Please check your internet connection and try again."
   - Rate limiting: "Too many requests. Please wait a moment before trying again."
   - Validation errors: "Invalid request. Please check your input and try again."
   - Server errors: "The server encountered an error. Please try again later."

## Backend Error Handling
The backend API now:
1. Uses custom exception handlers for consistent error responses
2. Maps HTTP status codes to appropriate error codes
3. Provides detailed technical information for debugging
4. Maintains security by not exposing sensitive internal details

## Testing
- Unit tests verify error response format
- Integration tests validate end-to-end error handling
- Error scenarios are properly covered in test suite