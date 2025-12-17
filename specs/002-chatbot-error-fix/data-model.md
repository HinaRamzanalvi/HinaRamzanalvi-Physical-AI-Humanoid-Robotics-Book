# Data Model: Chatbot Error Response Fix

**Feature**: 002-chatbot-error-fix
**Created**: 2025-12-16

## Entities

### Chat Request
**Description**: Represents a user's question or input to the chatbot system

**Fields**:
- `id` (string): Unique identifier for the request
- `userId` (string, optional): Identifier for the user making the request
- `sessionId` (string): Session identifier to maintain conversation context
- `message` (string): The user's question or input text
- `timestamp` (datetime): When the request was made
- `metadata` (object, optional): Additional context about the request

**Validation**:
- `message` must be non-empty string with maximum length of 10000 characters
- `sessionId` must be a valid identifier format
- `timestamp` must be in ISO 8601 format

### Chat Response
**Description**: Represents the system's reply to the user, including error responses

**Fields**:
- `id` (string): Unique identifier for the response
- `requestId` (string): Reference to the corresponding request
- `content` (string): The response content or error message
- `type` (enum): Response type (success, error, loading, info)
- `timestamp` (datetime): When the response was generated
- `sources` (array, optional): Array of source documents used in RAG response

**Validation**:
- `content` must be non-empty string for success responses
- `type` must be one of the defined enum values
- `requestId` must reference an existing Chat Request

### Error Log
**Description**: Contains technical details about system errors for debugging purposes

**Fields**:
- `id` (string): Unique identifier for the log entry
- `requestId` (string, optional): Reference to the associated request
- `errorType` (string): Category of the error (API_ERROR, VALIDATION_ERROR, etc.)
- `message` (string): Technical error message
- `stackTrace` (string, optional): Full stack trace for debugging
- `timestamp` (datetime): When the error occurred
- `severity` (enum): Error severity level (LOW, MEDIUM, HIGH, CRITICAL)
- `userMessage` (string): User-friendly error message to display

**Validation**:
- `errorType` and `severity` must be from predefined enums
- `timestamp` must be in ISO 8601 format
- `userMessage` should be non-empty and user-appropriate

### User Session
**Description**: Tracks user interaction state and context across multiple exchanges

**Fields**:
- `id` (string): Unique session identifier
- `userId` (string, optional): Reference to the user
- `createdAt` (datetime): Session start time
- `lastActiveAt` (datetime): Time of last interaction
- `context` (object): Conversation context and history
- `status` (enum): Session status (ACTIVE, INACTIVE, EXPIRED)

**Validation**:
- `id` must be unique across all sessions
- `createdAt` and `lastActiveAt` must be in ISO 8601 format
- `status` must be one of the defined enum values