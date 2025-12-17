# Data Model: RAG Chatbot for Physical AI & Humanoid Robotics Textbook

## Entity: ChatSession
**Description**: Represents a user's conversation with the RAG chatbot, including message history and context
**Fields**:
- `id`: UUID (primary key) - Unique identifier for the session
- `user_id`: UUID (optional) - Identifier for authenticated user, null for anonymous sessions
- `created_at`: DateTime - Timestamp when session was created
- `updated_at`: DateTime - Timestamp when session was last updated
- `expires_at`: DateTime - Expiration time for the session
- `metadata`: JSON - Additional session metadata (user preferences, context flags)

**Validation Rules**:
- `id` must be unique and non-null
- `created_at` must be before `updated_at`
- `expires_at` must be in the future

**State Transitions**: None applicable - session is active until expiration

## Entity: TextbookChunk
**Description**: Represents a segment of the textbook content with associated metadata (module, chapter, section, source file)
**Fields**:
- `id`: UUID (primary key) - Unique identifier for the chunk
- `content`: Text - The actual text content of the chunk
- `module`: String - The module identifier (e.g., "Module 1: ROS 2 Nervous System")
- `chapter`: String - The chapter name within the module
- `section_title`: String - The specific section title
- `source_file`: String - Path to the source Markdown file
- `chunk_order`: Integer - Order of this chunk within the section
- `embedding_vector`: Binary - Vector embedding of the content (stored separately in vector database)
- `created_at`: DateTime - When this chunk was processed
- `updated_at`: DateTime - When this chunk was last updated

**Validation Rules**:
- `content` must not be empty
- `module`, `chapter`, `section_title`, and `source_file` must not be null
- `chunk_order` must be non-negative
- `embedding_vector` must exist (though stored separately in vector DB)

**State Transitions**: None applicable - chunks are static once created

## Entity: UserQuery
**Description**: Represents a question or request from the user, including the selected text context and mode (entire book vs selected text)
**Fields**:
- `id`: UUID (primary key) - Unique identifier for the query
- `session_id`: UUID (foreign key) - References the chat session
- `query_text`: Text - The user's question or input
- `query_mode`: Enum (AskBook, AskSelectedText) - Mode of the query
- `selected_text`: Text (optional) - Text selected by user for "Ask Selected Text" mode
- `created_at`: DateTime - When the query was submitted
- `processed_at`: DateTime (optional) - When the query was processed
- `status`: Enum (Pending, Processing, Completed, Failed) - Current status of the query
- `response_id`: UUID (foreign key, optional) - References the response if generated

**Validation Rules**:
- `query_text` must not be empty
- `session_id` must reference an existing session
- When `query_mode` is "AskSelectedText", `selected_text` must not be empty
- `created_at` must be before `processed_at` if the latter exists

**State Transitions**:
- Pending → Processing: When query is picked up for processing
- Processing → Completed: When response is successfully generated
- Processing → Failed: When query processing encounters an error

## Entity: RetrievedContext
**Description**: Represents the relevant textbook content retrieved based on the user's query for response generation
**Fields**:
- `id`: UUID (primary key) - Unique identifier for the retrieved context
- `query_id`: UUID (foreign key) - References the user query
- `chunk_id`: UUID (foreign key) - References the textbook chunk
- `relevance_score`: Float - Similarity score between query and chunk (0.0 to 1.0)
- `rank`: Integer - Rank of this chunk in the relevance order
- `created_at`: DateTime - When this context was retrieved

**Validation Rules**:
- `query_id` must reference an existing query
- `chunk_id` must reference an existing textbook chunk
- `relevance_score` must be between 0.0 and 1.0
- `rank` must be non-negative

**State Transitions**: None applicable - context is static once retrieved

## Entity: ChatResponse
**Description**: Represents the AI-generated response to a user query
**Fields**:
- `id`: UUID (primary key) - Unique identifier for the response
- `query_id`: UUID (foreign key) - References the user query
- `response_text`: Text - The AI-generated response
- `citations`: JSON - List of source chunks referenced in the response
- `confidence_score`: Float - Confidence level of the response (0.0 to 1.0)
- `created_at`: DateTime - When the response was generated
- `token_usage`: JSON - Information about tokens used in generation

**Validation Rules**:
- `query_id` must reference an existing query
- `response_text` must not be empty
- `confidence_score` must be between 0.0 and 1.0
- `citations` must be a valid JSON array of chunk references

**State Transitions**: None applicable - responses are static once generated