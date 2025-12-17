# Research: RAG Chatbot Implementation

## Decision: Backend Framework Choice
**Rationale**: FastAPI chosen for its excellent async support, automatic API documentation, and strong typing capabilities which are essential for an AI-powered backend service.
**Alternatives considered**:
- Flask: Simpler but less performant for async operations
- Django: More complex, overkill for this API-focused application
- Express.js: Node.js alternative, but Python ecosystem better for AI/ML

## Decision: Vector Database Strategy
**Rationale**: Qdrant Cloud selected as specified in requirements. It provides managed vector search capabilities with good performance for semantic similarity searches required in RAG systems.
**Alternatives considered**:
- Pinecone: Commercial alternative but not specified in requirements
- Weaviate: Open source alternative but Qdrant was specified
- FAISS: Facebook's library but requires more manual management

## Decision: Database for Metadata and Chat History
**Rationale**: Neon Serverless Postgres chosen as specified in requirements. It provides serverless PostgreSQL with branch/clone capabilities, perfect for storing structured metadata about textbook chunks and chat history.
**Alternatives considered**:
- MongoDB: NoSQL option but Postgres better for structured data
- SQLite: Simpler but not scalable for concurrent users
- Redis: Good for caching but not ideal for structured metadata storage

## Decision: Textbook Content Processing
**Rationale**: Textbook content in Markdown files will be parsed and chunked by module/chapter/section as specified. Each chunk will be stored with metadata (module, chapter, section_title, source_file) to enable proper citations.
**Alternatives considered**:
- PDF parsing: More complex, Markdown is already available
- HTML parsing: Not needed since source is Markdown
- Manual chunking: Too time-consuming, automated approach preferred

## Decision: Frontend Integration Approach
**Rationale**: Docusaurus integration via React components provides seamless embedding of the chatbot into the existing textbook interface. This allows for text selection functionality and proper context awareness.
**Alternatives considered**:
- Standalone web application: Loses textbook context
- Iframe embedding: More complex to handle text selection
- Browser extension: Overly complex for this use case

## Decision: RAG Implementation Pattern
**Rationale**: Retrieval-Augmented Generation pattern will follow standard approach: 1) Parse and embed textbook content, 2) Store embeddings in vector database, 3) For queries, retrieve relevant chunks, 4) Generate response with context, 5) Provide citations.
**Alternatives considered**:
- Simple keyword search: Less effective for semantic queries
- Full LLM without RAG: Would cause hallucinations, violates requirement
- Rule-based system: Not sophisticated enough for complex queries

## Decision: Text Selection Mode Implementation
**Rationale**: For "Ask Selected Text" mode, the frontend will capture selected text and send it as context to the backend, which will limit retrieval to only that text segment. This ensures responses are restricted to user-highlighted content.
**Alternatives considered**:
- Client-side filtering: Less secure, harder to verify compliance
- Backend-only approach: Would require complex text identification
- Hybrid approach: Selected text sent as additional context with special flag

## Decision: AI Model Selection
**Rationale**: OpenAI models (as specified in requirements) will be used for response generation. OpenAI embeddings will be used for vector storage to maintain compatibility.
**Alternatives considered**:
- Open-source models (LLaMA, etc.): Not specified in requirements
- Anthropic Claude: Not specified in requirements
- Self-hosted models: More complex infrastructure