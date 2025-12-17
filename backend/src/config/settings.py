from pydantic_settings import BaseSettings
from typing import Optional
import os


class Settings(BaseSettings):
    # OpenAI Configuration
    openai_api_key: str = os.getenv("OPENAI_API_KEY", "")

    # Google Gemini Configuration
    gemini_api_key: str = os.getenv("GEMINI_API_KEY", "")

    # Cohere Configuration
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")

    # Qdrant Vector Database Configuration
    qdrant_url: Optional[str] = os.getenv("QDRANT_URL")
    qdrant_api_key: Optional[str] = os.getenv("QDRANT_API_KEY")
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "textbook_chunks")

    # Neon Postgres Configuration
    database_url: str = os.getenv("DATABASE_URL", "postgresql://localhost/rag_chatbot")

    # Application Configuration
    api_key: Optional[str] = os.getenv("API_KEY")
    debug: bool = os.getenv("DEBUG", "false").lower() == "true"
    log_level: str = os.getenv("LOG_LEVEL", "INFO")

    # RAG Configuration
    vector_db_timeout: int = int(os.getenv("VECTOR_DB_TIMEOUT", "30"))
    llm_temperature: float = float(os.getenv("LLM_TEMPERATURE", "0.7"))
    max_context_length: int = int(os.getenv("MAX_CONTEXT_LENGTH", "4096"))
    chunk_size: int = int(os.getenv("CHUNK_SIZE", "1000"))

    # API Configuration
    allowed_hosts: str = os.getenv("ALLOWED_HOSTS", "*")
    port: int = int(os.getenv("PORT", "8000"))

    class Config:
        env_file = ".env"


# Create settings instance
settings = Settings()