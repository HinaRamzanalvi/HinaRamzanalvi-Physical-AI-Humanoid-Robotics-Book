from fastapi import APIRouter, Depends
from typing import Dict, Any
from sqlalchemy import text
from src.config.database import get_db
from src.config.vector_db import client
from src.config.settings import settings
import time

router = APIRouter()


@router.get("/health", summary="Check the health status of the RAG chatbot service")
async def health_check() -> Dict[str, Any]:
    """
    Check the health status of the RAG chatbot service
    """
    health_status = {
        "status": "healthy",
        "timestamp": time.time(),
        "services": {
            "vector_db": "unknown",
            "database": "unknown",
            "llm": "unknown",
            "textbook_content": "unknown"
        }
    }

    # Check vector database connection
    try:
        # Try to get collection info, but handle validation errors that occur with newer Qdrant versions
        try:
            client.get_collection(settings.qdrant_collection_name)
            health_status["services"]["vector_db"] = "healthy"
        except Exception as config_error:
            # If we get validation errors related to configuration schema mismatch,
            # check if it's just a schema issue by trying a basic operation
            if "validation errors" in str(config_error) or "parse_as_type" in str(config_error):
                # This is likely a schema mismatch issue, but the connection might still work
                # Try to list collections as a basic connectivity test
                try:
                    client.get_collections()
                    health_status["services"]["vector_db"] = "healthy (config validation issue - connection OK)"
                except:
                    health_status["services"]["vector_db"] = f"unhealthy: {str(config_error)}"
                    health_status["status"] = "unhealthy"
            else:
                # Some other error
                health_status["services"]["vector_db"] = f"unhealthy: {str(config_error)}"
                health_status["status"] = "unhealthy"
    except Exception as e:
        health_status["services"]["vector_db"] = f"unhealthy: {str(e)}"
        health_status["status"] = "unhealthy"

    # Check database connection
    try:
        db = next(get_db())
        db.execute(text("SELECT 1"))
        health_status["services"]["database"] = "healthy"
    except Exception as e:
        health_status["services"]["database"] = f"unhealthy: {str(e)}"
        health_status["status"] = "unhealthy"
    finally:
        try:
            db.close()
        except:
            pass

    # Check LLM connection (basic check for API key presence)
    try:
        if settings.openai_api_key:
            health_status["services"]["llm"] = "configured"
        else:
            health_status["services"]["llm"] = "not configured"
            health_status["status"] = "unhealthy"
    except Exception as e:
        health_status["services"]["llm"] = f"error: {str(e)}"
        health_status["status"] = "unhealthy"

    # Check textbook content availability (check if there are chunks in the database)
    try:
        from src.config.database import SessionLocal
        from src.models.textbook_chunk import TextbookChunk

        db_session = SessionLocal()
        chunk_count = db_session.query(TextbookChunk).count()
        db_session.close()

        if chunk_count > 0:
            health_status["services"]["textbook_content"] = f"available ({chunk_count} chunks)"
        else:
            health_status["services"]["textbook_content"] = "no content available"
            # Note: This is not necessarily an unhealthy state - content might not be processed yet
    except Exception as e:
        health_status["services"]["textbook_content"] = f"error: {str(e)}"
        health_status["status"] = "unhealthy"

    return health_status