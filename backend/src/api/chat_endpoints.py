from fastapi import APIRouter, Depends, HTTPException, status
from typing import Dict, Any, Optional
from uuid import UUID
import uuid
from pydantic import BaseModel, Field

from src.services.rag_service import RAGService
from src.services.chat_history_service import ChatHistoryService
from src.config.database import get_db
from src.utils.exceptions import RAGException, ValidationError
from src.utils.logging import log_info, log_error

router = APIRouter()


class ChatQueryRequest(BaseModel):
    query_text: str = Field(..., min_length=1, max_length=2000, description="The user's question")
    query_mode: str = Field(..., pattern="^(AskBook|AskSelectedText)$", description="Mode of the query")
    selected_text: Optional[str] = Field(None, max_length=5000, description="Text selected by user for AskSelectedText mode")
    explanation_complexity: Optional[str] = Field(None, pattern="^(beginner|intermediate|advanced)$", description="Complexity level for explanations")
    session_id: Optional[str] = Field(None, pattern="^[a-f0-9]{8}-[a-f0-9]{4}-4[a-f0-9]{3}-[89ab][a-f0-9]{3}-[a-f0-9]{12}$", description="Existing session ID, will be created if not provided")


class ChatQueryResponse(BaseModel):
    query_id: str
    session_id: str
    response_text: str
    citations: list
    confidence_score: float
    status: str


@router.post("/chat/query", response_model=Dict[str, Any], summary="Submit a query to the RAG chatbot")
async def submit_query(request: ChatQueryRequest):
    """
    Submit a query to the RAG chatbot
    """
    try:
        # Initialize services
        rag_service = RAGService()
        history_service = ChatHistoryService()

        # Validate session_id if provided
        session_id = None
        if request.session_id:
            try:
                session_id = UUID(request.session_id)
                # Verify session exists
                session = history_service.get_session(session_id)
                if not session:
                    raise ValidationError("Session not found")
            except ValueError:
                raise ValidationError("Invalid session ID format")

        # If no session_id provided, create a new session
        if not session_id:
            session = history_service.create_session()
            session_id = session.id

        # Add query to session history
        user_query = history_service.add_query_to_session(
            session_id=session_id,
            query_text=request.query_text,
            query_mode=request.query_mode,
            selected_text=request.selected_text
        )

        # Process the query using RAG service
        result = rag_service.process_query(
            query_text=request.query_text,
            query_mode=request.query_mode,
            session_id=str(session_id),
            selected_text=request.selected_text,
            explanation_complexity=request.explanation_complexity
        )

        # Create response in database
        chat_response = history_service.create_response(
            query_id=user_query.id,
            response_text=result["response_text"],
            citations=result["citations"],
            confidence_score=result["confidence_score"]
        )

        # Update query with response ID
        history_service.update_query_response(user_query.id, chat_response.id)

        # Prepare response
        response = {
            "query_id": str(user_query.id),
            "session_id": str(session_id),
            "response_text": result["response_text"],
            "citations": result["citations"],
            "confidence_score": result["confidence_score"],
            "status": result["status"]
        }

        log_info(f"Query processed successfully", extra={
            "query_id": str(user_query.id),
            "session_id": str(session_id),
            "query_mode": request.query_mode
        })

        return response

    except ValidationError as e:
        log_error(f"Validation error in chat query: {str(e)}")
        raise HTTPException(status_code=status.HTTP_422_UNPROCESSABLE_ENTITY, detail=str(e))
    except RAGException as e:
        log_error(f"RAG error in chat query: {str(e)}")
        raise HTTPException(status_code=e.status_code, detail=e.detail)
    except Exception as e:
        log_error(f"Unexpected error in chat query: {str(e)}")
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=f"Internal server error: {str(e)}")


@router.post("/ask", response_model=Dict[str, Any], summary="Submit a query to the RAG chatbot (short alias)")
async def ask_query(request: ChatQueryRequest):
    """
    Submit a query to the RAG chatbot (short alias for /chat/query)
    """
    return await submit_query(request)


@router.post("/chat", response_model=Dict[str, Any], summary="Submit a query to the RAG chatbot (short alias)")
async def chat_query(request: ChatQueryRequest):
    """
    Submit a query to the RAG chatbot (short alias for /chat/query)
    """
    return await submit_query(request)


@router.post("/query", response_model=Dict[str, Any], summary="Submit a query to the RAG chatbot (short alias)")
async def simple_query(request: ChatQueryRequest):
    """
    Submit a query to the RAG chatbot (short alias for /chat/query)
    """
    return await submit_query(request)


class GetSessionResponse(BaseModel):
    session_id: str
    queries: list
    created_at: str
    updated_at: str


@router.get("/chat/session/{session_id}", response_model=Dict[str, Any], summary="Retrieve chat history for a specific session")
async def get_session_history(session_id: str):
    """
    Retrieve chat history for a specific session
    """
    try:
        history_service = ChatHistoryService()

        # Validate session_id format
        try:
            session_uuid = UUID(session_id)
        except ValueError:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Invalid session ID format")

        # Get session to verify it exists
        session = history_service.get_session(session_uuid)
        if not session:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Session not found")

        # Get session history
        history = history_service.get_session_history(session_uuid)

        response = {
            "session_id": session_id,
            "queries": history,
            "created_at": session.created_at.isoformat() if session.created_at else None,
            "updated_at": session.updated_at.isoformat() if session.updated_at else None
        }

        log_info(f"Retrieved session history", extra={"session_id": session_id})
        return response

    except HTTPException:
        raise
    except Exception as e:
        log_error(f"Error retrieving session history: {str(e)}", extra={"session_id": session_id})
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Internal server error")


class ClearSessionResponse(BaseModel):
    session_id: str
    message: str
    cleared_at: str


@router.post("/chat/session/{session_id}/clear", response_model=Dict[str, Any], summary="Clear chat history for a specific session")
async def clear_session_history(session_id: str):
    """
    Clear chat history for a specific session
    """
    try:
        history_service = ChatHistoryService()

        # Validate session_id format
        try:
            session_uuid = UUID(session_id)
        except ValueError:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Invalid session ID format")

        # Get session to verify it exists
        session = history_service.get_session(session_uuid)
        if not session:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Session not found")

        # Clear session history
        success = history_service.clear_session_history(session_uuid)

        if success:
            from datetime import datetime
            response = {
                "session_id": session_id,
                "message": "Session history cleared successfully",
                "cleared_at": datetime.utcnow().isoformat()
            }

            log_info(f"Cleared session history", extra={"session_id": session_id})
            return response
        else:
            raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Failed to clear session history")

    except HTTPException:
        raise
    except Exception as e:
        log_error(f"Error clearing session history: {str(e)}", extra={"session_id": session_id})
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Internal server error")