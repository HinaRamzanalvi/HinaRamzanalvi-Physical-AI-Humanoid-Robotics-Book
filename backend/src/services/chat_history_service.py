from typing import List, Dict, Any, Optional
from uuid import UUID
from sqlalchemy.orm import Session
from src.config.database import SessionLocal
from src.models.chat_session import ChatSession
from src.models.user_query import UserQuery
from src.models.chat_response import ChatResponse
from src.utils.exceptions import ValidationError
from src.utils.logging import log_info, log_error


class ChatHistoryService:
    def __init__(self):
        pass

    def create_session(self) -> ChatSession:
        """
        Create a new chat session
        """
        db = SessionLocal()
        try:
            session = ChatSession()
            db.add(session)
            db.commit()
            db.refresh(session)
            log_info(f"Created new chat session", extra={"session_id": str(session.id)})
            return session
        except Exception as e:
            db.rollback()
            log_error(f"Error creating chat session: {str(e)}")
            raise ValidationError(f"Error creating chat session: {str(e)}")
        finally:
            db.close()

    def get_session(self, session_id: UUID) -> Optional[ChatSession]:
        """
        Get a chat session by ID
        """
        db = SessionLocal()
        try:
            session = db.query(ChatSession).filter(ChatSession.id == session_id).first()
            return session
        except Exception as e:
            log_error(f"Error getting chat session: {str(e)}", extra={"session_id": str(session_id)})
            raise ValidationError(f"Error getting chat session: {str(e)}")
        finally:
            db.close()

    def get_session_history(self, session_id: UUID) -> List[Dict[str, Any]]:
        """
        Get the chat history for a session
        """
        db = SessionLocal()
        try:
            # Get all queries for this session
            queries = (
                db.query(UserQuery, ChatResponse)
                .outerjoin(ChatResponse, UserQuery.response_id == ChatResponse.id)
                .filter(UserQuery.session_id == session_id)
                .order_by(UserQuery.created_at)
                .all()
            )

            history = []
            for user_query, chat_response in queries:
                history_item = {
                    "query_id": str(user_query.id),
                    "query_text": user_query.query_text,
                    "response_text": chat_response.response_text if chat_response else None,
                    "created_at": user_query.created_at.isoformat() if user_query.created_at else None,
                    "citations": chat_response.citations if chat_response else None
                }
                history.append(history_item)

            return history

        except Exception as e:
            log_error(f"Error getting session history: {str(e)}", extra={"session_id": str(session_id)})
            raise ValidationError(f"Error getting session history: {str(e)}")
        finally:
            db.close()

    def add_query_to_session(self, session_id: UUID, query_text: str, query_mode: str, selected_text: Optional[str] = None) -> UserQuery:
        """
        Add a query to a session
        """
        db = SessionLocal()
        try:
            user_query = UserQuery(
                session_id=session_id,
                query_text=query_text,
                query_mode=query_mode,
                selected_text=selected_text,
                status="Processing"
            )
            db.add(user_query)
            db.commit()
            db.refresh(user_query)
            log_info(f"Added query to session", extra={
                "session_id": str(session_id),
                "query_id": str(user_query.id),
                "query_mode": query_mode
            })
            return user_query
        except Exception as e:
            db.rollback()
            log_error(f"Error adding query to session: {str(e)}", extra={
                "session_id": str(session_id),
                "query_text": query_text
            })
            raise ValidationError(f"Error adding query to session: {str(e)}")
        finally:
            db.close()

    def update_query_response(self, query_id: UUID, response_id: UUID) -> bool:
        """
        Update a query with its response ID
        """
        db = SessionLocal()
        try:
            user_query = db.query(UserQuery).filter(UserQuery.id == query_id).first()
            if not user_query:
                raise ValidationError(f"Query with ID {query_id} not found")

            user_query.response_id = response_id
            user_query.status = "Completed"
            db.commit()

            log_info(f"Updated query with response", extra={
                "query_id": str(query_id),
                "response_id": str(response_id)
            })

            return True
        except Exception as e:
            db.rollback()
            log_error(f"Error updating query response: {str(e)}", extra={
                "query_id": str(query_id),
                "response_id": str(response_id)
            })
            raise ValidationError(f"Error updating query response: {str(e)}")
        finally:
            db.close()

    def create_response(self, query_id: UUID, response_text: str, citations: List[Dict[str, str]], confidence_score: float) -> ChatResponse:
        """
        Create a chat response
        """
        db = SessionLocal()
        try:
            chat_response = ChatResponse(
                query_id=query_id,
                response_text=response_text,
                citations=citations,
                confidence_score=confidence_score
            )
            db.add(chat_response)
            db.commit()
            db.refresh(chat_response)
            log_info(f"Created chat response", extra={
                "query_id": str(query_id),
                "response_id": str(chat_response.id),
                "response_length": len(response_text)
            })
            return chat_response
        except Exception as e:
            db.rollback()
            log_error(f"Error creating chat response: {str(e)}", extra={
                "query_id": str(query_id)
            })
            raise ValidationError(f"Error creating chat response: {str(e)}")
        finally:
            db.close()

    def clear_session_history(self, session_id: UUID) -> bool:
        """
        Clear all chat history for a session
        """
        db = SessionLocal()
        try:
            # Delete all responses associated with queries in this session
            queries = db.query(UserQuery).filter(UserQuery.session_id == session_id).all()
            for query in queries:
                if query.response_id:
                    db.query(ChatResponse).filter(ChatResponse.id == query.response_id).delete()

            # Delete all queries in this session
            db.query(UserQuery).filter(UserQuery.session_id == session_id).delete()

            db.commit()
            log_info(f"Cleared session history", extra={"session_id": str(session_id)})
            return True
        except Exception as e:
            db.rollback()
            log_error(f"Error clearing session history: {str(e)}", extra={"session_id": str(session_id)})
            raise ValidationError(f"Error clearing session history: {str(e)}")
        finally:
            db.close()