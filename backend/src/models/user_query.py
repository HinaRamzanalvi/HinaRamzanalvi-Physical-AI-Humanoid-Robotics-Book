from sqlalchemy import Column, String, DateTime, Text, ForeignKey, Integer, Enum
from sqlalchemy.sql import func
from src.config.database import Base
from src.utils.db_utils import ID_TYPE, generate_uuid


class UserQuery(Base):
    __tablename__ = "user_queries"

    id = Column(ID_TYPE, primary_key=True, default=generate_uuid)
    session_id = Column(ID_TYPE, ForeignKey("chat_sessions.id"), nullable=False)
    query_text = Column(Text, nullable=False)  # The user's question or input
    query_mode = Column(String(20), nullable=False)  # Mode of the query (AskBook, AskSelectedText)
    selected_text = Column(Text, nullable=True)  # Text selected by user for "Ask Selected Text" mode
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    processed_at = Column(DateTime(timezone=True), nullable=True)  # When the query was processed
    status = Column(String(20), default="Pending")  # Current status of the query
    response_id = Column(ID_TYPE, ForeignKey("chat_responses.id"), nullable=True)  # References the response if generated