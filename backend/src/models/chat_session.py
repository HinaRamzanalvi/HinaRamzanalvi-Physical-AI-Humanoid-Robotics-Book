from sqlalchemy import Column, String, DateTime, JSON
from sqlalchemy.sql import func
from src.config.database import Base
from src.utils.db_utils import ID_TYPE, generate_uuid


class ChatSession(Base):
    __tablename__ = "chat_sessions"

    id = Column(ID_TYPE, primary_key=True, default=generate_uuid)
    user_id = Column(ID_TYPE, nullable=True)  # Optional: null for anonymous sessions
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
    expires_at = Column(DateTime(timezone=True))  # Expiration time for the session
    session_metadata = Column(JSON)  # Additional session metadata (user preferences, context flags)