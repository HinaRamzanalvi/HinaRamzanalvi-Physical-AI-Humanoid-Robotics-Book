from sqlalchemy import Column, String, DateTime, Text, Float, JSON, ForeignKey
from sqlalchemy.sql import func
from src.config.database import Base
from src.utils.db_utils import ID_TYPE, generate_uuid


class ChatResponse(Base):
    __tablename__ = "chat_responses"

    id = Column(ID_TYPE, primary_key=True, default=generate_uuid)
    query_id = Column(ID_TYPE, ForeignKey("user_queries.id"), nullable=False)  # References the user query
    response_text = Column(Text, nullable=False)  # The AI-generated response
    citations = Column(JSON)  # List of source chunks referenced in the response
    confidence_score = Column(Float, nullable=False)  # Confidence level of the response (0.0 to 1.0)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    token_usage = Column(JSON)  # Information about tokens used in generation