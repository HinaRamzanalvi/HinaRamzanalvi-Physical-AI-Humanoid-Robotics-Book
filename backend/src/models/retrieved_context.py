from sqlalchemy import Column, String, DateTime, Float, Integer, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid
from src.config.database import Base


class RetrievedContext(Base):
    __tablename__ = "retrieved_contexts"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    query_id = Column(UUID(as_uuid=True), ForeignKey("user_queries.id"), nullable=False)  # References the user query
    chunk_id = Column(UUID(as_uuid=True), ForeignKey("textbook_chunks.id"), nullable=False)  # References the textbook chunk
    relevance_score = Column(Float, nullable=False)  # Similarity score between query and chunk (0.0 to 1.0)
    rank = Column(Integer, nullable=False)  # Rank of this chunk in the relevance order
    created_at = Column(DateTime(timezone=True), server_default=func.now())