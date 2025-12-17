from sqlalchemy import Column, String, DateTime, Text, Integer
from sqlalchemy.sql import func
from src.config.database import Base
from src.utils.db_utils import ID_TYPE, generate_uuid


class TextbookChunk(Base):
    __tablename__ = "textbook_chunks"

    id = Column(ID_TYPE, primary_key=True, default=generate_uuid)
    content = Column(Text, nullable=False)  # The actual text content of the chunk
    module = Column(String, nullable=False)  # The module identifier (e.g., "Module 1: ROS 2 Nervous System")
    chapter = Column(String, nullable=False)  # The chapter name within the module
    section_title = Column(String, nullable=False)  # The specific section title
    source_file = Column(String, nullable=False)  # Path to the source Markdown file
    chunk_order = Column(Integer, nullable=False, default=0)  # Order of this chunk within the section
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())