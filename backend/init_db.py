"""
Initialize the database and populate with mock textbook data
"""
import os
import sys
import random

# Set the database URL before importing any modules
os.environ["DATABASE_URL"] = "sqlite:///./textbook_rag.db"
os.environ["QDRANT_COLLECTION_NAME"] = "textbook_chunks"

# Add the src directory to the path so we can import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

# Import all models to ensure they are registered with SQLAlchemy Base
from src.models.textbook_chunk import TextbookChunk
from src.models.chat_session import ChatSession
from src.models.user_query import UserQuery
from src.models.chat_response import ChatResponse

from src.config.database import engine, Base
from src.config.vector_db import initialize_vector_db
from src.scripts.mock_textbook_data import add_mock_textbook_data

def init_database():
    """
    Initialize the database and populate with mock data
    """
    print("Initializing database...")

    # Create all tables
    Base.metadata.create_all(bind=engine)
    print("Database tables created successfully")

    # Initialize vector database
    print("Initializing vector database...")
    initialize_vector_db()
    print("Vector database initialized successfully")

    # Add mock textbook data
    print("Adding mock textbook data...")
    add_mock_textbook_data()
    print("Mock textbook data added successfully")

    print("Database initialization completed!")

if __name__ == "__main__":
    init_database()