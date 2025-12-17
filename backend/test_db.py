"""
Simple test to verify database models work with SQLite
"""
import os
import sys

# Set the database URL before importing any modules
os.environ["DATABASE_URL"] = "sqlite:///./textbook_rag_test.db"

# Add the src directory to the path so we can import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from src.config.database import engine, Base, SessionLocal
from src.models.textbook_chunk import TextbookChunk

def test_db():
    """
    Test if database models work correctly
    """
    print("Creating tables...")
    Base.metadata.create_all(bind=engine)
    print("Tables created successfully")

    print("Testing model creation...")
    db = SessionLocal()
    try:
        # Create a test chunk
        test_chunk = TextbookChunk(
            content="Test content for database",
            module="Test Module",
            chapter="Test Chapter",
            section_title="Test Section",
            source_file="test/file.md"
        )

        print(f"Created object with ID: {test_chunk.id}")
        print(f"ID type: {type(test_chunk.id)}")

        # Add to database
        db.add(test_chunk)
        db.commit()
        db.refresh(test_chunk)

        print(f"After commit/refresh, ID: {test_chunk.id}")
        print(f"Object saved successfully with ID: {test_chunk.id}")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        db.close()

if __name__ == "__main__":
    test_db()