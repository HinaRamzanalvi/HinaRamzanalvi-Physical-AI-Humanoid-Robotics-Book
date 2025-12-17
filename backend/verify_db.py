"""
Script to verify that the database was populated correctly
"""
import os
import sys

# Set the database URL before importing any modules
os.environ["DATABASE_URL"] = "sqlite:///./textbook_rag.db"

# Add the src directory to the path so we can import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from src.config.database import SessionLocal
from src.models.textbook_chunk import TextbookChunk

def verify_db():
    """
    Verify that the database was populated correctly
    """
    print("Verifying database content...")

    db = SessionLocal()
    try:
        # Count the number of textbook chunks
        count = db.query(TextbookChunk).count()
        print(f"Number of textbook chunks in database: {count}")

        # Get a sample of chunks
        chunks = db.query(TextbookChunk).limit(5).all()
        print("\nSample textbook chunks:")
        for i, chunk in enumerate(chunks):
            print(f"{i+1}. Module: {chunk.module}")
            print(f"   Chapter: {chunk.chapter}")
            print(f"   Section: {chunk.section_title}")
            print(f"   Content preview: {chunk.content[:100]}...")
            print(f"   ID: {chunk.id}")
            print()

        if count > 0:
            print("✅ Database verification successful! Textbook content is available.")
        else:
            print("❌ Database is empty. Something went wrong during initialization.")

    except Exception as e:
        print(f"❌ Error verifying database: {e}")
        import traceback
        traceback.print_exc()
    finally:
        db.close()

if __name__ == "__main__":
    verify_db()