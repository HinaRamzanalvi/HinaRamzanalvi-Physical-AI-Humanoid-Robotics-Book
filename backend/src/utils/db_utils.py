"""
Database utility functions for handling different database types
"""
from sqlalchemy import String
from sqlalchemy.dialects.postgresql import UUID
import uuid
from src.config.database import DATABASE_URL


# Determine the appropriate ID type based on the database being used
if DATABASE_URL.startswith("sqlite"):
    # For SQLite, use String type for UUID
    ID_TYPE = String(36)  # UUID is 36 characters long
    def generate_uuid():
        return str(uuid.uuid4())
else:
    # For PostgreSQL, use UUID type
    ID_TYPE = UUID(as_uuid=True)
    def generate_uuid():
        return uuid.uuid4()