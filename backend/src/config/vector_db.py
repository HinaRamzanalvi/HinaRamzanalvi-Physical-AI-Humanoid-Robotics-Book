from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from typing import List, Optional, Dict, Any
import os
from dotenv import load_dotenv

load_dotenv()

# Qdrant configuration
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "textbook_chunks")

# Initialize Qdrant client
if QDRANT_URL and QDRANT_API_KEY:
    try:
        client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            timeout=30
        )
        # Test the connection
        client.get_collections()
    except Exception as e:
        print(f"Warning: Could not connect to remote Qdrant: {e}")
        print("Falling back to in-memory storage for development")
        client = QdrantClient(":memory:")  # In-memory storage for testing
else:
    # For development/local testing
    client = QdrantClient(":memory:")  # In-memory storage for testing


def create_collection_if_not_exists(
    collection_name: str = QDRANT_COLLECTION_NAME,
    vector_size: int = 1536,  # Default size for OpenAI embeddings
    distance: Distance = Distance.COSINE
) -> None:
    """
    Create the collection if it doesn't exist
    """
    try:
        # Check if collection exists
        client.get_collection(collection_name)
        # If we get here, collection exists, so we don't need to create it
    except Exception as e:
        # Check if it's a "collection already exists" error (409) or similar
        if hasattr(e, 'status_code') and e.status_code == 409:
            # Collection already exists, which is fine
            return
        elif "already exists" in str(e).lower():
            # Collection already exists, which is fine
            return
        else:
            # Create collection if it doesn't exist
            try:
                client.create_collection(
                    collection_name=collection_name,
                    vectors_config=VectorParams(size=vector_size, distance=distance),
                )
            except Exception as create_error:
                if "already exists" in str(create_error).lower():
                    # Collection was created between our check and create attempt
                    return
                else:
                    # Re-raise the actual error
                    raise create_error


def initialize_vector_db() -> None:
    """
    Initialize the vector database with required collections
    """
    create_collection_if_not_exists()


# Initialize vector database on module import
initialize_vector_db()