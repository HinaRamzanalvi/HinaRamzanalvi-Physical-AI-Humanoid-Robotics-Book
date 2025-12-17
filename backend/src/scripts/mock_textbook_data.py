"""
Mock script to add sample textbook content to the vector database for testing
This creates mock embeddings to simulate the textbook content without requiring an OpenAI API key
"""
import sys
import os
from pathlib import Path
import random

# Add the src directory to the path so we can import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from src.config.database import SessionLocal
from src.config.vector_db import client, QDRANT_COLLECTION_NAME
from src.models.textbook_chunk import TextbookChunk
from src.utils.logging import log_info, log_error

def create_mock_embedding(text: str) -> list:
    """
    Create a mock embedding for testing purposes
    In a real scenario, this would call the OpenAI API to generate embeddings
    """
    # Create a deterministic mock embedding based on the text
    # This ensures consistent results for testing
    text_hash = hash(text) % (10 ** 8)
    embedding = []
    for i in range(1536):  # OpenAI embedding size
        val = ((text_hash + i) % 10000) / 5000.0 - 1.0  # Values between -1 and 1
        embedding.append(val)
    return embedding

def add_mock_textbook_data():
    """
    Add mock textbook content to both the database and vector store
    """
    # Check if data already exists to avoid duplicates
    db = SessionLocal()
    try:
        existing_count = db.query(TextbookChunk).count()
        if existing_count > 0:
            log_info(f"Found {existing_count} existing textbook chunks, skipping mock data insertion")
            return
    except Exception as e:
        log_error(f"Error checking existing data: {str(e)}")
    finally:
        db.close()

    # Sample content from the textbook
    sample_content = [
        {
            "content": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.",
            "module": "Module 1: Robotic Nervous System (ROS 2)",
            "chapter": "ROS 2 Basics",
            "section_title": "Introduction to ROS 2",
            "source_file": "book/docs/module1_ros2/ros2_basics.md"
        },
        {
            "content": "Humanoid robots are robots with a human-like body structure, typically having a head, torso, two arms, and two legs. They are designed to operate in human environments and potentially interact with humans in a natural way.",
            "module": "Module 1: Robotic Nervous System (ROS 2)",
            "chapter": "URDF for Humanoids",
            "section_title": "Understanding Humanoid Robots",
            "source_file": "book/docs/module1_ros2/urdf_humanoids.md"
        },
        {
            "content": "Digital twins are virtual replicas of physical systems that enable real-time monitoring, simulation, and optimization. In robotics, digital twins allow for testing robot behaviors in virtual environments before deployment.",
            "module": "Module 2: Digital Twin (Gazebo & Unity)",
            "chapter": "Physics Simulation",
            "section_title": "Digital Twin Concepts",
            "source_file": "book/docs/module2_digital_twin/physics_sim.md"
        },
        {
            "content": "NVIDIA Isaac is a robotics platform that provides tools for developing AI-powered robots. It includes Isaac Sim for simulation, Isaac ROS for perception, and Isaac Navigation for autonomous navigation.",
            "module": "Module 3: AI-Robot Brain (NVIDIA Isaac)",
            "chapter": "Isaac Introduction",
            "section_title": "NVIDIA Isaac Platform",
            "source_file": "book/docs/module3_ai_robot_brain/isaac_intro.md"
        },
        {
            "content": "Vision-Language-Action (VLA) models enable robots to understand human language commands and perform corresponding actions. These models combine computer vision, natural language processing, and motor control.",
            "module": "Module 4: Vision-Language-Action (VLA)",
            "chapter": "NLP to ROS",
            "section_title": "VLA Models",
            "source_file": "book/docs/module4_vla/nlp_to_ros.md"
        },
        {
            "content": "A Retrieval-Augmented Generation (RAG) chatbot combines information retrieval with language generation. It retrieves relevant documents from a knowledge base and uses them to generate contextually appropriate responses.",
            "module": "Module 5: RAG Chatbots for Robotics",
            "chapter": "RAG Chatbot Integration",
            "section_title": "RAG Architecture",
            "source_file": "book/docs/module5_rag_chatbots/index.md"
        }
    ]

    log_info(f"Adding {len(sample_content)} mock textbook chunks to the database")

    for i, chunk_data in enumerate(sample_content):
        try:
            # Generate mock embedding
            embedding = create_mock_embedding(chunk_data["content"])

            # Create database record
            db_chunk = TextbookChunk(
                content=chunk_data["content"],
                module=chunk_data["module"],
                chapter=chunk_data["chapter"],
                section_title=chunk_data["section_title"],
                source_file=chunk_data["source_file"],
                chunk_order=0
            )

            # Save to database
            db = SessionLocal()
            try:
                db.add(db_chunk)
                db.commit()
                db.refresh(db_chunk)

                # Ensure the ID is properly set after refresh
                chunk_id = getattr(db_chunk, 'id', str(i))

                # Save to Qdrant vector database (this might fail if Qdrant is not available)
                try:
                    client.upsert(
                        collection_name=QDRANT_COLLECTION_NAME,
                        points=[{
                            "id": str(chunk_id),
                            "vector": embedding,
                            "payload": {
                                "chunk_id": str(chunk_id),
                                "module": chunk_data["module"],
                                "chapter": chunk_data["chapter"],
                                "section_title": chunk_data["section_title"],
                                "source_file": chunk_data["source_file"],
                                "chunk_order": 0
                            }
                        }]
                    )
                except Exception as qdrant_error:
                    log_error(f"Error saving to Qdrant (this is OK for testing): {str(qdrant_error)}")
                    # Continue anyway, as we have the data in the database

                log_info(f"Added mock chunk {i+1}/{len(sample_content)}: {chunk_data['section_title']}")

            except Exception as e:
                db.rollback()
                log_error(f"Error saving mock chunk to database: {str(e)}")
            finally:
                db.close()

        except Exception as e:
            log_error(f"Error processing mock chunk: {str(e)}")
            continue

    log_info("Mock textbook content processing completed")

if __name__ == "__main__":
    add_mock_textbook_data()