"""
Script to parse and embed textbook content for the RAG Chatbot
This script reads all Markdown files from the textbook, chunks content by module/chapter/section,
generates embeddings for each chunk, stores chunks in Postgres with metadata, and stores
embeddings in Qdrant vector database.
"""
import os
import sys
import argparse
from pathlib import Path
import markdown
from bs4 import BeautifulSoup
import re
from typing import List, Dict, Any
from uuid import uuid4
import logging

# Add the src directory to the path so we can import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from src.config.database import SessionLocal, engine
from src.config.vector_db import client, QDRANT_COLLECTION_NAME
from src.models.textbook_chunk import TextbookChunk
from src.utils.logging import log_info, log_error
from src.config.settings import settings
from openai import OpenAI


def extract_textbook_structure(file_path: Path) -> List[Dict[str, Any]]:
    """
    Extract textbook structure (modules, chapters, sections) from a markdown file
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Parse markdown to HTML
    html = markdown.markdown(content)
    soup = BeautifulSoup(html, 'html.parser')

    # Extract the file path components to determine module and chapter
    relative_path = file_path.relative_to(file_path.parent)  # This will be updated based on source_dir
    path_parts = relative_path.parts

    # Default values
    module = "Unknown Module"
    chapter = "Unknown Chapter"
    section_title = "Unknown Section"

    # Try to extract module and chapter from path
    if len(path_parts) >= 2:
        module = path_parts[0].replace('_', ' ').title()
        chapter = path_parts[1].replace('.md', '').replace('_', ' ').title()
    elif len(path_parts) == 1:
        chapter = path_parts[0].replace('.md', '').replace('_', ' ').title()

    # Extract sections from the markdown content
    sections = []
    current_section = {
        'title': section_title,
        'content': '',
        'order': 0
    }

    # Find all headings to identify sections
    headings = soup.find_all(['h1', 'h2', 'h3', 'h4', 'h5', 'h6'])

    for i, heading in enumerate(headings):
        # Get content between this heading and the next
        content_parts = []
        next_heading = headings[i + 1] if i + 1 < len(headings) else None

        # Find all elements between current heading and next heading
        current = heading.next_sibling
        while current and current != next_heading:
            if hasattr(current, 'get_text'):
                content_parts.append(current.get_text())
            elif isinstance(current, str) and current.strip():
                content_parts.append(current.strip())
            current = current.next_sibling

        section_content = ' '.join(content_parts).strip()

        # Create a section entry
        section = {
            'title': heading.get_text().strip(),
            'content': f"{heading.get_text().strip()}\n\n{section_content}",
            'order': i
        }
        sections.append(section)

    # If no sections were found, use the entire content as one section
    if not sections:
        sections.append({
            'title': chapter,
            'content': content,
            'order': 0
        })

    # Add file path info to each section
    for section in sections:
        section['module'] = module
        section['chapter'] = chapter
        section['source_file'] = str(file_path)

    return sections


def chunk_text(text: str, chunk_size: int = 1000) -> List[str]:
    """
    Split text into chunks of specified size
    """
    sentences = re.split(r'[.!?]+', text)
    chunks = []
    current_chunk = ""

    for sentence in sentences:
        sentence = sentence.strip()
        if not sentence:
            continue

        if len(current_chunk) + len(sentence) < chunk_size:
            current_chunk += " " + sentence
        else:
            if current_chunk.strip():
                chunks.append(current_chunk.strip())
            current_chunk = sentence

    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks


def get_embedding(text: str) -> List[float]:
    """
    Get embedding for text using OpenAI API
    """
    client = OpenAI(api_key=settings.openai_api_key)

    response = client.embeddings.create(
        input=text,
        model="text-embedding-ada-002"
    )

    return response.data[0].embedding


def process_textbook_content(source_dir: str):
    """
    Process all textbook content from the source directory
    """
    source_path = Path(source_dir)
    if not source_path.exists():
        log_error(f"Source directory does not exist: {source_dir}")
        return

    # Get all markdown files
    markdown_files = list(source_path.rglob("*.md")) + list(source_path.rglob("*.mdx"))

    log_info(f"Found {len(markdown_files)} markdown files to process")

    # Process each markdown file
    for file_path in markdown_files:
        log_info(f"Processing file: {file_path}")

        try:
            # Extract structure from the file
            sections = extract_textbook_structure(file_path)

            # Process each section
            for i, section in enumerate(sections):
                # Chunk the content
                content_chunks = chunk_text(section['content'], settings.chunk_size)

                # Process each chunk
                for j, chunk_content in enumerate(content_chunks):
                    # Generate embedding
                    embedding = get_embedding(chunk_content)

                    # Create database record
                    db_chunk = TextbookChunk(
                        content=chunk_content,
                        module=section['module'],
                        chapter=section['chapter'],
                        section_title=section['title'],
                        source_file=str(file_path),
                        chunk_order=j
                    )

                    # Save to database
                    db = SessionLocal()
                    try:
                        db.add(db_chunk)
                        db.commit()
                        db.refresh(db_chunk)

                        # Save to Qdrant vector database
                        client.upsert(
                            collection_name=QDRANT_COLLECTION_NAME,
                            points=[{
                                "id": str(db_chunk.id),
                                "vector": embedding,
                                "payload": {
                                    "chunk_id": str(db_chunk.id),
                                    "module": section['module'],
                                    "chapter": section['chapter'],
                                    "section_title": section['title'],
                                    "source_file": str(file_path),
                                    "chunk_order": j
                                }
                            }]
                        )

                        log_info(f"Processed chunk {j+1}/{len(content_chunks)} for section '{section['title']}' in {file_path}")

                    except Exception as e:
                        db.rollback()
                        log_error(f"Error saving chunk to database: {str(e)}")
                    finally:
                        db.close()

        except Exception as e:
            log_error(f"Error processing file {file_path}: {str(e)}")
            continue

    log_info("Textbook content processing completed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process textbook content for RAG Chatbot")
    parser.add_argument("--source-dir", required=True, help="Directory containing textbook markdown files")
    parser.add_argument("--chunk-size", type=int, default=1000, help="Size of text chunks")

    args = parser.parse_args()

    # Update settings with command line args if provided
    if args.chunk_size:
        settings.chunk_size = args.chunk_size

    process_textbook_content(args.source_dir)