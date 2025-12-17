from typing import List, Dict, Any, Optional
from uuid import UUID
import logging
import time
import os
from qdrant_client.http import models
from src.config.vector_db import client, QDRANT_COLLECTION_NAME
from src.config.database import SessionLocal
from src.models.textbook_chunk import TextbookChunk
from src.models.user_query import UserQuery
from src.models.chat_response import ChatResponse
from src.models.chat_session import ChatSession
from src.config.settings import settings
from src.utils.exceptions import QueryProcessingError, VectorDBError
from src.utils.logging import log_info, log_error

# Conditionally import OpenAI
try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False
    OpenAI = None

# Conditionally import Cohere
try:
    import cohere
    COHERE_AVAILABLE = True
except ImportError:
    COHERE_AVAILABLE = False
    cohere = None


class RAGService:
    def __init__(self):
        # Initialize OpenAI
        if OPENAI_AVAILABLE and settings.openai_api_key and not settings.openai_api_key.startswith("sk-xxx"):
            try:
                self.openai_client = OpenAI(api_key=settings.openai_api_key)
                self.use_openai = True
            except Exception:
                log_error("Failed to initialize OpenAI client, using mock responses")
                self.use_openai = False
                self.openai_client = None
        else:
            log_info("OpenAI not configured, using mock responses")
            self.use_openai = False
            self.openai_client = None

        # Initialize Cohere
        if COHERE_AVAILABLE and settings.cohere_api_key and not settings.cohere_api_key.startswith("xxx"):
            try:
                self.cohere_client = cohere.Client(api_key=settings.cohere_api_key)
                self.use_cohere = True
            except Exception:
                log_error("Failed to initialize Cohere client, using mock responses")
                self.use_cohere = False
                self.cohere_client = None
        else:
            log_info("Cohere not configured, using mock responses")
            self.use_cohere = False
            self.cohere_client = None

    def process_query(
        self,
        query_text: str,
        query_mode: str,
        session_id: Optional[str] = None,
        selected_text: Optional[str] = None,
        explanation_complexity: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Process a user query and return a response
        """
        start_time = time.time()
        try:
            # If session_id is not provided, create a new one
            if not session_id:
                session_id = str(self._create_new_session().id)

            # Search for relevant context based on query mode
            retrieval_start = time.time()
            if query_mode == "AskBook":
                retrieved_chunks = self._retrieve_context_from_book(query_text)
            elif query_mode == "AskSelectedText":
                if not selected_text:
                    raise QueryProcessingError("Selected text is required for AskSelectedText mode")
                retrieved_chunks = self._retrieve_context_from_selected_text(query_text, selected_text)
            else:
                raise QueryProcessingError(f"Invalid query mode: {query_mode}")
            retrieval_time = time.time() - retrieval_start

            # Generate response using the retrieved context
            generation_start = time.time()
            response_text = self._generate_response(query_text, retrieved_chunks, explanation_complexity)
            generation_time = time.time() - generation_start

            # Create citations
            citations = self._create_citations(retrieved_chunks)

            # Calculate confidence score (for now, just return a placeholder)
            confidence_score = self._calculate_confidence_score(response_text, retrieved_chunks)

            # Create response object
            response = {
                "response_text": response_text,
                "citations": citations,
                "confidence_score": confidence_score,
                "status": "completed"
            }

            total_time = time.time() - start_time

            # Log the query and response with performance metrics
            log_info(f"Processed query for session {session_id}", extra={
                "query": query_text,
                "session_id": session_id,
                "response_length": len(response_text),
                "retrieval_time": retrieval_time,
                "generation_time": generation_time,
                "total_time": total_time,
                "chunk_count": len(retrieved_chunks)
            })

            return response

        except Exception as e:
            log_error(f"Error processing query: {str(e)}", extra={
                "query": query_text,
                "session_id": session_id,
                "query_mode": query_mode
            })
            raise QueryProcessingError(f"Error processing query: {str(e)}")

    def _retrieve_context_from_book(self, query_text: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant context from the entire book
        """
        try:
            # Initialize the vector database if needed
            from src.config.vector_db import initialize_vector_db
            initialize_vector_db()

            # Get embedding for the query
            query_embedding = self._get_embedding(query_text)

            # Search in Qdrant vector database
            search_result = client.search(
                collection_name=QDRANT_COLLECTION_NAME,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True
            )

            # Get the relevant chunks
            retrieved_chunks = []
            for result in search_result:
                chunk = {
                    "id": result.id,
                    "content": result.payload.get("content", ""),
                    "module": result.payload.get("module", ""),
                    "chapter": result.payload.get("chapter", ""),
                    "section_title": result.payload.get("section_title", ""),
                    "source_file": result.payload.get("source_file", ""),
                    "relevance_score": result.score
                }
                retrieved_chunks.append(chunk)

            # If no results from vector search, try keyword search as fallback
            if not retrieved_chunks:
                log_info(f"No results from vector search, trying keyword search for query: {query_text}")
                retrieved_chunks = self._keyword_search_fallback(query_text, top_k)

            return retrieved_chunks

        except Exception as e:
            log_error(f"Error retrieving context from book: {str(e)}")
            # Try keyword search as fallback if vector search fails
            try:
                return self._keyword_search_fallback(query_text, top_k)
            except Exception as fallback_error:
                log_error(f"Keyword search fallback also failed: {str(fallback_error)}")
                return []  # Return empty list if both searches fail

    def _keyword_search_fallback(self, query_text: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Fallback method to search for content using keyword matching in the database
        """
        from sqlalchemy import func
        from src.config.database import SessionLocal
        from src.models.textbook_chunk import TextbookChunk

        db = SessionLocal()
        try:
            # Simple keyword search - look for query terms in content
            # This is a basic implementation, could be enhanced with full-text search
            search_terms = query_text.lower().split()

            # Build a query that looks for any of the terms in the content
            filter_conditions = []
            for term in search_terms:
                if len(term) > 2:  # Only search for terms longer than 2 characters
                    filter_conditions.append(func.lower(TextbookChunk.content).contains(term))

            if filter_conditions:
                # Use OR condition to match any of the terms
                combined_filter = filter_conditions[0]
                for condition in filter_conditions[1:]:
                    combined_filter = combined_filter | condition

                chunks = db.query(TextbookChunk).filter(combined_filter).limit(top_k).all()
            else:
                # If no meaningful search terms, just get some chunks
                chunks = db.query(TextbookChunk).limit(top_k).all()

            # Convert to the expected format
            retrieved_chunks = []
            for chunk in chunks:
                chunk_dict = {
                    "id": str(chunk.id),
                    "content": chunk.content,
                    "module": chunk.module,
                    "chapter": chunk.chapter,
                    "section_title": chunk.section_title,
                    "source_file": chunk.source_file,
                    "relevance_score": 0.5  # Default score for keyword matches
                }
                retrieved_chunks.append(chunk_dict)

            log_info(f"Keyword search fallback returned {len(retrieved_chunks)} chunks")
            return retrieved_chunks

        except Exception as e:
            log_error(f"Error in keyword search fallback: {str(e)}")
            return []
        finally:
            db.close()

    def _retrieve_context_from_selected_text(
        self,
        query_text: str,
        selected_text: str,
        top_k: int = 3
    ) -> List[Dict[str, Any]]:
        """
        Retrieve relevant context from the selected text only
        """
        try:
            # For selected text mode, we'll create a temporary "chunk" with the selected text
            # and use it as the context for the query
            chunk = {
                "id": "selected_text",
                "content": selected_text,
                "module": "Selected Text",
                "chapter": "User Selection",
                "section_title": "User Selected Content",
                "source_file": "user_selection",
                "relevance_score": 1.0  # Highest relevance for selected text
            }

            return [chunk]

        except Exception as e:
            log_error(f"Error retrieving context from selected text: {str(e)}")
            raise QueryProcessingError(f"Error processing selected text: {str(e)}")

    def _generate_response(self, query: str, context_chunks: List[Dict[str, Any]], explanation_complexity: Optional[str] = None) -> str:
        """
        Generate a response using OpenAI, Cohere, or mock based on availability
        """
        try:
            # Check if Cohere is available and configured
            if self.use_cohere:
                # Format the context for the LLM
                context_text = "\n\n".join([chunk["content"] for chunk in context_chunks])

                # Get complexity-specific instructions
                complexity_instruction = self._get_complexity_instruction(explanation_complexity)

                # Create the prompt
                prompt = self._build_prompt(query, context_text, complexity_instruction)

                # Call Cohere Chat API (updated method)
                # Try multiple models in case of deprecation - using current available models
                models_to_try = ["command-r-08-2024", "command-r7b-12-2024", "command-a-reasoning-08-2025", "command-a-vision-07-2025"]

                response = None
                for model in models_to_try:
                    try:
                        response = self.cohere_client.chat(
                            model=model,
                            message=prompt,
                            temperature=settings.llm_temperature,
                            max_tokens=int(settings.max_context_length/2)  # Cohere uses different max_tokens limit
                        )
                        break  # If successful, break out of the loop
                    except Exception as e:
                        # Check if this is a model-specific error
                        if "model" in str(e).lower() and ("removed" in str(e).lower() or "not found" in str(e).lower()):
                            continue  # Try the next model
                        else:
                            # If it's a different kind of error, re-raise it
                            raise e

                if response is None:
                    # If all models failed, use the last exception or return an error
                    raise Exception("All Cohere models failed. Please check your API key and available models.")

                # Extract and return the response
                return response.text.strip()
            elif self.use_openai:
                # Format the context for the LLM
                context_text = "\n\n".join([chunk["content"] for chunk in context_chunks])

                # Get complexity-specific instructions
                complexity_instruction = self._get_complexity_instruction(explanation_complexity)

                # Create the prompt
                prompt = self._build_prompt(query, context_text, complexity_instruction)

                # Call OpenAI API
                response = self.openai_client.chat.completions.create(
                    model="gpt-3.5-turbo",
                    messages=[
                        {
                            "role": "system",
                            "content": "You are an AI assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based only on the provided textbook content. Do not hallucinate or use external knowledge."
                        },
                        {
                            "role": "user",
                            "content": prompt
                        }
                    ],
                    temperature=settings.llm_temperature,
                    max_tokens=settings.max_context_length
                )

                # Extract and return the response
                return response.choices[0].message.content.strip()
            else:
                # Use mock response when neither OpenAI nor Cohere is available
                return self._generate_mock_response(query, context_chunks, explanation_complexity)

        except Exception as e:
            log_error(f"Error generating response: {str(e)}")
            raise QueryProcessingError(f"Error generating response: {str(e)}")

    def _generate_mock_response(self, query: str, context_chunks: List[Dict[str, Any]], explanation_complexity: Optional[str] = None) -> str:
        """
        Generate a mock response based on the query and context when OpenAI is not available
        """
        # Create a response based on the context and query
        if context_chunks:
            # Use the first chunk as a basis for the response
            sample_content = context_chunks[0]["content"][:200] + "..." if len(context_chunks[0]["content"]) > 200 else context_chunks[0]["content"]

            # Determine complexity level
            if explanation_complexity == "beginner":
                response = f"Based on the textbook content: {sample_content}\n\nThis is a simplified explanation for beginners. The key concept is related to your question about '{query}'."
            elif explanation_complexity == "advanced":
                response = f"Based on the textbook content: {sample_content}\n\nFor advanced understanding: This involves complex technical concepts related to your query '{query}'. The implementation requires detailed knowledge of the underlying principles."
            else:  # intermediate or default
                response = f"Based on the textbook content: {sample_content}\n\nThis explains the concepts related to your question '{query}'. The textbook provides detailed information about this topic."
        else:
            response = f"I found information related to your question '{query}' in the textbook. The RAG system has retrieved relevant content to answer your question based on the Physical AI & Humanoid Robotics textbook."

        return response

    def _get_complexity_instruction(self, explanation_complexity: Optional[str]) -> str:
        """
        Get complexity-specific instruction based on user preference
        """
        if not explanation_complexity:
            return ""

        complexity_map = {
            "beginner": "Explain concepts in beginner-friendly terms with simple language and analogies.",
            "intermediate": "Provide explanations with moderate technical detail suitable for students with some background.",
            "advanced": "Provide detailed, technical explanations with precise terminology and advanced concepts."
        }

        instruction = complexity_map.get(explanation_complexity.lower(), "Provide clear explanations appropriate for the question.")
        return f"\n{instruction}"

    def _build_prompt(self, query: str, context_text: str, complexity_instruction: str) -> str:
        """
        Build the prompt for the LLM
        """
        return f"""
            You are an AI assistant for the Physical AI & Humanoid Robotics textbook.
            Answer the user's question based ONLY on the provided textbook content.
            Do not use any external knowledge or make up information.{complexity_instruction}

            Textbook Content:
            {context_text}

            User Question: {query}

            Answer:
            """.strip()

    def _create_citations(self, retrieved_chunks: List[Dict[str, Any]]) -> List[Dict[str, str]]:
        """
        Create citations from the retrieved chunks
        """
        citations = []
        for chunk in retrieved_chunks:
            citation = {
                "module": chunk.get("module", ""),
                "chapter": chunk.get("chapter", ""),
                "section_title": chunk.get("section_title", ""),
                "source_file": chunk.get("source_file", "")
            }
            citations.append(citation)
        return citations

    def _calculate_confidence_score(self, response: str, retrieved_chunks: List[Dict[str, Any]]) -> float:
        """
        Calculate a confidence score based on the response and retrieved chunks
        """
        # For now, return a simple score based on the number of chunks used
        if len(retrieved_chunks) > 0:
            return min(0.95, 0.5 + (len(retrieved_chunks) * 0.1))
        else:
            return 0.1  # Low confidence if no chunks were retrieved

    def _get_embedding(self, text: str) -> List[float]:
        """
        Get embedding for text using OpenAI API, Cohere API, or mock embedding when neither is available
        """
        try:
            if self.use_openai:
                response = self.openai_client.embeddings.create(
                    input=text,
                    model="text-embedding-ada-002"
                )
                return response.data[0].embedding
            elif self.use_cohere:
                # Use Cohere's embed function with fallback models - using current available models
                embedding_models_to_try = ["embed-v4.0", "embed-english-v3.0", "embed-multilingual-v3.0", "embed-english-light-v3.0"]

                response = None
                for model in embedding_models_to_try:
                    try:
                        response = self.cohere_client.embed(
                            texts=[text],
                            model=model,
                            input_type="search_query"
                        )
                        break  # If successful, break out of the loop
                    except Exception as e:
                        # Check if this is a model-specific error
                        if "model" in str(e).lower() and ("removed" in str(e).lower() or "not found" in str(e).lower()):
                            continue  # Try the next model
                        else:
                            # If it's a different kind of error, re-raise it
                            raise e

                if response is None:
                    # If all models failed, use the last exception or return an error
                    raise Exception("All Cohere embedding models failed. Please check your API key and available models.")

                return response.embeddings[0]
            else:
                # Generate mock embedding when neither OpenAI nor Cohere is available
                return self._generate_mock_embedding(text)
        except Exception as e:
            log_error(f"Error getting embedding: {str(e)}")
            raise VectorDBError(f"Error getting embedding: {str(e)}")

    def _generate_mock_embedding(self, text: str) -> List[float]:
        """
        Generate a mock embedding for testing purposes when OpenAI is not available
        """
        import random

        # Create a deterministic mock embedding based on the text
        text_hash = hash(text) % (10 ** 8)
        random.seed(text_hash)  # Use the hash as seed for reproducible results

        # Generate a 1536-dimensional vector (same as OpenAI embeddings)
        embedding = [random.uniform(-1, 1) for _ in range(1536)]

        # Reset the random seed to avoid affecting other parts of the program
        random.seed()

        return embedding

    def _create_new_session(self) -> ChatSession:
        """
        Create a new chat session
        """
        db = SessionLocal()
        try:
            session = ChatSession()
            db.add(session)
            db.commit()
            db.refresh(session)
            return session
        finally:
            db.close()