from typing import List, Dict, Any, Optional
from uuid import UUID
import logging
from qdrant_client.http import models
from src.config.vector_db import client, QDRANT_COLLECTION_NAME
from src.config.settings import settings
from src.utils.exceptions import VectorDBError
from src.utils.logging import log_info, log_error


class VectorStoreService:
    def __init__(self):
        self.collection_name = QDRANT_COLLECTION_NAME

    def search(self, query_vector: List[float], top_k: int = 5, filters: Optional[Dict] = None) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in the vector store
        """
        try:
            # Prepare filters if provided
            qdrant_filters = None
            if filters:
                filter_conditions = []
                for key, value in filters.items():
                    filter_conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        )
                    )

                if filter_conditions:
                    qdrant_filters = models.Filter(
                        must=filter_conditions
                    )

            # Perform search in Qdrant
            search_result = client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                query_filter=qdrant_filters,
                with_payload=True,
                with_vectors=False
            )

            # Format results
            results = []
            for result in search_result:
                formatted_result = {
                    "id": result.id,
                    "payload": result.payload,
                    "score": result.score
                }
                results.append(formatted_result)

            log_info(f"Vector search completed: found {len(results)} results", extra={
                "top_k": top_k,
                "query_length": len(query_vector)
            })

            return results

        except Exception as e:
            log_error(f"Error searching vector store: {str(e)}")
            raise VectorDBError(f"Error searching vector store: {str(e)}")

    def add_embedding(self, vector_id: str, vector: List[float], payload: Dict[str, Any]) -> bool:
        """
        Add a new embedding to the vector store
        """
        try:
            client.upsert(
                collection_name=self.collection_name,
                points=[{
                    "id": vector_id,
                    "vector": vector,
                    "payload": payload
                }]
            )

            log_info(f"Added embedding to vector store", extra={
                "vector_id": vector_id,
                "vector_length": len(vector)
            })

            return True

        except Exception as e:
            log_error(f"Error adding embedding to vector store: {str(e)}", extra={
                "vector_id": vector_id
            })
            raise VectorDBError(f"Error adding embedding to vector store: {str(e)}")

    def delete_embedding(self, vector_id: str) -> bool:
        """
        Delete an embedding from the vector store
        """
        try:
            client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=[vector_id]
                )
            )

            log_info(f"Deleted embedding from vector store", extra={
                "vector_id": vector_id
            })

            return True

        except Exception as e:
            log_error(f"Error deleting embedding from vector store: {str(e)}", extra={
                "vector_id": vector_id
            })
            raise VectorDBError(f"Error deleting embedding from vector store: {str(e)}")

    def update_embedding(self, vector_id: str, vector: Optional[List[float]] = None, payload: Optional[Dict[str, Any]] = None) -> bool:
        """
        Update an existing embedding in the vector store
        """
        try:
            # Get current point to merge with updates
            current_point = client.retrieve(
                collection_name=self.collection_name,
                ids=[vector_id]
            )

            if not current_point:
                raise VectorDBError(f"Vector with ID {vector_id} not found")

            # Prepare update
            update_payload = current_point[0].payload
            if payload:
                update_payload.update(payload)

            # Prepare vector
            update_vector = vector if vector is not None else current_point[0].vector

            # Update the point
            client.upsert(
                collection_name=self.collection_name,
                points=[{
                    "id": vector_id,
                    "vector": update_vector,
                    "payload": update_payload
                }]
            )

            log_info(f"Updated embedding in vector store", extra={
                "vector_id": vector_id
            })

            return True

        except Exception as e:
            log_error(f"Error updating embedding in vector store: {str(e)}", extra={
                "vector_id": vector_id
            })
            raise VectorDBError(f"Error updating embedding in vector store: {str(e)}")

    def get_embedding(self, vector_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a specific embedding from the vector store
        """
        try:
            points = client.retrieve(
                collection_name=self.collection_name,
                ids=[vector_id]
            )

            if points:
                point = points[0]
                return {
                    "id": point.id,
                    "vector": point.vector,
                    "payload": point.payload
                }

            return None

        except Exception as e:
            log_error(f"Error getting embedding from vector store: {str(e)}", extra={
                "vector_id": vector_id
            })
            raise VectorDBError(f"Error getting embedding from vector store: {str(e)}")

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the vector collection
        """
        try:
            collection_info = client.get_collection(self.collection_name)
            return {
                "name": collection_info.config.params.vectors.size,
                "vector_size": collection_info.config.params.vectors.size,
                "distance": collection_info.config.params.vectors.distance,
                "point_count": collection_info.point_count
            }
        except Exception as e:
            log_error(f"Error getting collection info: {str(e)}")
            raise VectorDBError(f"Error getting collection info: {str(e)}")