from fastapi import HTTPException, status
from typing import Optional


class RAGException(HTTPException):
    """Base exception class for RAG-related errors"""

    def __init__(self, detail: str, status_code: int = status.HTTP_500_INTERNAL_SERVER_ERROR):
        super().__init__(status_code=status_code, detail=detail)


class QueryProcessingError(RAGException):
    """Exception raised when there's an error processing a user query"""

    def __init__(self, detail: str = "Error processing query"):
        super().__init__(detail=detail, status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)


class VectorDBError(RAGException):
    """Exception raised when there's an error with the vector database"""

    def __init__(self, detail: str = "Vector database error"):
        super().__init__(detail=detail, status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)


class TextbookContentError(RAGException):
    """Exception raised when there's an error with textbook content"""

    def __init__(self, detail: str = "Error with textbook content"):
        super().__init__(detail=detail, status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)


class ConfigurationError(RAGException):
    """Exception raised when there's a configuration error"""

    def __init__(self, detail: str = "Configuration error"):
        super().__init__(detail=detail, status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)


class ValidationError(RAGException):
    """Exception raised when there's a validation error"""

    def __init__(self, detail: str = "Validation error"):
        super().__init__(detail=detail, status_code=status.HTTP_422_UNPROCESSABLE_ENTITY)


def handle_exception(exception: Exception, logger=None):
    """
    Generic exception handler that logs and returns appropriate HTTP responses
    """
    if logger:
        logger.error(f"Exception occurred: {str(exception)}", exc_info=True)

    # Re-raise as RAGException if it's not already one
    if not isinstance(exception, RAGException):
        raise RAGException(detail=str(exception))
    else:
        raise exception