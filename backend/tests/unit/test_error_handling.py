import unittest
from unittest.mock import Mock, patch, MagicMock, AsyncMock
import sys
import os

# Add the src directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from fastapi.testclient import TestClient
from src.main import app
from src.api.chat_endpoints import ChatQueryRequest
from src.utils.exceptions import RAGException, ValidationError
from pydantic import ValidationError as PydanticValidationError


class TestChatAPIErrorHandling(unittest.TestCase):
    """
    Unit tests for chat API error handling
    """

    def setUp(self):
        """
        Set up test client before each test method
        """
        self.client = TestClient(app)

    @patch('src.services.rag_service.RAGService')
    @patch('src.services.chat_history_service.ChatHistoryService')
    def test_chat_endpoint_validation_error(self, mock_history_service, mock_rag_service):
        """
        Test that validation errors return proper error response format
        """
        # Mock the history service to raise a validation error
        mock_history_instance = Mock()
        mock_history_instance.create_session.side_effect = ValidationError("Invalid session data")
        mock_history_service.return_value = mock_history_instance

        # Send a request with invalid data
        response = self.client.post(
            "/api/v1/chat/query",
            json={
                "query_text": "",  # Invalid - empty query
                "query_mode": "InvalidMode"  # Invalid mode
            }
        )

        # Should return 422 Unprocessable Entity with structured error
        self.assertEqual(response.status_code, 422)
        response_data = response.json()

        # Check the error response structure matches our contracts
        self.assertIn("error", response_data)
        self.assertIn("code", response_data["error"])
        self.assertIn("message", response_data["error"])
        self.assertIn("details", response_data["error"])

        # Check error code is appropriate
        self.assertEqual(response_data["error"]["code"], "CHAT_001")

    @patch('src.services.rag_service.RAGService')
    @patch('src.services.chat_history_service.ChatHistoryService')
    def test_chat_endpoint_rag_exception(self, mock_history_service, mock_rag_service):
        """
        Test that RAG exceptions return proper error response format
        """
        # Mock the RAG service to raise an exception
        mock_rag_instance = Mock()
        mock_rag_instance.process_query.side_effect = RAGException("Model unavailable", 503)
        mock_rag_service.return_value = mock_rag_instance

        # Mock the history service to return normal session
        mock_history_instance = Mock()
        mock_session = Mock()
        mock_session.id = "test-session-id"
        mock_history_instance.create_session.return_value = mock_session
        mock_history_instance.add_query_to_session.return_value = Mock(id="test-query-id")
        mock_history_instance.create_response.return_value = Mock(id="test-response-id")
        mock_history_service.return_value = mock_history_instance

        # Send a valid request that will cause RAG exception
        response = self.client.post(
            "/api/v1/chat/query",
            json={
                "query_text": "Test query",
                "query_mode": "AskBook"
            }
        )

        # Should return structured error response (status may vary depending on where error occurs in flow)
        response_data = response.json()

        # Check the error response structure matches our contracts
        self.assertIn("error", response_data)
        self.assertIn("code", response_data["error"])
        self.assertIn("message", response_data["error"])
        self.assertIn("details", response_data["error"])

    @patch('src.services.rag_service.RAGService')
    @patch('src.services.chat_history_service.ChatHistoryService')
    def test_chat_endpoint_general_exception(self, mock_history_service, mock_rag_service):
        """
        Test that general exceptions return proper error response format
        """
        # Mock the RAG service to raise a general exception
        mock_rag_instance = Mock()
        mock_rag_instance.process_query.side_effect = Exception("Unexpected error")
        mock_rag_service.return_value = mock_rag_instance

        # Mock the history service
        mock_history_instance = Mock()
        mock_session = Mock()
        mock_session.id = "test-session-id"
        mock_history_instance.create_session.return_value = mock_session
        mock_history_instance.add_query_to_session.return_value = Mock(id="test-query-id")
        mock_history_instance.create_response.return_value = Mock(id="test-response-id")
        mock_history_service.return_value = mock_history_instance

        # Send a valid request that will cause general exception
        response = self.client.post(
            "/api/v1/chat/query",
            json={
                "query_text": "Test query",
                "query_mode": "AskBook"
            }
        )

        # Should return structured error response (status may vary depending on where error occurs in flow)
        response_data = response.json()

        # Check the error response structure matches our contracts
        self.assertIn("error", response_data)
        self.assertIn("code", response_data["error"])
        self.assertIn("message", response_data["error"])
        self.assertIn("details", response_data["error"])

    def test_invalid_json_request(self):
        """
        Test that invalid JSON requests return proper error response
        """
        # Send invalid JSON
        response = self.client.post(
            "/api/v1/chat/query",
            content="{invalid json}",
            headers={"Content-Type": "application/json"}
        )

        # Should return 422 with structured error
        self.assertEqual(response.status_code, 422)
        response_data = response.json()

        # Check the error response structure
        self.assertIn("error", response_data)
        self.assertIn("code", response_data["error"])
        self.assertIn("message", response_data["error"])
        self.assertIn("details", response_data["error"])

    def test_missing_required_fields(self):
        """
        Test that requests with missing required fields return validation errors
        """
        # Send request with missing required fields
        response = self.client.post(
            "/api/v1/chat/query",
            json={
                "query_mode": "AskBook"  # Missing required query_text
            }
        )

        # Should return 422 with structured error
        self.assertEqual(response.status_code, 422)
        response_data = response.json()

        # Check the error response structure
        self.assertIn("error", response_data)
        self.assertIn("code", response_data["error"])
        self.assertIn("message", response_data["error"])
        self.assertIn("details", response_data["error"])
        self.assertIn("validation_errors", response_data["error"]["details"])

        # Check error code is appropriate
        self.assertEqual(response_data["error"]["code"], "CHAT_001")


class TestPydanticValidation(unittest.TestCase):
    """
    Unit tests for Pydantic model validation
    """

    def test_chat_query_request_validation(self):
        """
        Test validation of ChatQueryRequest model
        """
        # Test valid request
        valid_request = ChatQueryRequest(
            query_text="Test query",
            query_mode="AskBook"
        )
        self.assertEqual(valid_request.query_text, "Test query")
        self.assertEqual(valid_request.query_mode, "AskBook")

        # Test invalid query_mode
        with self.assertRaises(PydanticValidationError):
            ChatQueryRequest(
                query_text="Test query",
                query_mode="InvalidMode"
            )

        # Test empty query_text
        with self.assertRaises(PydanticValidationError):
            ChatQueryRequest(
                query_text="",
                query_mode="AskBook"
            )

        # Test too long query_text
        with self.assertRaises(PydanticValidationError):
            ChatQueryRequest(
                query_text="x" * 3000,  # Exceeds max length of 2000
                query_mode="AskBook"
            )

        # Test invalid session_id format
        with self.assertRaises(PydanticValidationError):
            ChatQueryRequest(
                query_text="Test query",
                query_mode="AskBook",
                session_id="invalid-uuid-format"
            )


if __name__ == '__main__':
    unittest.main()