import unittest
from unittest.mock import Mock, patch, MagicMock
from src.services.rag_service import RAGService
from src.config.settings import settings


class TestRAGService(unittest.TestCase):
    """
    Unit tests for RAGService
    """

    def setUp(self):
        """
        Set up test fixtures before each test method.
        """
        self.rag_service = RAGService()

    @patch('src.services.rag_service.client')
    @patch('src.services.rag_service.OpenAI')
    def test_process_query_ask_book_mode(self, mock_openai, mock_qdrant_client):
        """
        Test processing a query in AskBook mode
        """
        # Mock the Qdrant search result
        mock_search_result = Mock()
        mock_search_result.id = "test_chunk_id"
        mock_search_result.payload = {
            "content": "Test textbook content",
            "module": "Test Module",
            "chapter": "Test Chapter",
            "section_title": "Test Section",
            "source_file": "test_file.md"
        }
        mock_search_result.score = 0.9
        mock_qdrant_client.search.return_value = [mock_search_result]

        # Mock the OpenAI response
        mock_openai_instance = Mock()
        mock_openai.chat.completions.create.return_value = mock_openai_instance
        mock_choice = Mock()
        mock_choice.message.content = "Test response from AI"
        mock_openai_instance.choices = [mock_choice]

        # Test the method
        result = self.rag_service.process_query(
            query_text="Test question?",
            query_mode="AskBook",
            session_id="test-session-id"
        )

        # Assertions
        self.assertEqual(result["status"], "completed")
        self.assertIn("Test response from AI", result["response_text"])
        self.assertEqual(len(result["citations"]), 1)

    @patch('src.services.rag_service.OpenAI')
    def test_process_query_ask_selected_text_mode(self, mock_openai):
        """
        Test processing a query in AskSelectedText mode
        """
        # Mock the OpenAI response
        mock_openai_instance = Mock()
        mock_openai.chat.completions.create.return_value = mock_openai_instance
        mock_choice = Mock()
        mock_choice.message.content = "Test response based on selected text"
        mock_openai_instance.choices = [mock_choice]

        # Test the method
        result = self.rag_service.process_query(
            query_text="What does this mean?",
            query_mode="AskSelectedText",
            selected_text="This is the selected text",
            session_id="test-session-id"
        )

        # Assertions
        self.assertEqual(result["status"], "completed")
        self.assertIn("Test response based on selected text", result["response_text"])

    def test_get_complexity_instruction(self):
        """
        Test the _get_complexity_instruction method
        """
        # Test beginner level
        instruction = self.rag_service._get_complexity_instruction("beginner")
        self.assertIn("beginner-friendly terms", instruction)

        # Test intermediate level
        instruction = self.rag_service._get_complexity_instruction("intermediate")
        self.assertIn("moderate technical detail", instruction)

        # Test advanced level
        instruction = self.rag_service._get_complexity_instruction("advanced")
        self.assertIn("detailed, technical explanations", instruction)

        # Test invalid level (should return default)
        instruction = self.rag_service._get_complexity_instruction("invalid")
        self.assertIn("clear explanations", instruction)

        # Test None level
        instruction = self.rag_service._get_complexity_instruction(None)
        self.assertEqual(instruction, "")


if __name__ == '__main__':
    unittest.main()