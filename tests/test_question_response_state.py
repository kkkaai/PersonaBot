import unittest
import asyncio
from unittest.mock import patch

from src.demo_main_control.main_control import MainControl
from src.demo_main_control.states.question_response_state import QuestionResponseState

class TestQuestionResponseState(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        main_control = MainControl()
        cls.question_response_state = QuestionResponseState(main_control.state_control)

    @patch('your_project.question_response_logic.QuestionResponseLogicFixed.handle_question')
    def test_manual_answer_1(self, mock_handle_question):
        self.question_response_state.handle_command("manual_answer_1")
        mock_handle_question.assert_called_once()

    @patch('your_project.question_response_logic.QuestionResponseLogicDynamic.handle_question')
    def test_auto_answer_start(self, mock_handle_question):
        asyncio.run(self.question_response_state.handle_command("auto_answer_start"))
        mock_handle_question.assert_called_once()

    @patch('your_project.question_response_logic.QuestionResponseLogicDynamic.handle_question')
    def test_auto_answer_stop(self, mock_handle_question):
        self.question_response_state.handle_command("auto_answer_stop")
        self.assertIsNone(self.question_response_state.current_logic)

if __name__ == '__main__':
    unittest.main()

