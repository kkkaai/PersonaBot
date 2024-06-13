import asyncio
from src.demo_main_control.states.question_response_state import QuestionResponseState

def main():
    question_response_state = QuestionResponseState('answers.json')

    # Example: Manual answer for question 1
    question_response_state.handle_command("manual_answer_1")

    # Example: Start auto answering
    asyncio.run(question_response_state.handle_command("auto_answer_start"))

    # Example: Stop auto answering
    question_response_state.handle_command("auto_answer_stop")

if __name__ == '__main__':
    main()
