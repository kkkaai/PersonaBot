import json
from .question_response_logic import QuestionResponseLogic

class QuestionResponseLogicFixed(QuestionResponseLogic):
    def __init__(self):
        super().__init__()

        self.answers_file = 'data/answers.json'
        with open(self.answers_file, 'r') as f:
            self.answers = json.load(f)


    def handle_question(self):
        if self.current_question_id is None:
            raise ValueError("No question ID set")

        answer = self.answers.get(self.current_question_id)
        if not answer:
            raise ValueError("Answer not found for question ID")

        audio_data = answer['audio']
        movement_sequence = answer['movement']

        self.play_audio(audio_data)
        self.execute_arm_movement(movement_sequence)

