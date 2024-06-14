import json
import numpy as np
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
        movement_sequence_csv = answer['movement']
        movement_fps = answer.get('fps', 20)
        
        # Load motion csv file
        with open(movement_sequence_csv, 'r') as f:
            movement_sequence = f.readlines()
        movement_sequence = [list(map(float, line.strip().split(','))) for line in movement_sequence]
        
        # Convert to numpy array
        movement_sequence = np.array(movement_sequence)

        self.play_audio(audio_data)
        self.execute_arm_movement(movement_sequence,movement_fps)
