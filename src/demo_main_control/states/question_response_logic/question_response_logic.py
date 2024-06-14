class QuestionResponseLogic:
    def __init__(self, arm_controller):
        self.current_question_id = None
        self.arm_controller = arm_controller

    def set_question_id(self, question_id):
        self.current_question_id = question_id

    def handle_question(self):
        raise NotImplementedError("This method should be implemented by subclasses")

    def play_audio(self, audio_data):
        # Code to play audio
        raise NotImplementedError("This method should be implemented by subclasses")
        pass

    def execute_arm_movement(self, movement_sequence, fps=20):
        # Code to execute arm movement
        self.arm_controller.set_joint_rotations(movement_sequence, fps)
