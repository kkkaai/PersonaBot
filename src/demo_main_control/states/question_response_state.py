import asyncio
from .base_state import BaseState
from .natural_standby_state import NaturalStandbyState
from src.demo_main_control.states.question_response_logic.question_response_logic_fixed import QuestionResponseLogicFixed
from src.demo_main_control.states.question_response_logic.question_response_logic_dynamic import QuestionResponseLogicDynamic


class QuestionResponseState(BaseState):
    def __init__(self, context):
        super().__init__(context)
        self.fixed_logic = QuestionResponseLogicFixed()
        self.dynamic_logic = QuestionResponseLogicDynamic()
        self.current_logic = None
        
    async def execute(self):
        print("Executing Question Response State")

        # Implementation for answering questions
        # Transition to next state if needed

        await asyncio.sleep(10)  # Add a delay to prevent continuous execution

        # Example: Transition to NaturalStandbyState
        self.context.change_state(NaturalStandbyState(self.context))


    async def handle_command(self, command):
        print("Question Response State received a command")
        if command == "auto_answer_start":
            self.current_logic = self.dynamic_logic
            await self.dynamic_logic.handle_question()
        elif command == "auto_answer_stop":
            self.current_logic = None  # Stop listening
        elif command.startswith("manual_answer_"):
            question_id = command.split("_")[-1]
            self.current_logic = self.fixed_logic
            self.set_question_id(question_id)
            self.fixed_logic.handle_question()
        else:
            # Handle other commands or delegate to parent class
            await super().handle_command(command)
