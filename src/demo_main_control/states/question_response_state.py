import asyncio
from .base_state import BaseState
from .natural_standby_state import NaturalStandbyState

class QuestionResponseState(BaseState):
    async def execute(self):
        print("Executing Question Response State")
        # Implementation for answering questions
        # Transition to next state if needed

        await asyncio.sleep(10)  # Add a delay to prevent continuous execution

        # Example: Transition to NaturalStandbyState
        self.context.change_state(NaturalStandbyState(self.context))


    async def handle_command(self, command):
        print("Question Response State received a command")
        if command == "some_command":
            # Handle specific command
            pass
        else:
            # Delegate to parent class or handle default behavior
            await super().handle_command(command)