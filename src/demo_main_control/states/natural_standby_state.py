import asyncio
from .base_state import BaseState

class NaturalStandbyState(BaseState):
    async def execute(self):
        print("Executing Natural Standby State")
        # Implementation for natural standby

        # Add a delay to prevent continuous execution
        await asyncio.sleep(1)

        # Example: Transition to another state, e.g., QuestionResponseState
        # self.context.change_state(QuestionResponseState(self.context))

    async def handle_command(self, command):
        print(f"Handling command in NaturalStandbyState: {command}")
        if command == "some_command":
            # Handle specific command
            pass
        else:
            await super().handle_command(command)
