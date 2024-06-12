from .base_state import BaseState

class NavigationState(BaseState):
    async def execute(self):
        print("Executing Navigation State")
        # Implementation for navigation

    async def handle_command(self, command):
        print("Navigation State received a command")
        if command == "some_command":
            # Handle specific command
            pass
        else:
            await super().handle_command(command)

