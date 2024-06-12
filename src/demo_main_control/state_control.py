import asyncio
from .states.base_state import BaseState
from .states.natural_standby_state import NaturalStandbyState
from .states.question_response_state import QuestionResponseState
from .states.pick_drop_state import PickDropState
from .states.navigation_state import NavigationState

class StateControl:
    def __init__(self):
        self.current_state = NaturalStandbyState(self)

    async def start(self):
        while True:
            await self.current_state.execute()

    def change_state(self, new_state: BaseState):
        print(f"Changing state to {new_state.__class__.__name__}")
        self.current_state = new_state

    async def handle_command(self, command):
        await self.current_state.handle_command(command)
        
