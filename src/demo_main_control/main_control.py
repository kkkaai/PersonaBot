import asyncio
from .state_control import StateControl

class MainControl:
    def __init__(self):
        self.state_control = StateControl()

    def run(self):
        asyncio.run(self.state_control.start())

    #async def run(self):
    #    await self.state_control.start()

    def shutdown(self):
        # Add shutdown procedures here
        pass