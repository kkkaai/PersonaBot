class BaseState:
    def __init__(self, context):
        self.context = context

    async def execute(self):
        raise NotImplementedError

    async def handle_command(self, command):
        raise NotImplementedError
    
