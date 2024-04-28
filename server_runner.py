import asyncio
from foxglove_base_server.msg import Msg


class ServerRunner:

    def __init__(self, server_handler) -> None:
        self._server_handler = server_handler

    async def run_server(self):
        await self._server_handler.start_server()

    def run_asyncio(self):
        asyncio.run(self.run_server())

    def add_msg(self, msg: Msg):
        self._server_handler.add_msg(msg)

    def update_msg(self, msg: Msg):
        self._server_handler.update_msg(msg)

    def stop(self):
        self._server_handler.stop()