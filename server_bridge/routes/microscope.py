from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Set
import asyncio
from ..ros2_bridge import get_ros2_node
from ..app_globals import globals

microscope_router = APIRouter()
_micro_connections: Set[WebSocket] = set()


async def _micro_send_to_all(message: str):
    disconnected = set()
    for ws in _micro_connections.copy():
        try:
            await ws.send_text(message)
        except Exception as e:
            print(f"Microscope WS send error: {e}")
            disconnected.add(ws)
    _micro_connections.difference_update(disconnected)


def register_micro_ros2_callback():
    ros2_node = get_ros2_node()
    if ros2_node is None:
        raise RuntimeError("ROS2 node not initialized")

    def push_to_ws(msg: str):
        loop = getattr(globals, "loop", None)
        if loop and loop.is_running():
            asyncio.run_coroutine_threadsafe(_micro_send_to_all(msg), loop)
        else:
            print("WARN: No running event loop for microscope WS")

    ros2_node.register_micro_callback(push_to_ws)


@microscope_router.websocket("/microscope")
async def microscope_ws(websocket: WebSocket):
    await websocket.accept()
    _micro_connections.add(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            ros2_node = get_ros2_node()
            if ros2_node is not None:
                ros2_node.publish_micro_message(data)
            else:
                await websocket.send_text("Error: ROS2 node not available")
    except WebSocketDisconnect:
        _micro_connections.discard(websocket)
