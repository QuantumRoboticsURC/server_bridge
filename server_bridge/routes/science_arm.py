from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Set
import asyncio
from ..ros2_bridge import get_ros2_node
from ..app_globals import globals

science_arm_router = APIRouter()
_science_connections: Set[WebSocket] = set()


async def _science_send_to_all(message: str):
    disconnected = set()
    for ws in _science_connections.copy():
        try:
            await ws.send_text(message)
        except Exception as e:
            print(f"Science arm WS send error: {e}")
            disconnected.add(ws)
    _science_connections.difference_update(disconnected)


def register_science_ros2_callback():
    ros2_node = get_ros2_node()
    if ros2_node is None:
        raise RuntimeError("ROS2 node not initialized")

    def push_to_ws(msg: str):
        loop = getattr(globals, "loop", None)
        if loop and loop.is_running():
            asyncio.run_coroutine_threadsafe(_science_send_to_all(msg), loop)
        else:
            print("WARN: No running event loop for science arm WS")

    ros2_node.register_science_callback(push_to_ws)


@science_arm_router.websocket("/science_move")
async def science_arm_ws(websocket: WebSocket):
    await websocket.accept()
    _science_connections.add(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            ros2_node = get_ros2_node()
            if ros2_node is not None:
                ros2_node.publish_science_message(data)
                await websocket.send_text(data)
            else:
                await websocket.send_text("Error: ROS2 node not available")
    except WebSocketDisconnect:
        _science_connections.discard(websocket)
