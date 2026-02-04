# routes/pos_router.py
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Set
import asyncio
from ..ros2_bridge import get_ros2_node
from ..app_globals import globals  # <-- necesitamos el loop principal

pos_router = APIRouter()
active_connections: Set[WebSocket] = set()

# Enviar a todos los clientes conectados
async def send_to_websockets(message: str):
    disconnected = set()
    for websocket in active_connections.copy():
        try:
            await websocket.send_text(f"{message}")  # quité el prefijo "ROS2: "
        except Exception as e:
            print(f"Error sending message: {e}")
            disconnected.add(websocket)
    active_connections.difference_update(disconnected)

# Registrar callback ROS2 -> WS en el loop correcto
def register_ros2_callback():
    ros2_node = get_ros2_node()
    if ros2_node is None:
        raise RuntimeError("ROS2 node not initialized")

    def push_to_ws(msg: str):
        loop = getattr(globals, "loop", None)
        if loop and loop.is_running():
            # Programa el coroutine en el loop principal (thread-safe)
            asyncio.run_coroutine_threadsafe(send_to_websockets(msg), loop)
        else:
            print("WARN: No running event loop to push WS message")

    ros2_node.register_callback(push_to_ws)

@pos_router.websocket("/move")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_connections.add(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            ros2_node = get_ros2_node()
            if ros2_node is not None:
                ros2_node.publish_message(data)
                await websocket.send_text(f"{data}")  # eco opcional
            else:
                await websocket.send_text("Error: ROS2 node not available")
    except WebSocketDisconnect:
        active_connections.remove(websocket)
