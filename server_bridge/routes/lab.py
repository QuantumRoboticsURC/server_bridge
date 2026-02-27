# routes/lab_router.py
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Set
import asyncio
import json
from ..ros2_bridge import get_ros2_node
from ..app_globals import globals

lab_router = APIRouter()
active_connections: Set[WebSocket] = set()


# ---------------------------------------------------------------
# WebSocket → all clients
# ---------------------------------------------------------------

async def send_to_websockets(message: str):
    disconnected = set()
    for websocket in active_connections.copy():
        try:
            await websocket.send_text(message)
        except Exception as e:
            print(f"Lab WS send error: {e}")
            disconnected.add(websocket)
    active_connections.difference_update(disconnected)


# ---------------------------------------------------------------
# ROS2 callback → WebSocket (gas data)
# ---------------------------------------------------------------

def register_ros2_callback():
    ros2_node = get_ros2_node()
    if ros2_node is None:
        raise RuntimeError("ROS2 node not initialized")

    def push_to_ws(msg: str):
        loop = getattr(globals, "loop", None)
        if loop and loop.is_running():
            asyncio.run_coroutine_threadsafe(send_to_websockets(msg), loop)

    ros2_node.register_lab_callback(push_to_ws)


# ---------------------------------------------------------------
# WebSocket endpoint
# ---------------------------------------------------------------

@lab_router.websocket("/lab")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_connections.add(websocket)
    print(f"🧪 Lab client connected ({len(active_connections)} total)")

    try:
        while True:
            data = await websocket.receive_text()
            ros2_node = get_ros2_node()
            if ros2_node is None:
                await websocket.send_text(json.dumps({"error": "ROS2 node not available"}))
                continue

            try:
                msg = json.loads(data)
                msg_type = msg.get("type", "")
                value = msg.get("data", 0)

                # Publish to appropriate ROS2 topic based on message type
                if msg_type == "elevator":
                    ros2_node.publish_lab_control("elevator", value)

                elif msg_type == "servo_right":
                    ros2_node.publish_lab_control("servo_right", value)

                elif msg_type == "servo_left":
                    ros2_node.publish_lab_control("servo_left", value)

                elif msg_type == "gate_left":
                    ros2_node.publish_lab_control("gate_left", value)

                elif msg_type == "gate_right":
                    ros2_node.publish_lab_control("gate_right", value)

                elif msg_type == "lab_camera":
                    ros2_node.publish_lab_control("lab_camera", value)

                else:
                    print(f"🧪 Unknown lab message type: {msg_type}")

            except json.JSONDecodeError:
                print(f"🧪 Invalid JSON from client: {data}")

    except WebSocketDisconnect:
        pass
    finally:
        active_connections.discard(websocket)
        print(f"🧪 Lab client disconnected ({len(active_connections)} remaining)")