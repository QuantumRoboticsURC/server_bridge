from fastapi import APIRouter
from ..ros2_bridge import get_ros2_node

autonomous_router = APIRouter()

@autonomous_router.post("/init_state/{state}")
async def set_init_state(state: int):
    ros2_node = get_ros2_node()
    if ros2_node is not None:
        ros2_node.publish_init_state(state)
        return {"status": "success", "message": f"Init state {state} published to ROS2"}
    return {"status": "error", "message": "ROS2 node not available"}

@autonomous_router.post("/input_target/")
async def set_input_target(targets: list[float]):
    ros2_node = get_ros2_node()
    if ros2_node is not None:
        ros2_node.publish_input_target(targets)
        return {"status": "success", "message": f"Input target {targets} published to ROS2"}
    return {"status": "error", "message": "ROS2 node not available"}

@autonomous_router.get("/gps")
async def get_gps():
    ros2_node = get_ros2_node()
    if ros2_node is not None:
        return {
            "status": "success",
            "data": {
                "latitude": ros2_node._gps["latitude"],
                "longitude": ros2_node._gps["longitude"],
            }
        }
    return {"status": "error", "message": "ROS2 node not available"}