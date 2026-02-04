from fastapi import APIRouter

autonomous_router = APIRouter()


@autonomous_router.post("/init_state/{state}")
async def set_init_state(state: int):
    from ..ros2_bridge import get_ros2_node

    ros2_node = get_ros2_node()
    if ros2_node is not None:
        ros2_node.publish_init_state(state)
        return {"status": "success", "message": f"Init state {state} published to ROS2"}
    else:
        return {"status": "error", "message": "ROS2 node not available"}
    
@autonomous_router.post("/input_target/")
async def set_input_target(targets: list[float]):
    from ..ros2_bridge import get_ros2_node

    ros2_node = get_ros2_node()
    if ros2_node is not None:
        ros2_node.publish_input_target(targets)
        return {"status": "success", "message": f"Input target {targets} published to ROS2"}
    else:
        return {"status": "error", "message": "ROS2 node not available"}
