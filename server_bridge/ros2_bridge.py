# ros2_bridge.py (versión con ROS timer para enviar pose a WS + Lab integration)
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from threading import Thread, Lock
from typing import List, Callable, Dict, Any, Optional
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, String, Float64MultiArray, Int8, Int32
import json
import math

class ROS2Bridge(Node):
    def __init__(self):
        super().__init__('fastapi_ros2_bridge')

        # ---------------------------------------------------------------
        # ARM Publishers
        # ---------------------------------------------------------------
        self.pubjoint1 = self.create_publisher(Float64, '/arm_teleop/joint1', 10)
        self.pubjoint2 = self.create_publisher(Float64, '/arm_teleop/joint2', 10)
        self.pubjoint3 = self.create_publisher(Float64, '/arm_teleop/joint3', 10)
        self.pubjoint4 = self.create_publisher(Float64, '/arm_teleop/joint4', 10)
        self.pubjoint5 = self.create_publisher(Float64, '/arm_teleop/servo_rotacion', 10)
        self.pubcamera = self.create_publisher(Float64, '/arm_teleop/camera1', 10)
        self.pubcamera2 = self.create_publisher(Float64, '/arm_teleop/camera2', 10)
        self.pubcamera3 = self.create_publisher(Float64, '/arm_teleop/camera3', 10)
        self.pubcamera4 = self.create_publisher(Float64, '/arm_teleop/camera4', 10)
        self.pubgripper = self.create_publisher(Float64, '/arm_teleop/gripper', 10)
        self.pub_linear_actuator = self.create_publisher(Float64, '/arm_teleop/linear_actuator', 10)

        self.pub_input_target = self.create_publisher(Float64MultiArray, '/input_target', 10)
        self.pub_init_state = self.create_publisher(Int8, '/input_state', 10)

        # Joystick
        self.subscriber_joy = self.create_subscription(Joy, "joy", self.callbackjoy, 10)

        # Mensajes cache ARM
        self.joint1_msg = Float64()
        self.joint2_msg = Float64()
        self.joint3_msg = Float64()
        self.joint4_msg = Float64()
        self.joint5_msg = Float64()
        self.camera_msg = Float64()
        self.camera2_msg = Float64()
        self.camera3_msg = Float64()
        self.camera4_msg = Float64()
        self.linear_actuator_msg = Float64()
        self.gripper_msg = Float64()
        self.init_state_msg = Int8()
        self.input_target_msg = Float64MultiArray()

        # Estado de joystick
        self.buttons: List[int] = []
        self.axes: List[float] = []

        # Pose (solo a WS, no a ROS)
        self._pose_lock = Lock()
        self.pose: Dict[str, float] = {
            "x": 0.15, "y": 0.0, "z": 0.35, "roll": 0.0, "pitch": 0.0
        }
        self._last_sent_pose: Dict[str, float] = dict(self.pose)

        # Pasos joystick
        self.linear_step = 0.003
        self.angular_step = 1.0

        # Callbacks ARM → WS
        self._callbacks: List[Callable[[str], None]] = []

        # Timer de envío pose WS
        self.pose_ws_hz = 10.0
        self.deadband_lin = 1e-4
        self.deadband_ang = 0.05
        self.quant_lin = 1e-4
        self.quant_ang = 0.01
        self.pose_pub_timer = self.create_timer(1.0 / self.pose_ws_hz, self.timer_publish_pose_ws)

        # ---------------------------------------------------------------
        # LAB Publishers (UI → ROS2) — all Float64
        # ---------------------------------------------------------------
        self.pub_elevator = self.create_publisher(Float64, '/lab/elevator', 10)
        self.pub_servo_right = self.create_publisher(Float64, '/lab/servo_right', 10)
        self.pub_servo_left = self.create_publisher(Float64, '/lab/servo_left', 10)
        self.pub_gate_left = self.create_publisher(Float64, '/lab/gate_left', 10)
        self.pub_gate_right = self.create_publisher(Float64, '/lab/gate_right', 10)
        self.pub_lab_camera = self.create_publisher(Float64, '/lab/camera_servo', 10)

        self._lab_publishers: Dict[str, Any] = {
            "elevator": self.pub_elevator,
            "servo_right": self.pub_servo_right,
            "servo_left": self.pub_servo_left,
            "gate_left": self.pub_gate_left,
            "gate_right": self.pub_gate_right,
            "lab_camera": self.pub_lab_camera,
        }

        # Mensajes cache LAB
        self._lab_msg = Float64()

        # ---------------------------------------------------------------
        # LAB Subscribers (ROS2 → UI) — gas sensors, all Float64
        # ---------------------------------------------------------------
        self._lab_callbacks: List[Callable[[str], None]] = []
        self._gas_values: Dict[str, float] = {
            "co2": 0.0, "nh3": 0.0, "alcohol": 0.0, "benzene": 0.0
        }

        self.create_subscription(Float64, '/lab/gas/co2', lambda msg: self._on_gas("co2", msg), 10)
        self.create_subscription(Float64, '/lab/gas/nh3', lambda msg: self._on_gas("nh3", msg), 10)
        self.create_subscription(Float64, '/lab/gas/alcohol', lambda msg: self._on_gas("alcohol", msg), 10)
        self.create_subscription(Float64, '/lab/gas/benzene', lambda msg: self._on_gas("benzene", msg), 10)

        # Push gas data to WS at 1 Hz
        self.create_timer(1.0, self._publish_gas_to_ws)

        self.get_logger().info("🚀 ROS2Bridge initialized (arm + lab)")

    # ---------------------------------------------------------------
    # JOYSTICK → POSE
    # ---------------------------------------------------------------
    def callbackjoy(self, msg: Joy):
        self.buttons = list(msg.buttons[:])
        self.axes = list(msg.axes[:])

        with self._pose_lock:
            x = self.pose["x"]
            y = self.pose["y"]
            z = self.pose["z"]
            roll = self.pose["roll"]
            pitch = self.pose["pitch"]

        try:
            x += self.linear_step * self._get_axis_safe(1)
            y += self.linear_step * self._get_axis_safe(3)

            lt = self._get_axis_safe(2)
            rt = self._get_axis_safe(5)
            z += (self.linear_step * (abs(rt) - 1.0)) - (self.linear_step * (abs(lt) - 1.0))

            lb = self._get_button_safe(4)
            rb = self._get_button_safe(5)
            roll += self.angular_step * (lb - rb)

            hat_y = self._get_axis_safe(7)
            pitch += self.angular_step * hat_y

        except Exception as e:
            self.get_logger().warn(f"Joy mapping error: {e}")

        self._set_pose({"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch})

    # ---------------------------------------------------------------
    # TIMER: pose → WS
    # ---------------------------------------------------------------
    def timer_publish_pose_ws(self):
        with self._pose_lock:
            payload = {
                "type": "pose",
                "data": {
                    "x": self.pose["x"],
                    "y": self.pose["y"],
                    "z": self.pose["z"],
                    "roll": self.pose["roll"],
                    "pitch": self.pose["pitch"],
                }
            }
        msg = json.dumps(payload)
        for cb in self._callbacks:
            try:
                cb(msg)
            except Exception as e:
                self.get_logger().warn(f"WS callback error: {e}")

    # ---------------------------------------------------------------
    # LAB: publish control command
    # ---------------------------------------------------------------
    def publish_lab_control(self, control_type: str, value):
        pub = self._lab_publishers.get(control_type)
        if pub is None:
            self.get_logger().warn(f"Unknown lab control: {control_type}")
            return
        self._lab_msg.data = float(value)
        pub.publish(self._lab_msg)

    # ---------------------------------------------------------------
    # LAB: gas sensor callbacks
    # ---------------------------------------------------------------
    def _on_gas(self, gas_key: str, msg):
        self._gas_values[gas_key] = msg.data

    def _publish_gas_to_ws(self):
        if not self._lab_callbacks:
            return
        payload = json.dumps({
            "type": "gas_data",
            "data": {
                "co2": round(self._gas_values["co2"], 2),
                "nh3": round(self._gas_values["nh3"], 2),
                "alcohol": round(self._gas_values["alcohol"], 2),
                "benzene": round(self._gas_values["benzene"], 2),
            }
        })
        for cb in self._lab_callbacks:
            try:
                cb(payload)
            except Exception as e:
                self.get_logger().warn(f"Lab WS callback error: {e}")

    def register_lab_callback(self, callback: Callable[[str], None]):
        self._lab_callbacks.append(callback)

    # ---------------------------------------------------------------
    # Utilities
    # ---------------------------------------------------------------
    def _get_axis_safe(self, idx: int) -> float:
        return float(self.axes[idx]) if idx < len(self.axes) else 0.0

    def _get_button_safe(self, idx: int) -> int:
        return int(self.buttons[idx]) if idx < len(self.buttons) else 0

    def _set_pose(self, p: Dict[str, Any]) -> None:
        with self._pose_lock:
            self.pose["x"] = float(p.get("x", self.pose["x"]))
            self.pose["y"] = float(p.get("y", self.pose["y"]))
            self.pose["z"] = float(p.get("z", self.pose["z"]))
            self.pose["roll"] = float(p.get("roll", self.pose["roll"]))
            self.pose["pitch"] = float(p.get("pitch", self.pose["pitch"]))

    # ---------------------------------------------------------------
    # ARM: publish helpers
    # ---------------------------------------------------------------
    def _publish_angles(self, angles: List[float]) -> None:
        if len(angles) != 5:
            raise ValueError("joint_angles requiere 5 valores (q1..q5)")
        self.joint1_msg.data, self.joint2_msg.data, self.joint3_msg.data, self.joint4_msg.data, self.joint5_msg.data = angles
        self.pubjoint1.publish(self.joint1_msg)
        self.pubjoint2.publish(self.joint2_msg)
        self.pubjoint3.publish(self.joint3_msg)
        self.pubjoint4.publish(self.joint4_msg)
        self.pubjoint5.publish(self.joint5_msg)

    def _publish_camera(self, cam: float) -> None:
        self.camera_msg.data = float(cam)
        self.pubcamera.publish(self.camera_msg)

    def _publish_camera2(self, cam: float) -> None:
        self.camera2_msg.data = float(cam)
        self.pubcamera2.publish(self.camera2_msg)

    def _publish_camera3(self, cam: float) -> None:
        self.camera3_msg.data = float(cam)
        self.pubcamera3.publish(self.camera3_msg)

    def _publish_camera4(self, cam: float) -> None:
        self.camera4_msg.data = float(cam)
        self.pubcamera4.publish(self.camera4_msg)

    def _publish_gripper(self, grip: float) -> None:
        self.gripper_msg.data = float(grip)
        self.pubgripper.publish(self.gripper_msg)

    def _publish_linear_actuator(self, value: float) -> None:
        self.linear_actuator_msg.data = float(value)
        self.pub_linear_actuator.publish(self.linear_actuator_msg)

    def _extract_angles_array(self, payload: Any) -> Optional[List[float]]:
        if isinstance(payload, dict) and "data" in payload:
            payload = payload["data"]
        if isinstance(payload, dict):
            keys = ["q1", "q2", "q3", "q4", "q5"]
            if all(k in payload for k in keys):
                return [float(payload[k]) for k in keys]
        if isinstance(payload, list) and len(payload) == 5:
            return [float(v) for v in payload]
        return None

    # ---------------------------------------------------------------
    # ARM: WebSocket → ROS2
    # ---------------------------------------------------------------
    def publish_message(self, message: str):
        try:
            data = json.loads(message)

            if isinstance(data, dict) and "type" in data:
                mtype = data["type"]
                if mtype == "joint_angles":
                    angles = self._extract_angles_array(data)
                    if angles:
                        self._publish_angles(angles)
                    else:
                        raise ValueError("Formato inválido para joint_angles")
                    return
                if mtype == "camera":
                    self._publish_camera(data.get("data"))
                    return
                if mtype == "camera2":
                    self._publish_camera2(data.get("data"))
                    return
                if mtype == "camera3":
                    self._publish_camera3(data.get("data"))
                    return
                if mtype == "camera4":
                    self._publish_camera4(data.get("data"))
                    return
                if mtype == "gripper":
                    self._publish_gripper(data.get("data"))
                    return
                if mtype == "linear_actuator":
                    self._publish_linear_actuator(data.get("data"))
                    return
                if mtype == "pose":
                    pose = data.get("data", {})
                    if isinstance(pose, dict):
                        self._set_pose(pose)
                        return
                    raise ValueError("Formato inválido para pose")

                self.get_logger().warn(f"Tipo de mensaje no soportado: {mtype}")
                return

            # Formato combinado (opcional)
            if "joint_angles" in data:
                angles = self._extract_angles_array(data["joint_angles"])
                if angles:
                    self._publish_angles(angles)
                else:
                    raise ValueError("Formato inválido para joint_angles (combinado)")
            if "camera" in data and isinstance(data["camera"], dict) and "data" in data["camera"]:
                self._publish_camera(data["camera"]["data"])
            if "camera2" in data and isinstance(data["camera2"], dict) and "data" in data["camera2"]:
                self._publish_camera2(data["camera2"]["data"])
            if "camera3" in data and isinstance(data["camera3"], dict) and "data" in data["camera3"]:
                self._publish_camera3(data["camera3"]["data"])
            if "camera4" in data and isinstance(data["camera4"], dict) and "data" in data["camera4"]:
                self._publish_camera4(data["camera4"]["data"])
            if "gripper" in data and isinstance(data["gripper"], dict) and "data" in data["gripper"]:
                self._publish_gripper(data["gripper"]["data"])
            if "pose" in data and isinstance(data["pose"], dict):
                self._set_pose(data["pose"])

        except (json.JSONDecodeError, KeyError, ValueError, TypeError) as e:
            self.get_logger().error(f'Error processing message: {e} | raw="{message}"')

    def publish_init_state(self, state: int):
        self.init_state_msg.data = int(state)
        self.pub_init_state.publish(self.init_state_msg)

    def publish_input_target(self, targets: List[float]):
        self.input_target_msg.data = targets
        self.pub_input_target.publish(self.input_target_msg)

    # ---------------------------------------------------------------
    # ARM: Push ROS2 → WS
    # ---------------------------------------------------------------
    def listener_callback(self, msg: String):
        self.get_logger().info(f'Received: "{msg.data}"')
        for cb in self._callbacks:
            cb(msg.data)

    def register_callback(self, callback: Callable[[str], None]):
        self._callbacks.append(callback)


# --- Singleton & spin ---
ros2_node: ROS2Bridge = None
executor = None

def start_ros2():
    global ros2_node, executor
    rclpy.init()
    ros2_node = ROS2Bridge()
    executor = SingleThreadedExecutor()
    executor.add_node(ros2_node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()

def stop_ros2():
    global ros2_node, executor
    if executor is not None:
        executor.shutdown()
    if ros2_node is not None:
        ros2_node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass

def get_ros2_node() -> ROS2Bridge:
    return ros2_node