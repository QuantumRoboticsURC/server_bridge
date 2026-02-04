# ros2_bridge.py (versión con ROS timer para enviar pose a WS)
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from threading import Thread, Lock
from typing import List, Callable, Dict, Any, Optional
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, String, Float64MultiArray, Int8
import json
import math

class ROS2Bridge(Node):
    def __init__(self):
        super().__init__('fastapi_ros2_bridge')

        # Publishers (si mandas q1..q5/cámara/gripper desde backend)
        self.pubjoint1 = self.create_publisher(Float64, '/arm_teleop/joint1', 10)
        self.pubjoint2 = self.create_publisher(Float64, '/arm_teleop/joint2', 10)
        self.pubjoint3 = self.create_publisher(Float64, '/arm_teleop/joint3', 10)
        self.pubjoint4 = self.create_publisher(Float64, '/arm_teleop/joint4', 10)
        self.pubjoint5 = self.create_publisher(Float64, '/arm_teleop/joint5', 10)
        self.pubcamera = self.create_publisher(Float64, '/arm_teleop/camera', 10)
        self.pubgripper = self.create_publisher(Float64, '/arm_teleop/gripper', 10)

        self.pub_input_target = self.create_publisher(Float64MultiArray, '/input_target', 10)
        self.pub_init_state = self.create_publisher(Int8, '/input_state', 10)

        # Suscripción al joystick
        self.subscriber_joy = self.create_subscription(Joy, "joy", self.callbackjoy, 10)

        # Mensajes cache
        self.joint1_msg = Float64()
        self.joint2_msg = Float64()
        self.joint3_msg = Float64()
        self.joint4_msg = Float64()
        self.joint5_msg = Float64()
        self.camera_msg = Float64()
        self.gripper_msg = Float64()
        self.init_state_msg = Int8()
        self.input_target_msg = Float64MultiArray()

        # Estado de joystick
        self.buttons: List[int] = []
        self.axes: List[float] = []

        # Pose almacenada (NO se publica a ROS; solo a WS)
        self._pose_lock = Lock()
        self.pose: Dict[str, float] = {
            "x": 0.15, "y": 0.0, "z": 0.35, "roll": 0.0, "pitch": 0.0
        }

        # Última pose enviada (para deadband)
        self._last_sent_pose: Dict[str, float] = dict(self.pose)

        # Pasos de incremento (joystick)
        self.linear_step = 0.003   # m por tick
        self.angular_step = 1.0    # grados por tick

        # Callbacks para push a WebSocket (los registra el router FastAPI)
        self._callbacks: List[Callable[[str], None]] = []

        # ======== TIMER de envío WS ========
        # Frecuencia de envío (parametrizable)
        self.pose_ws_hz = 10.0  # 30 Hz (ajusta 20–60 Hz según necesites)
        self.deadband_lin = 1e-4   # m (no mandar si cambio < deadband)
        self.deadband_ang = 0.05   # deg
        self.quant_lin = 1e-4      # m (cuantiza para evitar ruido)
        self.quant_ang = 0.01      # deg

        self.pose_pub_timer = self.create_timer(1.0 / self.pose_ws_hz, self.timer_publish_pose_ws)

    # ====== JOYSTICK → POSE (solo actualiza estado; NO envía WS aquí) ======
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
            # Mapeo ejemplo (ajústalo a tu control)
            x += self.linear_step * self._get_axis_safe(1)  # LY
            y += self.linear_step * self._get_axis_safe(3)  # RX

            lt = self._get_axis_safe(2)   # LT [-1..1]
            rt = self._get_axis_safe(5)   # RT [-1..1]
            z += ( self.linear_step * (abs(rt) - 1.0) ) - ( self.linear_step * (abs(lt) - 1.0) )

            lb = self._get_button_safe(4) # LB
            rb = self._get_button_safe(5) # RB
            roll += self.angular_step * (lb - rb)

            hat_y = self._get_axis_safe(7) # D-pad vertical (1,0,-1)
            pitch += self.angular_step * hat_y

        except Exception as e:
            self.get_logger().warn(f"Joy mapping error: {e}")

        self._set_pose({"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch})

    # ====== TIMER: publica pose a WS a frecuencia fija ======
    def timer_publish_pose_ws(self):
    # Lee pose actual bajo lock
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

        # Empuja a todos los WS con los callbacks (el router lo corre en el event loop correcto)
        for cb in self._callbacks:
            try:
                cb(msg)
            except Exception as e:
                self.get_logger().warn(f"WS callback error: {e}")

    # ====== utilidades ======
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

    # ====== Helpers publicación (si decides publicar q/cam/grip desde backend) ======
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

    def _publish_gripper(self, grip: float) -> None:
        self.gripper_msg.data = float(grip)
        self.pubgripper.publish(self.gripper_msg)

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

    # ====== Entrada desde WebSocket (frontend -> backend) ======
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
                if mtype == "gripper":
                    self._publish_gripper(data.get("data"))
                    return
                if mtype == "pose":
                    pose = data.get("data", {})
                    if isinstance(pose, dict):
                        self._set_pose(pose)
                        # no hace falta re-emitir aquí; el timer lo enviará a frecuencia fija
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
            if "gripper" in data and isinstance(data["gripper"], dict) and "data" in data["gripper"]:
                self._publish_gripper(data["gripper"]["data"])
            if "pose" in data and isinstance(data["pose"], dict):
                self._set_pose(data["pose"])
                # tampoco re-emites aquí; el timer se encarga

        except (json.JSONDecodeError, KeyError, ValueError, TypeError) as e:
            self.get_logger().error(f'Error processing message: {e} | raw="{message}"')

    def publish_init_state(self, state: int):
        self.init_state_msg.data = int(state)
        self.pub_init_state.publish(self.init_state_msg)

    def publish_input_target(self, targets: List[float]):
        self.input_target_msg.data = targets
        self.pub_input_target.publish(self.input_target_msg)

    # ====== Push ROS2 -> WS (registro desde el router) ======
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
    rclpy.shutdown()

def get_ros2_node() -> ROS2Bridge:
    return ros2_node
