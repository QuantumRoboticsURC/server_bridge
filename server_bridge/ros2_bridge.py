import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from threading import Thread, Lock
from typing import List, Callable, Dict, Any, Optional
from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Float64, String, Float64MultiArray, Int8, Int32, Int32MultiArray, Bool
from geometry_msgs.msg import Twist, Pose
import json
import math


class ROS2Bridge(Node):
    def __init__(self):
        super().__init__('fastapi_ros2_bridge')

        # ---------------------------------------------------------------
        # ARM Publishers (teleop directo — gripper, cámaras, actuador)
        # ---------------------------------------------------------------
        self.pubcamera = self.create_publisher(Float64, '/arm_teleop/camera1', 10)
        self.pubcamera2 = self.create_publisher(Float64, '/arm_teleop/camera2', 10)
        self.pubcamera3 = self.create_publisher(Float64, '/arm_teleop/camera3', 10)
        self.pubcamera4 = self.create_publisher(Float64, '/arm_teleop/camera4', 10)
        self.pubgripper = self.create_publisher(Float64, '/arm_teleop/gripper', 10)
        self.pub_linear_actuator = self.create_publisher(Float64, '/arm_teleop/elevador', 10)

        # Autonomous
        self.pub_input_target = self.create_publisher(Float64MultiArray, '/input_target', 10)
        self.pub_init_state = self.create_publisher(Int8, '/input_state', 10)

        # Mensajes cache
        self.camera_msg = Float64()
        self.camera2_msg = Float64()
        self.camera3_msg = Float64()
        self.camera4_msg = Float64()
        self.linear_actuator_msg = Float64()
        self.gripper_msg = Float64()
        self.init_state_msg = Int8()
        self.input_target_msg = Float64MultiArray()

        # ---------------------------------------------------------------
        # ARM Mode Manager — Publishers (UI → ROS2)
        # ---------------------------------------------------------------
        self.pub_arm_mode = self.create_publisher(String, '/arm_mode/set', 10)
        self.pub_arm_jog_mode = self.create_publisher(String, '/arm_jog/mode', 10)
        self.pub_cmd_vel_arm = self.create_publisher(Twist, '/cmd_vel/arm', 10)
        self.pub_arm_target = self.create_publisher(Pose, '/arm_cartesian/target', 10)
        self.pub_arm_go_home = self.create_publisher(Bool, '/arm_cartesian/go_home', 10)
        self.pub_arm_cancel = self.create_publisher(Bool, '/arm_cartesian/cancel', 10)

        # ---------------------------------------------------------------
        # ARM Mode Manager — State cache
        # ---------------------------------------------------------------
        self._arm_mode_state: Dict[str, Any] = {
            "mode": "idle",
            "jog_mode": "cartesian",
            "is_executing": False,
            "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0},
            "joints": {"q1": 0.0, "q2": 0.0, "q3": 0.0, "q4": 0.0, "q5": 0.0},
        }

        # ARM Mode Manager — Subscribers (ROS2 → UI)
        self.create_subscription(String, '/arm_mode/current',
            lambda msg: self._on_arm_status("mode", msg.data), 10)
        self.create_subscription(String, '/arm_jog/mode/current',
            lambda msg: self._on_arm_status("jog_mode", msg.data), 10)
        self.create_subscription(Bool, '/arm_cartesian/is_executing',
            lambda msg: self._on_arm_status("is_executing", msg.data), 10)
        self.create_subscription(Pose, '/arm_cartesian/current_pose',
            self._on_arm_pose, 10)

        # Feedback joints desde encoders
        self.create_subscription(Float64, '/arm_feedback/joint1_deg',
            lambda msg: self._on_arm_joint("q1", msg.data), 10)
        self.create_subscription(Float64, '/arm_feedback/joint2_deg',
            lambda msg: self._on_arm_joint("q2", msg.data), 10)
        self.create_subscription(Float64, '/arm_feedback/joint3_deg',
            lambda msg: self._on_arm_joint("q3", msg.data), 10)
        self.create_subscription(Float64, '/arm_feedback/joint4_deg',
            lambda msg: self._on_arm_joint("q4", msg.data), 10)

        # Push arm state to WS at 10 Hz
        self.create_timer(0.1, self._publish_arm_state_to_ws)

        # ---------------------------------------------------------------
        # Joystick
        # ---------------------------------------------------------------
        self.subscriber_joy = self.create_subscription(Joy, "joy", self.callbackjoy, 10)
        self.buttons: List[int] = []
        self.axes: List[float] = []

        # ---------------------------------------------------------------
        # GPS subscribers
        # ---------------------------------------------------------------
        self._gps: Dict[str, float] = {"latitude": 0.0, "longitude": 0.0}
        self.create_subscription(Float64, '/latitude',
            lambda msg: self._on_gps("latitude", msg), 10)
        self.create_subscription(Float64, '/longitude',
            lambda msg: self._on_gps("longitude", msg), 10)
        self.create_timer(0.5, self._publish_gps_to_ws)

        # ---------------------------------------------------------------
        # IMU subscriber
        # ---------------------------------------------------------------
        self._yaw = 0.0
        self.create_subscription(Imu, '/bno055/imu', self._imu_callback, 10)
        self.create_timer(0.1, self._publish_imu_to_ws)

        # ---------------------------------------------------------------
        # SCIENCE ARM Publishers (UI → ROS2)
        # ---------------------------------------------------------------
        self.pub_science_arm_mode = self.create_publisher(String, '/science_arm/mode/set', 10)
        self.pub_science_arm_jog_mode = self.create_publisher(String, '/science_arm_jog/mode', 10)
        self.pub_cmd_vel_science_arm = self.create_publisher(Twist, '/cmd_vel/science_arm', 10)
        self.pub_science_arm_target = self.create_publisher(Pose, '/science_arm_cartesian/target', 10)
        self.pub_science_arm_go_home = self.create_publisher(Bool, '/science_arm_cartesian/go_home', 10)
        self.pub_science_arm_cancel = self.create_publisher(Bool, '/science_arm_cartesian/cancel', 10)
        self.pub_drill_servo = self.create_publisher(Float64, '/science_arm_teleop/drill_servo', 10)

        # SCIENCE ARM State cache
        self._science_arm_state: Dict[str, Any] = {
            "mode": "idle",
            "jog_mode": "cartesian",
            "is_executing": False,
            "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "pitch": 0.0},
            "joints": {"q1": 0.0, "q2": 0.0, "q3": 0.0, "q4": 90.0},
        }

        # SCIENCE ARM Subscribers (ROS2 → UI)
        self.create_subscription(String, '/science_arm/mode/current',
            lambda msg: self._on_science_arm_status("mode", msg.data), 10)
        self.create_subscription(String, '/science_arm_jog/mode/current',
            lambda msg: self._on_science_arm_status("jog_mode", msg.data), 10)
        self.create_subscription(Bool, '/science_arm_cartesian/is_executing',
            lambda msg: self._on_science_arm_status("is_executing", msg.data), 10)
        self.create_subscription(Pose, '/science_arm_cartesian/current_pose',
            self._on_science_arm_pose, 10)
        self.create_subscription(Float64, '/science_arm_feedback/joint1_deg',
            lambda msg: self._on_science_arm_joint("q1", msg.data), 10)
        self.create_subscription(Float64, '/science_arm_feedback/joint2_deg',
            lambda msg: self._on_science_arm_joint("q2", msg.data), 10)
        self.create_subscription(Float64, '/science_arm_feedback/joint3_deg',
            lambda msg: self._on_science_arm_joint("q3", msg.data), 10)
        self.create_subscription(Float64, '/science_arm_feedback/joint4_deg',
            lambda msg: self._on_science_arm_joint("q4", msg.data), 10)

        self.create_timer(0.1, self._publish_science_arm_state_to_ws)

        # ---------------------------------------------------------------
        # Callbacks ARM → WS  /  LAB → WS  /  SCIENCE ARM → WS
        # ---------------------------------------------------------------
        self._callbacks: List[Callable[[str], None]] = []
        self._lab_callbacks: List[Callable[[str], None]] = []
        self._science_callbacks: List[Callable[[str], None]] = []

        # ---------------------------------------------------------------
        # LAB Publishers (UI → ROS2)
        # ---------------------------------------------------------------
        self.pub_elevator = self.create_publisher(Float64, '/lab/elevator', 10)
        self.pub_servo_right = self.create_publisher(Float64, '/lab/servo_right', 10)
        self.pub_servo_left = self.create_publisher(Float64, '/lab/servo_left', 10)
        self.pub_gate_left = self.create_publisher(Float64, '/lab/gate_left', 10)
        self.pub_gate_right = self.create_publisher(Float64, '/lab/gate_right', 10)
        self.pub_lab_camera = self.create_publisher(Float64, '/lab/camera_servo', 10)

        self._lab_publishers: Dict[str, Any] = {
            "elevator":    self.pub_elevator,
            "servo_right": self.pub_servo_right,
            "servo_left":  self.pub_servo_left,
            "gate_left":   self.pub_gate_left,
            "gate_right":  self.pub_gate_right,
            "lab_camera":  self.pub_lab_camera,
        }
        self._lab_msg = Float64()

        # LAB Subscribers — gas sensors
        self._gas_values: Dict[str, float] = {
            "co2": 0.0, "nh3": 0.0, "alcohol": 0.0, "benzene": 0.0
        }
        self.create_subscription(Float64, '/lab/gas/co2',
            lambda msg: self._on_gas("co2", msg), 10)
        self.create_subscription(Float64, '/lab/gas/nh3',
            lambda msg: self._on_gas("nh3", msg), 10)
        self.create_subscription(Float64, '/lab/gas/alcohol',
            lambda msg: self._on_gas("alcohol", msg), 10)
        self.create_subscription(Float64, '/lab/gas/benzene',
            lambda msg: self._on_gas("benzene", msg), 10)
        self.create_timer(1.0, self._publish_gas_to_ws)

        # ---------------------------------------------------------------
        # MICROSCOPE Publishers (WS → ROS2 → microscope_bridge → ESP32)
        # ---------------------------------------------------------------
        self.pub_micro_motor = self.create_publisher(
            Int32MultiArray, '/microscope/cmd/motor_move', 10)
        self.pub_micro_sample = self.create_publisher(
            Int32, '/microscope/cmd/sample', 10)
        self.pub_micro_home = self.create_publisher(
            Bool, '/microscope/cmd/home', 10)
        self.pub_micro_led = self.create_publisher(
            String, '/microscope/cmd/led', 10)
        self.pub_micro_sensors = self.create_publisher(
            Bool, '/microscope/cmd/sensors', 10)

        # MICROSCOPE State cache (microscope_bridge → ROS2 → WS)
        self._micro_state: Dict[str, Any] = {
            "motor1_pos": 0,
            "motor2_pos": 0,
            "current_sample": 1,
        }
        self._micro_sensors: Dict[str, Any] = {
            "mq135_1":  [0.0, 0.0, 0.0, 0.0, 0.0],
            "mq135_2":  [0.0, 0.0, 0.0, 0.0, 0.0],
            "mics5524": [0.0, 0.0, 0.0, 0.0],
        }
        self._micro_callbacks: List[Callable[[str], None]] = []

        # MICROSCOPE Subscribers (microscope_bridge → ROS2 → WS)
        self.create_subscription(String, '/microscope/status',
            self._on_micro_status, 10)
        self.create_subscription(Float64MultiArray, '/microscope/sensor/mq135_1',
            lambda msg: self._on_micro_sensor("mq135_1", msg.data), 10)
        self.create_subscription(Float64MultiArray, '/microscope/sensor/mq135_2',
            lambda msg: self._on_micro_sensor("mq135_2", msg.data), 10)
        self.create_subscription(Float64MultiArray, '/microscope/sensor/mics5524',
            lambda msg: self._on_micro_sensor("mics5524", msg.data), 10)
        self.create_subscription(Int32, '/microscope/state/motor1_pos',
            lambda msg: self._on_micro_state("motor1_pos", msg.data), 10)
        self.create_subscription(Int32, '/microscope/state/motor2_pos',
            lambda msg: self._on_micro_state("motor2_pos", msg.data), 10)
        self.create_subscription(Int32, '/microscope/state/current_sample',
            lambda msg: self._on_micro_state("current_sample", msg.data), 10)

        self.create_timer(0.2, self._publish_micro_state_to_ws)

        self.get_logger().info("🚀 ROS2Bridge initialized (arm mode manager + lab + imu + gps + microscope)")

    # ===================================================================
    # ARM Mode Manager — callbacks ROS2 → cache
    # ===================================================================
    def _on_arm_status(self, key: str, value: Any):
        self._arm_mode_state[key] = value

    def _on_arm_pose(self, msg: Pose):
        self._arm_mode_state["pose"] = {
            "x":     round(msg.position.x,    4),
            "y":     round(msg.position.y,    4),
            "z":     round(msg.position.z,    4),
            "roll":  round(msg.orientation.x, 4),
            "pitch": round(msg.orientation.y, 4),
        }

    def _on_arm_joint(self, joint: str, value: float):
        self._arm_mode_state["joints"][joint] = round(value, 2)

    def _publish_arm_state_to_ws(self):
        if not self._callbacks:
            return
        payload = json.dumps({
            "type": "arm_state",
            "data": self._arm_mode_state,
        })
        for cb in self._callbacks:
            try:
                cb(payload)
            except Exception as e:
                self.get_logger().warn(f"Arm state WS callback error: {e}")

    # ===================================================================
    # ARM Mode Manager — comandos UI → ROS2
    # ===================================================================
    def publish_arm_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.pub_arm_mode.publish(msg)

    def publish_arm_jog_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.pub_arm_jog_mode.publish(msg)

    def publish_cmd_vel_arm(self, vx: float, vy: float, vz: float,
                             vroll: float, vpitch: float):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.linear.y  = float(vy)
        msg.linear.z  = float(vz)
        msg.angular.x = float(vroll)
        msg.angular.y = float(vpitch)
        self.pub_cmd_vel_arm.publish(msg)

    def publish_arm_target(self, x: float, y: float, z: float,
                            roll: float, pitch: float):
        msg = Pose()
        msg.position.x    = float(x)
        msg.position.y    = float(y)
        msg.position.z    = float(z)
        msg.orientation.x = float(roll)
        msg.orientation.y = float(pitch)
        self.pub_arm_target.publish(msg)

    def publish_arm_go_home(self):
        msg = Bool()
        msg.data = True
        self.pub_arm_go_home.publish(msg)

    def publish_arm_cancel(self):
        msg = Bool()
        msg.data = True
        self.pub_arm_cancel.publish(msg)

    # ===================================================================
    # SCIENCE ARM — callbacks ROS2 → cache
    # ===================================================================
    def _on_science_arm_status(self, key: str, value: Any):
        self._science_arm_state[key] = value

    def _on_science_arm_pose(self, msg: Pose):
        self._science_arm_state["pose"] = {
            "x":     round(msg.position.x,    4),
            "y":     round(msg.position.y,    4),
            "z":     round(msg.position.z,    4),
            "pitch": round(msg.orientation.y, 4),
        }

    def _on_science_arm_joint(self, joint: str, value: float):
        self._science_arm_state["joints"][joint] = round(value, 2)

    def _publish_science_arm_state_to_ws(self):
        if not self._science_callbacks:
            return
        payload = json.dumps({
            "type": "science_arm_state",
            "data": self._science_arm_state,
        })
        for cb in self._science_callbacks:
            try:
                cb(payload)
            except Exception as e:
                self.get_logger().warn(f"Science arm state WS callback error: {e}")

    def register_science_callback(self, callback: Callable[[str], None]):
        self._science_callbacks.append(callback)

    # ===================================================================
    # SCIENCE ARM — comandos UI → ROS2
    # ===================================================================
    def publish_science_message(self, message: str):
        try:
            data = json.loads(message)
            if not isinstance(data, dict) or "type" not in data:
                return

            mtype = data["type"]

            if mtype == "science_arm_mode":
                msg = String()
                msg.data = str(data.get("data", "idle"))
                self.pub_science_arm_mode.publish(msg)
                return

            if mtype == "science_arm_jog_mode":
                msg = String()
                msg.data = str(data.get("data", "cartesian"))
                self.pub_science_arm_jog_mode.publish(msg)
                return

            if mtype == "cmd_vel_science_arm":
                d = data.get("data", {})
                msg = Twist()
                msg.linear.x  = float(d.get("vx",     0.0))
                msg.linear.y  = float(d.get("vy",     0.0))
                msg.linear.z  = float(d.get("vz",     0.0))
                msg.angular.y = float(d.get("vpitch", 0.0))
                self.pub_cmd_vel_science_arm.publish(msg)
                return

            if mtype == "science_arm_target":
                d = data.get("data", {})
                msg = Pose()
                msg.position.x    = float(d.get("x",     0.0))
                msg.position.y    = float(d.get("y",     0.0))
                msg.position.z    = float(d.get("z",     0.0))
                msg.orientation.y = float(d.get("pitch", 0.0))
                self.pub_science_arm_target.publish(msg)
                return

            if mtype == "science_arm_go_home":
                msg = Bool()
                msg.data = True
                self.pub_science_arm_go_home.publish(msg)
                return

            if mtype == "science_arm_cancel":
                msg = Bool()
                msg.data = True
                self.pub_science_arm_cancel.publish(msg)
                return

            if mtype == "science_drill_servo":
                msg = Float64()
                msg.data = float(data.get("data", 90.0))
                self.pub_drill_servo.publish(msg)
                return

            if mtype == "linear_actuator":
                self._publish_linear_actuator(data.get("data"))
                return

            self.get_logger().warn(f"Unknown science arm message type: {mtype}")

        except (json.JSONDecodeError, KeyError, ValueError, TypeError) as e:
            self.get_logger().error(f'Error processing science message: {e} | raw="{message}"')

    # ===================================================================
    # IMU → WS
    # ===================================================================
    def _imu_callback(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def _publish_imu_to_ws(self):
        if not self._callbacks:
            return
        payload = json.dumps({
            "type": "imu_data",
            "data": {"yaw": round(self._yaw, 2)}
        })
        for cb in self._callbacks:
            try:
                cb(payload)
            except Exception as e:
                self.get_logger().warn(f"IMU WS callback error: {e}")

    # ===================================================================
    # GPS → WS
    # ===================================================================
    def _on_gps(self, key: str, msg: Float64):
        self._gps[key] = msg.data

    def _publish_gps_to_ws(self):
        if not self._lab_callbacks:
            return
        payload = json.dumps({
            "type": "gps_data",
            "data": {
                "latitude":  round(self._gps["latitude"],  7),
                "longitude": round(self._gps["longitude"], 7),
            }
        })
        for cb in self._lab_callbacks:
            try:
                cb(payload)
            except Exception as e:
                self.get_logger().warn(f"GPS WS callback error: {e}")

    # ===================================================================
    # JOYSTICK → pose interna (legacy, por si se usa el joy callback)
    # ===================================================================
    def callbackjoy(self, msg: Joy):
        self.buttons = list(msg.buttons[:])
        self.axes    = list(msg.axes[:])

    # ===================================================================
    # LAB: control + gas
    # ===================================================================
    def publish_lab_control(self, control_type: str, value: Any):
        pub = self._lab_publishers.get(control_type)
        if pub is None:
            self.get_logger().warn(f"Unknown lab control: {control_type}")
            return
        self._lab_msg.data = float(value)
        pub.publish(self._lab_msg)

    def _on_gas(self, gas_key: str, msg: Float64):
        self._gas_values[gas_key] = msg.data

    def _publish_gas_to_ws(self):
        if not self._lab_callbacks:
            return
        payload = json.dumps({
            "type": "gas_data",
            "data": {
                "co2":     round(self._gas_values["co2"],     2),
                "nh3":     round(self._gas_values["nh3"],     2),
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

    # ===================================================================
    # ARM: publish helpers (cámaras, gripper, actuador — siguen igual)
    # ===================================================================
    def _publish_camera(self, cam: float):
        self.camera_msg.data = float(cam)
        self.pubcamera.publish(self.camera_msg)

    def _publish_camera2(self, cam: float):
        self.camera2_msg.data = float(cam)
        self.pubcamera2.publish(self.camera2_msg)

    def _publish_camera3(self, cam: float):
        self.camera3_msg.data = float(cam)
        self.pubcamera3.publish(self.camera3_msg)

    def _publish_camera4(self, cam: float):
        self.camera4_msg.data = float(cam)
        self.pubcamera4.publish(self.camera4_msg)

    def _publish_gripper(self, grip: float):
        self.gripper_msg.data = float(grip)
        self.pubgripper.publish(self.gripper_msg)

    def _publish_linear_actuator(self, value: float):
        self.linear_actuator_msg.data = float(value)
        self.pub_linear_actuator.publish(self.linear_actuator_msg)

    # ===================================================================
    # WebSocket → ROS2: router principal de mensajes
    # ===================================================================
    def publish_message(self, message: str):
        try:
            data = json.loads(message)
            if not isinstance(data, dict) or "type" not in data:
                return

            mtype = data["type"]

            # ── ARM Mode Manager ──
            if mtype == "arm_mode":
                self.publish_arm_mode(str(data.get("data", "idle")))
                return

            if mtype == "arm_jog_mode":
                self.publish_arm_jog_mode(str(data.get("data", "cartesian")))
                return

            if mtype == "cmd_vel_arm":
                d = data.get("data", {})
                self.publish_cmd_vel_arm(
                    d.get("vx",     0.0),
                    d.get("vy",     0.0),
                    d.get("vz",     0.0),
                    d.get("vroll",  0.0),
                    d.get("vpitch", 0.0),
                )
                return

            if mtype == "arm_target":
                d = data.get("data", {})
                self.publish_arm_target(
                    d.get("x",     0.0),
                    d.get("y",     0.0),
                    d.get("z",     0.0),
                    d.get("roll",  0.0),
                    d.get("pitch", 0.0),
                )
                return

            if mtype == "arm_go_home":
                self.publish_arm_go_home()
                return

            if mtype == "arm_cancel":
                self.publish_arm_cancel()
                return

            # ── Cámaras / gripper / actuador ──
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

            self.get_logger().warn(f"Tipo de mensaje no soportado: {mtype}")

        except (json.JSONDecodeError, KeyError, ValueError, TypeError) as e:
            self.get_logger().error(f'Error processing message: {e} | raw="{message}"')

    # ===================================================================
    # Autonomous
    # ===================================================================
    def publish_init_state(self, state: int):
        self.init_state_msg.data = int(state)
        self.pub_init_state.publish(self.init_state_msg)

    def publish_input_target(self, targets: List[float]):
        self.input_target_msg.data = targets
        self.pub_input_target.publish(self.input_target_msg)

    # ===================================================================
    # Callbacks WS ARM
    # ===================================================================
    def register_callback(self, callback: Callable[[str], None]):
        self._callbacks.append(callback)

    def listener_callback(self, msg: String):
        self.get_logger().info(f'Received: "{msg.data}"')
        for cb in self._callbacks:
            cb(msg.data)

    # ===================================================================
    # MICROSCOPE — callbacks ROS2 → cache → WS
    # ===================================================================
    def _on_micro_status(self, msg: String):
        payload = json.dumps({"type": "microscope_status", "data": msg.data})
        for cb in self._micro_callbacks:
            try:
                cb(payload)
            except Exception as e:
                self.get_logger().warn(f"Micro status WS error: {e}")

    def _on_micro_state(self, key: str, value: Any):
        self._micro_state[key] = value

    def _on_micro_sensor(self, key: str, data: Any):
        self._micro_sensors[key] = list(data)

    def _publish_micro_state_to_ws(self):
        if not self._micro_callbacks:
            return
        payload = json.dumps({
            "type": "microscope_state",
            "data": {**self._micro_state, "sensors": self._micro_sensors},
        })
        for cb in self._micro_callbacks:
            try:
                cb(payload)
            except Exception as e:
                self.get_logger().warn(f"Micro state WS error: {e}")

    def register_micro_callback(self, callback: Callable[[str], None]):
        self._micro_callbacks.append(callback)

    # ===================================================================
    # MICROSCOPE — comandos WS → ROS2
    # ===================================================================
    def publish_micro_message(self, message: str):
        try:
            data = json.loads(message)
            if not isinstance(data, dict) or "type" not in data:
                return

            mtype = data["type"]

            if mtype == "microscope_motor":
                d = data.get("data", {})
                motor = int(d.get("motor", 0))
                steps = int(d.get("steps", 0))
                if motor in (1, 2) and steps != 0:
                    msg = Int32MultiArray()
                    msg.data = [motor, steps]
                    self.pub_micro_motor.publish(msg)
                return

            if mtype == "microscope_sample":
                n = int(data.get("data", 0))
                if 1 <= n <= 10:
                    msg = Int32(); msg.data = n
                    self.pub_micro_sample.publish(msg)
                return

            if mtype == "microscope_home":
                msg = Bool(); msg.data = True
                self.pub_micro_home.publish(msg)
                return

            if mtype == "microscope_led":
                msg = String(); msg.data = str(data.get("data", "1"))
                self.pub_micro_led.publish(msg)
                return

            if mtype == "microscope_sensors":
                msg = Bool(); msg.data = bool(data.get("data", False))
                self.pub_micro_sensors.publish(msg)
                return

            self.get_logger().warn(f"Unknown microscope message: {mtype}")

        except (json.JSONDecodeError, KeyError, ValueError, TypeError) as e:
            self.get_logger().error(f'Error processing microscope message: {e}')

    # ===================================================================
    # Utilities
    # ===================================================================
    def _get_axis_safe(self, idx: int) -> float:
        return float(self.axes[idx]) if idx < len(self.axes) else 0.0

    def _get_button_safe(self, idx: int) -> int:
        return int(self.buttons[idx]) if idx < len(self.buttons) else 0


# ===================================================================
# Singleton & spin
# ===================================================================
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
